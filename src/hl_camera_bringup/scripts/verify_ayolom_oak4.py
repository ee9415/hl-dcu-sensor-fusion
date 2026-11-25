"""Verify actual OAK4 A-YOLOM frame bundles, geometry and completion cadence."""
import argparse
from collections import Counter
import json
from pathlib import Path
import socket
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'apps/yolopv2_standalone'))
from perception_wire import receive_bundle
from host_postprocess import validate_frame


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device-ip', required=True)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rows, errors = [], []
    counts = Counter()
    started = time.monotonic()
    last_session = None
    try:
        with socket.create_connection((args.device_ip, 8560), timeout=10) as connection:
            connection.settimeout(10)
            connection.sendall(b'OAK_PERCEPTION_1\n')
            while time.monotonic()-started < args.seconds:
                metadata, arrays = receive_bundle(connection)
                validate_frame(metadata, arrays)
                if metadata['model_size'] != [672, 384] or metadata['source'].get('model_family') != 'ayolom_v4':
                    raise ValueError('Camera is not exporting the requested A-YOLOM 672x384 pipeline')
                if arrays['lane'].shape != (180, 320) or arrays['drivable'].shape != (180, 320):
                    raise ValueError('Downstream mask coordinates changed')
                if last_session is not None and last_session != metadata['session']:
                    raise ValueError('Inference pipeline restarted during verification')
                last_session = metadata['session']
                for obj in metadata['objects']:
                    if obj['label'] not in range(5):
                        raise ValueError('Unexpected A-YOLOM class ID')
                    counts[metadata['source']['class_names'][obj['label']]] += 1
                rows.append(dict(frame_seq=metadata['frame_seq'], nn_seq=metadata['nn_seq'],
                    timestamp_s=metadata['capture_timestamp_s'], arrival_s=time.monotonic(),
                    export_seq=metadata['export_seq'], source=metadata['source'].get('input_source'),
                    road_pixels=int(np.count_nonzero(arrays['drivable'])),
                    lane_pixels=int(np.count_nonzero(arrays['lane'])), objects=len(metadata['objects']),
                    valid_depth_pixels=int(np.count_nonzero(arrays['depth'])),
                    depth_sync_s=abs(metadata['capture_timestamp_s']-metadata['depth_timestamp_s'])))
                if len(rows) == 1:
                    (args.output/'first_metadata.json').write_text(json.dumps(metadata, indent=2))
                    np.savez_compressed(args.output/'first_frame.npz', **arrays)
                    cv2.imwrite(str(args.output/'rgb.png'), arrays['rgb'])
                    cv2.imwrite(str(args.output/'drivable.png'), arrays['drivable'])
                    cv2.imwrite(str(args.output/'lane.png'), arrays['lane'])
    except (OSError, ValueError, EOFError) as exc:
        errors.append(f'{type(exc).__name__}: {exc}')
    report = dict(device_ip=args.device_ip, elapsed_s=time.monotonic()-started,
                  frames=len(rows), errors=errors, classes=dict(counts),
                  model_family='ayolom_v4', model_input_size=[672,384],
                  passed=len(rows) >= 10 and not errors)
    if len(rows) > 1:
        steady = rows[5:] if len(rows) > 10 else rows
        report.update(unique_sequences=len({r['nn_seq'] for r in rows}),
            sequence_mismatches=sum(r['frame_seq'] != r['nn_seq'] for r in rows),
            export_sequence_gaps=sum(max(0,b['export_seq']-a['export_seq']-1) for a,b in zip(rows,rows[1:])),
            arrival_fps=(len(steady)-1)/(steady[-1]['arrival_s']-steady[0]['arrival_s']),
            capture_fps=(len(steady)-1)/(steady[-1]['timestamp_s']-steady[0]['timestamp_s']),
            road_pixel_range=[min(r['road_pixels'] for r in rows),max(r['road_pixels'] for r in rows)],
            lane_pixel_range=[min(r['lane_pixels'] for r in rows),max(r['lane_pixels'] for r in rows)],
            max_depth_sync_s=max(r['depth_sync_s'] for r in rows))
        report['passed'] &= report['unique_sequences'] == len(rows) and report['sequence_mismatches'] == 0
    (args.output/'frames.json').write_text(json.dumps(rows, indent=2))
    (args.output/'report.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
