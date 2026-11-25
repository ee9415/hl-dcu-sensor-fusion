"""Summarize observed/estimated geometry and availability, not ground-truth accuracy."""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np
import cv2


def intervals(rows, predicate):
    groups = []
    start = last = None
    for row in rows:
        timestamp = row['media_time_s']
        if predicate(row['result']):
            if start is None:
                start = timestamp
            last = timestamp
        elif start is not None:
            groups.append(dict(start_s=start, end_s=last, duration_s=last-start))
            start = last = None
    if start is not None:
        groups.append(dict(start_s=start, end_s=last, duration_s=last-start))
    return sorted(groups, key=lambda item: item['duration_s'], reverse=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    args = parser.parse_args()
    rows = [json.loads(line) for line in (args.folder/'frames.jsonl').open()]
    report = json.loads((args.folder/'report.json').read_text())
    count = len(rows)
    counts = report['counts']
    summary = dict(frames=count, source_start_s=rows[0]['media_time_s'],
        source_end_s=rows[-1]['media_time_s'], duration_s=report['duration_s'],
        availability_percent={key:round(100*counts.get(key,0)/max(count,1),2) for key in
            ('boundary_valid','boundary_pair_valid','guidance_valid','guidance_display','object_frames')},
        states=report['boundary_states'], stop_line_states=report['stop_line_states'],
        invalid_pc_depth_frames=counts.get('invalid_pc_depth_frames',0),
        guidance_midpoint_violations=counts.get('guidance_midpoint_violations',0),
        longest_both_held=intervals(rows, lambda r:r['left_boundary_state']=='held' and r['right_boundary_state']=='held')[:10],
        longest_guidance_unavailable=intervals(rows, lambda r:not r['guidance_path_display_available'])[:10],
        longest_guidance_estimated=intervals(rows, lambda r:r['guidance_path_display_state']=='estimated')[:10],
        max_media_gap_s=report['max_media_gap_s'],stale_messages=report['stale_messages'])
    # Display polynomial changes in the fixed 320x180 coordinate system.
    changes = []
    for previous, current in zip(rows, rows[1:]):
        for side in ('left','right'):
            before = previous['result'].get(side+'_display_coefficients', [])
            after = current['result'].get(side+'_display_coefficients', [])
            points_before = np.asarray(previous['result'].get(side+'_display_image', []))
            points_after = np.asarray(current['result'].get(side+'_display_image', []))
            if (len(before)==3 and len(after)==3 and
                    points_before.ndim==2 and points_after.ndim==2 and
                    points_before[:,1].min() <= 140 <= points_before[:,1].max() and
                    points_after[:,1].min() <= 140 <= points_after[:,1].max()):
                y = 140.
                shift = abs(float(np.polyval(after,y)-np.polyval(before,y)))
                changes.append(dict(media_time_s=current['media_time_s'],side=side,
                    displacement_px_at_y140=shift,state=current['result'][side+'_boundary_state']))
    summary['largest_display_changes'] = sorted(changes,key=lambda row:row['displacement_px_at_y140'],reverse=True)[:15]
    rejections = Counter()
    for row in rows:
        for index, diagnostic in enumerate(row['result'].get('boundary_detection_diagnostics', [])):
            if not isinstance(diagnostic, dict):
                continue
            for reason, value in diagnostic.get('candidate_rejections', {}).items():
                rejections[('left' if index == 0 else 'right')+':'+reason] += value
    summary['candidate_rejection_counts'] = dict(rejections.most_common())
    capture = cv2.VideoCapture(str(args.folder/'ros_replay.mp4'))
    frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = capture.get(cv2.CAP_PROP_FPS)
    decoded = []
    for index in (0, frames//2, max(0,frames-1)):
        capture.set(cv2.CAP_PROP_POS_FRAMES,index)
        ok, image = capture.read()
        decoded.append(bool(ok and image.shape==(720,1280,3)))
    capture.release()
    summary['output_video'] = dict(frames=frames, fps=fps,
        duration_s=frames/fps if fps else None, sample_decode_ok=all(decoded))
    (args.folder/'analysis.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    main()
