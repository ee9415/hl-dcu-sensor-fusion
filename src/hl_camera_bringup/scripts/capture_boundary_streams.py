"""Read-only RTSP snapshots for deployment verification; never starts an app."""
import argparse
import json
import os
from pathlib import Path

os.environ.setdefault('OPENCV_FFMPEG_CAPTURE_OPTIONS', 'rtsp_transport;tcp')
import cv2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--device-ip', default='192.168.0.4')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    results = {}
    for name, port, path in (('rgb',8555,'rgb'), ('result',8554,'yolopv2'),
                             ('depth',8556,'depth'), ('bev',8553,'bev')):
        cap = cv2.VideoCapture(f'rtsp://{args.device_ip}:{port}/{path}', cv2.CAP_FFMPEG)
        try:
            ok, frame = False, None
            for _ in range(3):
                ok, frame = cap.read()
                if not ok:
                    break
            saved = bool(ok and cv2.imwrite(str(args.output/f'{name}.jpg'), frame))
            results[name] = dict(received=bool(ok), saved=saved,
                                 shape=list(frame.shape) if ok else None)
        finally:
            cap.release()
    (args.output/'streams.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
    print(json.dumps(results), flush=True)
    return 0 if all(r['saved'] for r in results.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
