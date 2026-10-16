"""Read actual RTSP outputs; do not infer vehicle maneuver validity."""
import os
from pathlib import Path
import cv2
import numpy as np
os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'rtsp_transport;tcp'
root = Path(__file__).resolve().parents[3]/'artifacts/turn_preview_20261016'
root.mkdir(exist_ok=True, parents=True)
for name, port, stream in (('result', 8554, 'yolopv2'), ('bev', 8553, 'bev')):
    cap = cv2.VideoCapture(f'rtsp://192.168.0.4:{port}/{stream}', cv2.CAP_FFMPEG)
    if not cap.isOpened():
        raise RuntimeError(f'{name} stream unavailable')
    for _ in range(20):
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError(f'{name} frame unavailable')
    cv2.imwrite(str(root/f'live_{name}.jpg'), frame)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    purple = cv2.inRange(hsv, (130, 100, 100), (170, 255, 255))
    purple[:60] = 0
    print(name, 'purple_path_pixels', int(np.count_nonzero(purple)), flush=True)
    cap.release()
