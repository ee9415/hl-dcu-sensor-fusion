"""Capture a short paired RTSP sequence and telemetry for manual frame review.

Receive-time pairing is approximate: independent RTSP decoders do not expose
the device frame sequence here. Never treat pairs as exact synchronization.
"""
import argparse
import json
import os
from pathlib import Path
import socket
import threading
import time

os.environ.setdefault('OPENCV_FFMPEG_CAPTURE_OPTIONS', 'rtsp_transport;tcp')
import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--device-ip', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=5)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    frames = {'rgb': [], 'result': []}
    telemetry = []
    errors = []
    ready = {name: threading.Event() for name in frames}
    stop = threading.Event()
    window = [float('inf'), float('inf')]

    def read_video(name, port, route):
        cap = cv2.VideoCapture(f'rtsp://{args.device_ip}:{port}/{route}', cv2.CAP_FFMPEG)
        try:
            if not cap.isOpened():
                errors.append(name + ': connection failed')
                return
            count = 0
            while not stop.is_set():
                ok, frame = cap.read()
                now = time.monotonic()
                if not ok:
                    errors.append(name + ': read failed')
                    break
                count += 1
                if count >= 15:
                    ready[name].set()
                if window[0] <= now < window[1]:
                    frames[name].append((now, frame))
        finally:
            cap.release()

    def read_telemetry():
        try:
            with socket.create_connection((args.device_ip, 8557), timeout=10) as conn:
                conn.settimeout(10)
                conn.sendall(b'HELLO\n')
                with conn.makefile('rb') as stream:
                    while not stop.is_set():
                        line = stream.readline()
                        if not line:
                            break
                        now = time.monotonic()
                        data = json.loads(line)
                        if window[0] <= now < window[1] and 'frame_seq' in data:
                            telemetry.append(dict(receive_s=now-window[0], **data))
        except Exception as exc:
            errors.append('telemetry: ' + str(exc))

    threads = [threading.Thread(target=read_video, args=(name, port, route), daemon=True)
               for name, port, route in [('rgb', 8555, 'rgb'), ('result', 8554, 'yolopv2')]]
    threads.append(threading.Thread(target=read_telemetry, daemon=True))
    for thread in threads:
        thread.start()
    if not all(event.wait(25) for event in ready.values()):
        stop.set()
        raise RuntimeError('Video warmup failed: ' + repr(errors))
    window[0] = time.monotonic()
    window[1] = window[0] + args.seconds
    stop.wait(args.seconds)
    stop.set()
    for thread in threads:
        thread.join(timeout=3)
    manifest = dict(seconds=args.seconds, pairing='nearest_receive_time_not_device_sync',
                    errors=errors, frames={}, pairs=[])
    for name, samples in frames.items():
        (args.output/name).mkdir()
        manifest['frames'][name] = []
        for index, (timestamp, frame) in enumerate(samples):
            path = f'{name}/{index:03d}.jpg'
            if not cv2.imwrite(str(args.output/path), frame, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                raise RuntimeError('Cannot save ' + path)
            manifest['frames'][name].append(dict(index=index, receive_s=timestamp-window[0], path=path))
    tiles = []
    for index, (timestamp, result) in enumerate(frames['result']):
        nearest = min(range(len(frames['rgb'])), key=lambda i: abs(frames['rgb'][i][0]-timestamp))
        rgb_time, rgb = frames['rgb'][nearest]
        manifest['pairs'].append(dict(result=index, rgb=nearest, receive_delta_ms=(rgb_time-timestamp)*1000))
        tile = np.zeros((250, 800, 3), np.uint8)
        tile[25:] = np.hstack([cv2.resize(rgb, (400, 225)), cv2.resize(result, (400, 225))])
        cv2.putText(tile, f'{index:03d}  t={timestamp-window[0]:.3f}s    RGB | RESULT',
                    (8, 18), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
        tiles.append(tile)
    for offset in range(0, len(tiles), 8):
        cv2.imwrite(str(args.output/f'sheet_{offset//8:02d}.jpg'), np.vstack(tiles[offset:offset+8]))
    (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2))
    (args.output/'telemetry.jsonl').write_text(''.join(json.dumps(frame)+'\n' for frame in telemetry))
    print(json.dumps(dict(frames={key:len(value) for key,value in frames.items()},
                          telemetry=len(telemetry), errors=errors)), flush=True)


if __name__ == '__main__':
    main()
