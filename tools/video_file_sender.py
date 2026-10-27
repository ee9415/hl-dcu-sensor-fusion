"""Paced PC video replay to the OAK's bounded JPEG receiver; no local inference."""
import math
from pathlib import Path
import socket
import sys
import threading
import time

import cv2
import numpy as np

APP = Path(__file__).resolve().parents[1] / 'apps' / 'yolopv2_standalone'
if str(APP) not in sys.path:
    sys.path.insert(0, str(APP))
from video_transport import PORT, read_message, send_message


def parse_video_time(value):
    """Accept seconds, mm:ss or hh:mm:ss; reject ambiguous/nonfinite times."""
    parts = str(value).strip().split(':')
    if not 1 <= len(parts) <= 3 or any(not p.strip() for p in parts):
        raise ValueError('시간은 초 또는 분:초 형식으로 입력해 주세요')
    values = [float(p) for p in parts]
    if any(not math.isfinite(v) or v < 0 for v in values):
        raise ValueError('시간은 0 이상의 유한한 값이어야 합니다')
    if len(values) > 1 and (any(v >= 60 for v in values[1:])
                            or any(not v.is_integer() for v in values[:-1])):
        raise ValueError('분:초의 초와 시:분:초의 분은 60 미만이어야 합니다')
    return sum(v * 60**i for i, v in enumerate(reversed(values)))


def select_camera(ip, port=PORT):
    with socket.create_connection((ip, port), timeout=3) as connection:
        connection.settimeout(3)
        send_message(connection, dict(op='select', source='camera'))
        with connection.makefile('rb') as stream:
            response, _ = read_message(stream)
        if not response.get('ok') or response.get('source') != 'camera':
            raise RuntimeError(response.get('error', 'Camera source was not selected'))
        return response


def fit_frame(bgr):
    """Preserve aspect ratio inside a 16:9 canvas before JPEG transmission."""
    height, width = bgr.shape[:2]
    if not height or not width:
        raise ValueError('Empty video frame')
    scale = min(960 / width, 540 / height)
    w, h = max(1, round(width * scale)), max(1, round(height * scale))
    output = np.zeros((540, 960, 3), np.uint8)
    x, y = (960-w)//2, (540-h)//2
    output[y:y+h, x:x+w] = cv2.resize(bgr, (w,h))
    return output


class VideoFileSender:
    def __init__(self, ip, path, loop=True, fps=10, port=PORT, start_s=0., end_s=None):
        self.ip, self.path, self.port = ip, Path(path), port
        if not math.isfinite(fps) or not 0 < fps <= 10:
            raise ValueError('Transmission FPS must be in (0,10]')
        self.loop, self.fps = loop, fps
        self.start_s = parse_video_time(start_s)
        self.end_s = None if end_s is None else parse_video_time(end_s)
        if self.end_s is not None and self.end_s <= self.start_s:
            raise ValueError('종료 시간은 시작 시간보다 커야 합니다')
        self.stop = threading.Event()
        self.paused = threading.Event()
        self.lock = threading.Lock()
        self.status = dict(state='대기', sent=0, acknowledged=0, media_time_s=0., error='',
                           source_selected=False)
        self.connection = None
        self.thread = None

    def start(self):
        if self.thread is not None:
            raise RuntimeError('Sender already started')
        self.thread = threading.Thread(target=self._run, daemon=True, name='pc-video-sender')
        self.thread.start()

    def _set(self, **values):
        with self.lock:
            self.status.update(values)

    def snapshot(self):
        with self.lock:
            return dict(self.status)

    def toggle_pause(self):
        if self.paused.is_set():
            self.paused.clear()
        else:
            self.paused.set()

    def close(self, timeout=2):
        self.stop.set()
        if self.connection:
            try:
                self.connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        if self.thread:
            self.thread.join(timeout)

    def _run(self):
        capture = None
        try:
            self._set(state='영상 여는 중')
            capture = cv2.VideoCapture(str(self.path))
            ok, frame = capture.read()
            if not capture.isOpened() or not ok:
                raise ValueError('영상 파일을 읽을 수 없습니다')
            source_fps = capture.get(cv2.CAP_PROP_FPS)
            if not math.isfinite(source_fps) or source_fps <= 0:
                raise ValueError('영상 파일의 FPS를 확인할 수 없습니다')
            count = capture.get(cv2.CAP_PROP_FRAME_COUNT)
            total = int(count) if math.isfinite(count) and count > 0 else None
            if (self.start_s > 0 or self.end_s is not None) and total is None:
                raise ValueError('구간 전송에 필요한 영상 길이를 확인할 수 없습니다')
            if total is not None and (self.start_s >= total/source_fps or
                    (self.end_s is not None and self.end_s > total/source_fps + 1e-6)):
                raise ValueError(f'전송 구간이 영상 길이({total/source_fps:.3f}초)를 벗어납니다')
            first = math.ceil(self.start_s * source_fps - 1e-9)
            last = math.ceil(self.end_s * source_fps - 1e-9) if self.end_s is not None else total
            if last is not None and first >= last:
                raise ValueError('선택한 구간에 영상 프레임이 없습니다')

            def seek_start():
                if not capture.set(cv2.CAP_PROP_POS_FRAMES, first):
                    raise ValueError('선택한 시작 위치로 이동할 수 없습니다')
                valid, image = capture.read()
                actual = capture.get(cv2.CAP_PROP_POS_FRAMES) - 1
                if not valid or abs(actual-first) > .5:
                    raise ValueError('선택한 시작 프레임을 정확히 읽을 수 없습니다')
                return image

            if first:
                frame = seek_start()
            self._set(range_start_s=self.start_s, range_end_s=self.end_s,
                      media_time_s=first/source_fps, media_epoch=0)
            if self.stop.is_set():
                self._set(state='전송 중지')
                return
            self._set(state='장치 연결 중')
            with socket.create_connection((self.ip, self.port), timeout=3) as connection:
                self.connection = connection
                connection.settimeout(3)
                with connection.makefile('rb') as stream:
                    def exchange(header, payload=b''):
                        send_message(connection, header, payload)
                        response, _ = read_message(stream)
                        if not response.get('ok'):
                            raise RuntimeError(response.get('error', '장치 수신 오류'))
                        return response
                    if self.stop.is_set():
                        self._set(state='전송 중지')
                        return
                    exchange(dict(op='select', source='pc'))
                    self._set(source_selected=True)
                    started = time.monotonic()
                    deadline = started
                    index, media_epoch, sent = first, 0, 0
                    while not self.stop.is_set():
                        if self.paused.is_set():
                            pause_start = time.monotonic()
                            self._set(state='일시정지')
                            while self.paused.is_set() and not self.stop.wait(.2):
                                exchange(dict(op='status'))
                            paused_for = time.monotonic() - pause_start
                            started += paused_for; deadline += paused_for
                            continue
                        if self.stop.wait(max(0, deadline-time.monotonic())):
                            break
                        target = first + int((time.monotonic()-started)*source_fps)
                        eof = last is not None and target >= last
                        while not eof and index < target:
                            if self.stop.is_set():
                                break
                            ok, next_frame = capture.read()
                            if not ok:
                                eof = True; break
                            frame = next_frame; index += 1
                        if self.stop.is_set():
                            break
                        if eof:
                            if not self.loop:
                                break
                            frame = seek_start()
                            index = first; media_epoch += 1
                            started = deadline = time.monotonic()
                        ok, encoded = cv2.imencode('.jpg', fit_frame(frame),
                                                   [cv2.IMWRITE_JPEG_QUALITY, 85])
                        if not ok:
                            raise ValueError('JPEG 프레임 인코딩 실패')
                        response = exchange(dict(op='frame', frame_index=index,
                            media_time_s=index/source_fps, media_epoch=media_epoch,
                            file=self.path.name), encoded.tobytes())
                        sent += 1
                        self._set(state='전송 중', sent=sent,
                                  acknowledged=response['received_frames'],
                                  consumed=response['consumed_frames'],
                                  media_time_s=index/source_fps, media_epoch=media_epoch)
                        deadline = max(deadline+1/self.fps, time.monotonic())
                    if not self.stop.is_set():
                        exchange(dict(op='stop'))
            self._set(state='전송 중지' if self.stop.is_set() else '재생 완료')
        except (OSError, ValueError, RuntimeError, EOFError, cv2.error) as exc:
            if self.stop.is_set():
                self._set(state='전송 중지')
            else:
                self._set(state='오류', error=str(exc))
        finally:
            self.connection = None
            if capture is not None:
                capture.release()
