"""CAM01 recording service: camera -> hardware H.264 -> RTSP. No AI imports."""
import json
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


def build_pipeline(dai, device, fps=30):
    pipeline = dai.Pipeline(device)
    camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    output = camera.requestOutput((1920, 1080), dai.ImgFrame.Type.NV12,
                                  fps=fps, enableUndistortion=True)
    encoder = pipeline.create(dai.node.VideoEncoder)
    encoder.setDefaultProfilePreset(fps, dai.VideoEncoderProperties.Profile.H264_MAIN)
    output.link(encoder.input)
    queue = encoder.out.createOutputQueue(maxSize=4, blocking=True)
    return pipeline, queue


def status_server(state, lock, port=8561):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path != '/status':
                self.send_error(404)
                return
            with lock:
                payload = dict(state)
            age = time.monotonic()-payload.pop('last_frame_at')
            payload['ok'] = payload['frames'] > 0 and age < 3
            payload['frame_age_s'] = age
            data = json.dumps(payload).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(('0.0.0.0', port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def main():
    import depthai as dai
    from rtsp_server import RTSPServer
    device = dai.Device()
    info = device.getDeviceInfo()
    state = dict(mode='rgb_only', source='camera', source_epoch=uuid.uuid4().hex,
                 mxid=str(info.getDeviceId()), fps=30, frames=0, last_frame_at=0.,
                 ai_enabled=False)
    lock = threading.Lock()
    pipeline, queue = build_pipeline(dai, device, state['fps'])
    stream = RTSPServer(state['fps'], 8560, '/rgb')
    server = status_server(state, lock)
    try:
        with pipeline:
            pipeline.start()
            while pipeline.isRunning():
                packet = queue.tryGet()
                if packet is None:
                    time.sleep(.002)
                    continue
                stream.send_data(bytes(packet.getData()))
                with lock:
                    state['frames'] += 1
                    state['last_frame_at'] = time.monotonic()
    finally:
        server.shutdown()
        server.server_close()


if __name__ == '__main__':
    main()
