"""Isolated DepthAI 2 worker. Stdout carries framed MJPEG; stderr carries errors."""
# OAK-D RGB 수신용 독립 워커 프로세스(DepthAI 2 전용).
# stdout으로 'OAKRGB <헤더 JSON>\n<JPEG 바이트>' 형식 패킷을 보내고, 오류는 stderr로 보낸다.
import argparse
import json
import os
import sys
import time

PACKETS = None


def emit(data, sequence, device_time, mxid=''):
    """JPEG 한 장을 헤더(길이, 시퀀스, 장치 시각, MX ID)와 함께 stdout 패킷으로 보낸다."""
    header = dict(length=len(data), sequence=sequence, device_time_s=device_time, mxid=mxid)
    PACKETS.write(b'OAKRGB '+json.dumps(header).encode()+b'\n'+data)
    PACKETS.flush()


def build_pipeline(dai, width, height, fps):
    """컬러 카메라(1080p→ISP 2/3 축소) → MJPEG 인코더 → XLink 출력 파이프라인을 만든다."""
    pipeline = dai.Pipeline()
    camera = pipeline.create(dai.node.ColorCamera)
    camera.setBoardSocket(dai.CameraBoardSocket.RGB)
    camera.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camera.setIspScale(2, 3)
    camera.setVideoSize(width, height)
    camera.setFps(fps)
    encoder = pipeline.create(dai.node.VideoEncoder)
    encoder.setDefaultProfilePreset(fps, dai.VideoEncoderProperties.Profile.MJPEG)
    encoder.setQuality(85)
    camera.video.link(encoder.input)
    output = pipeline.create(dai.node.XLinkOut)
    output.setStreamName('rgb_jpeg')
    encoder.bitstream.link(output.input)
    return pipeline


def main():
    """가상 모드면 카메라별 색 패턴 영상을 만들어 보내고, 실제 모드면 지정한 OAK-D에 접속해 JPEG을 계속 보낸다.
    5초간 프레임이 없거나 MX ID가 다르면 오류로 종료한다.
    """
    global PACKETS
    # Native SDK logging must never interleave with binary frame packets.
    # 원래 stdout은 패킷 전용으로 복제해 두고, 이후 일반 출력은 stderr로 돌린다.
    PACKETS = os.fdopen(os.dup(sys.stdout.fileno()), 'wb', buffering=0)
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    parser = argparse.ArgumentParser()
    parser.add_argument('--target', default='')
    parser.add_argument('--mxid', default='')
    parser.add_argument('--name', required=True)
    parser.add_argument('--virtual', action='store_true')
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--fps', type=int, default=10)
    parser.add_argument('--fail-after', type=int, default=0, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.virtual:
        import cv2
        import numpy as np
        index = int(args.name[-2:])
        sequence = 0
        started = time.monotonic()
        while True:
            # 카메라 번호마다 다른 배경색 + 움직이는 세로선으로 가상 영상을 만든다.
            image = np.full((args.height, args.width, 3), (index*31 % 160, 40, 80), np.uint8)
            x = (sequence*9) % args.width
            cv2.line(image, (x, 0), (x, args.height-1), (255, 255, 255), 3)
            cv2.putText(image, args.name+' VIRTUAL', (8, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        .65, (255, 255, 255), 2)
            ok, jpeg = cv2.imencode('.jpg', image)
            if not ok:
                raise RuntimeError('Virtual JPEG encoding failed')
            emit(jpeg.tobytes(), sequence, time.monotonic()-started)
            sequence += 1
            if args.fail_after and sequence >= args.fail_after:
                raise RuntimeError('Injected virtual disconnect')
            time.sleep(max(0, started+sequence/args.fps-time.monotonic()))
    else:
        try:
            import depthai as dai
        except ImportError as exc:
            raise RuntimeError('DepthAI 2 Python 환경이 필요합니다. 카메라 설정의 Python 경로를 확인하세요.') from exc
        if not str(dai.__version__).startswith('2.'):
            raise RuntimeError('OAK-D worker는 DepthAI 2 환경이 필요합니다. 기존 OAK4 환경을 변경하지 마세요.')
        if not args.target:
            raise ValueError('장치 IP 또는 MX ID가 필요합니다.')
        pipeline = build_pipeline(dai, args.width, args.height, args.fps)
        with dai.Device(pipeline, dai.DeviceInfo(args.target)) as device:
            actual_id = device.getMxId()
            if args.mxid and actual_id != args.mxid:
                raise RuntimeError('연결된 MX ID가 설정과 다릅니다: '+actual_id)
            queue = device.getOutputQueue(name='rgb_jpeg', maxSize=2, blocking=False)
            last = time.monotonic()
            while True:
                packet = queue.tryGet()
                if packet is None:
                    if time.monotonic()-last > 5:
                        raise RuntimeError('OAK-D RGB 수신 시간 초과')
                    time.sleep(.005)
                    continue
                last = time.monotonic()
                emit(packet.getData().tobytes(), int(packet.getSequenceNum()),
                     packet.getTimestamp().total_seconds(),actual_id)


if __name__ == '__main__':
    try:
        main()
    except (Exception, KeyboardInterrupt) as exc:
        print(str(exc), file=sys.stderr, flush=True)
        sys.exit(1)
