"""Check all video listeners after PC input starvation and client teardown."""
import argparse
import json
from pathlib import Path
import socket
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'tools'))
from oak_recorder import CaptureSource
from video_file_sender import select_camera, set_processing_mode
from video_transport import send_message, read_message

STREAMS = ((8553, 'bev'), (8554, 'yolopv2'), (8555, 'rgb'), (8556, 'depth'))


def ports(ip):
    result = {}
    for port, _ in STREAMS:
        try:
            with socket.create_connection((ip, port), timeout=1):
                result[str(port)] = True
        except OSError:
            result[str(port)] = False
    return result


def capture(ip):
    sources = {name:CaptureSource(name, name, f'rtsp://{ip}:{port}/{name}')
               for port, name in STREAMS}
    for source in sources.values():
        source.start()
    deadline = time.monotonic()+30
    while time.monotonic() < deadline:
        if all(s.frames_received >= 3 and s.info()['connected'] and s.info()['last_frame_age_s'] < 3
               for s in sources.values()):
            break
        time.sleep(.1)
    return sources


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device-ip', default='192.168.201.2')
    parser.add_argument('--cycles', type=int, default=2)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    report, sources = [], {}
    try:
        set_processing_mode(args.device_ip, 'full')
        for cycle in range(args.cycles):
            select_camera(args.device_ip)
            sources = capture(args.device_ip)
            baseline = {name:s.info() for name,s in sources.items()}
            assert all(s['frames_received'] >= 3 and s['connected']
                       and s['last_frame_age_s'] < 3 for s in baseline.values()), baseline
            with socket.create_connection((args.device_ip, 8558), timeout=3) as connection:
                connection.settimeout(3)
                send_message(connection, dict(op='select', source='pc'))
                with connection.makefile('rb') as stream:
                    reply, _ = read_message(stream)
                    assert reply.get('ok'), reply
            # No PC frames arrive. Disconnect all readers while appsrc is empty.
            time.sleep(8)
            for source in sources.values():
                source.close()
            sources = {}
            empty_probes = [ports(args.device_ip) for _ in range(3)]
            assert all(all(row.values()) for row in empty_probes), empty_probes
            select_camera(args.device_ip)
            sources = capture(args.device_ip)
            restored = {name:s.info() for name,s in sources.items()}
            assert all(s['frames_received'] >= 3 and s['connected'] and s['last_frame_age_s'] < 3
                       for s in restored.values()), restored
            report.append(dict(cycle=cycle+1, baseline=baseline, empty_probes=empty_probes,
                               restored=restored))
            print(json.dumps(report[-1]), flush=True)
            for source in sources.values():
                source.close()
            sources = {}
    finally:
        for source in sources.values():
            source.close()
        select_camera(args.device_ip)
        (args.output/'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    assert len(report) == args.cycles


if __name__ == '__main__':
    main()
