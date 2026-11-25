"""Windows-side Kvaser Leaf v3 service. No bus-on unless --enable-can is explicit."""
import argparse
import json
from pathlib import Path
import secrets
import socket
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/hl_camera_can'))
from hl_camera_can.kvaser import Canlib, KvaserDriver, DEFAULT_DLL
from hl_camera_can.windows_transport import serve_connection


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--list', action='store_true', help='Read-only channel enumeration')
    parser.add_argument('--enable-can', action='store_true')
    parser.add_argument('--virtual', action='store_true', help='Require a virtual Kvaser channel')
    parser.add_argument('--channel', type=int, default=0)
    parser.add_argument('--dll', default=DEFAULT_DLL)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=53200)
    parser.add_argument('--key-file', type=Path, default=ROOT/'artifacts/camera_can/windows.key')
    args = parser.parse_args()
    if args.list:
        print(json.dumps(Canlib(args.dll).channels(), ensure_ascii=False, indent=2))
        return
    args.key_file.parent.mkdir(parents=True, exist_ok=True)
    if not args.key_file.exists():
        with args.key_file.open('x', encoding='ascii') as stream:
            stream.write(secrets.token_hex(32))
    key = args.key_file.read_text(encoding='ascii').strip()
    if len(key) < 64:
        raise ValueError('Invalid Windows CAN key file')
    driver = KvaserDriver(args.channel, args.virtual, args.dll) if args.enable_can else None
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.bind((args.host, args.port))
            server.listen(1)
            print(json.dumps(dict(host=args.host, port=args.port, enabled=driver is not None,
                virtual=args.virtual, channel=args.channel)), flush=True)
            while True:
                connection, _ = server.accept()
                with connection:
                    try:
                        serve_connection(connection, key, driver, args.virtual)
                    except (OSError, ValueError, KeyError, TypeError) as exc:
                        print('CAN connection closed: '+str(exc), flush=True)
                        if driver is not None and driver.handle is None:
                            raise RuntimeError('CAN driver stopped after write failure; explicit restart required')
    except KeyboardInterrupt:
        pass
    finally:
        if driver is not None:
            driver.close()


if __name__ == '__main__':
    main()
