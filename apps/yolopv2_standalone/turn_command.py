"""Authenticated oakctl exec entry point; acknowledges pipeline acceptance."""
import argparse
import json
import os
import time
import uuid
from turn_preview import COMMAND_DIR


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=('left', 'right', 'road'))
    args = parser.parse_args()
    COMMAND_DIR.mkdir(exist_ok=True, parents=True)
    path = COMMAND_DIR / (uuid.uuid4().hex+'.request')
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps({'mode': args.mode, 'created': time.time()}))
    os.replace(temp, path)
    response = path.with_suffix('.response')
    deadline = time.monotonic()+5
    while time.monotonic() < deadline:
        if response.exists():
            result = json.loads(response.read_text())
            print('TURN_ACK '+json.dumps(result), flush=True)
            path.unlink(missing_ok=True)
            response.unlink(missing_ok=True)
            return 0 if result['accepted'] else 2
        time.sleep(.05)
    path.unlink(missing_ok=True)
    print('TURN_ACK '+json.dumps({'accepted': False, 'error': 'pipeline acknowledgement timeout'}))
    return 3


if __name__ == '__main__':
    raise SystemExit(main())
