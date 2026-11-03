"""Read-only DepthAI2 discovery helper, including optional Windows SDK path."""
import argparse
import json
import os
import sys


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--sdk-path')
    args=parser.parse_args()
    dll=None
    if args.sdk_path:
        sys.path.insert(0,args.sdk_path)
        if os.name=='nt':
            dll=os.add_dll_directory(args.sdk_path)
    import depthai as dai
    if not dai.__version__.startswith('2.'):
        raise RuntimeError('DepthAI 2 Python 환경을 지정해 주세요.')
    result={}
    for devices in (dai.Device.getAllAvailableDevices(),dai.DeviceBootloader.getAllAvailableDevices()):
        for device in devices:
            result[device.getMxId()]=dict(family='oakd',serial=device.getMxId(),
                ip=device.name,model='OAK-D / DepthAI2',state=str(device.state).split('.')[-1])
    print('OAKDISCOVERY '+json.dumps(list(result.values())),flush=True)


if __name__=='__main__':
    try:main()
    except Exception as exc:
        print(str(exc),file=sys.stderr);sys.exit(1)
