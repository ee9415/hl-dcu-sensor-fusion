"""Read-only DepthAI2 discovery helper, including optional Windows SDK path."""
# OAK-D(DepthAI 2) 장치 검색 보조 프로그램. 별도 프로세스로 실행되어 결과를
# 'OAKDISCOVERY <JSON>' 한 줄로 출력한다. 장치에 연결하거나 파이프라인을 시작하지 않는다.
import argparse
import json
import os
import sys


def main():
    """USB/PoE로 보이는 OAK-D 장치와 부트로더 상태 장치를 모두 찾아 MX ID 기준으로 합친다.
    --sdk-path를 주면 그 경로의 DepthAI(Windows SDK 등)를 사용한다.
    """
    parser=argparse.ArgumentParser()
    parser.add_argument('--sdk-path')
    args=parser.parse_args()
    dll=None
    if args.sdk_path:
        sys.path.insert(0,args.sdk_path)
        if os.name=='nt':
            # Windows에서는 SDK 폴더의 DLL을 찾을 수 있게 등록한다.
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
