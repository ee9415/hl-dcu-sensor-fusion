"""Reproduce a disconnected PC source, then recover four preview streams."""
import json
from pathlib import Path
import socket
import sys
import time

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools'))
from oak_recorder import CaptureSource
from video_file_sender import prepare_preview
from video_transport import send_message,read_message


def main():
    output=Path(sys.argv[1]);output.parent.mkdir(parents=True,exist_ok=True)
    sources=[]
    try:
        with socket.create_connection(('192.168.201.2',8558),timeout=3) as publisher:
            send_message(publisher,dict(op='select',source='pc'))
            with publisher.makefile('rb') as stream:read_message(stream)
            active=prepare_preview('192.168.201.2')
            assert active['source']=='pc' and active['publisher_connected']
            assert not active['preview_recovered']
        deadline=time.monotonic()+5
        while True:
            recovered=prepare_preview('192.168.201.2')
            if recovered['preview_recovered']:break
            assert time.monotonic()<deadline,'Publisher was not released'
            time.sleep(.1)
        assert recovered['source']=='camera'
        for port,name in ((8555,'rgb'),(8554,'yolopv2'),(8553,'bev'),(8556,'depth')):
            source=CaptureSource(name,name,f'rtsp://192.168.201.2:{port}/{name}')
            sources.append(source);source.start()
        deadline=time.monotonic()+35
        while time.monotonic()<deadline:
            final=[s.info() for s in sources]
            if all(s['connected'] and s['frames_received']>=8 and s['last_frame_age_s']<3 for s in final):break
            time.sleep(.5)
        else:raise AssertionError(final)
        output.write_text(json.dumps(dict(ok=True,active_publisher=active,recovered=recovered,streams=final),indent=2))
        print(json.dumps(dict(ok=True,streams=final)))
    finally:
        for source in sources:source.stop_event.set()
        for source in sources:source.close()


if __name__=='__main__':main()
