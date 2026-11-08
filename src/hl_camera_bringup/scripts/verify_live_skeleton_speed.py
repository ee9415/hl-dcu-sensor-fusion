"""Verify live camera response changes without a source/tracker reset."""
import json
from pathlib import Path
import socket
import sys
import time

ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'apps/yolopv2_standalone')]
from video_file_sender import set_skeleton_response_speed


def main():
    output=Path(sys.argv[1]);output.parent.mkdir(parents=True,exist_ok=True)
    results=[]
    with socket.create_connection(('192.168.201.2',8557),timeout=10) as connection:
        connection.settimeout(10);connection.sendall(b'HELLO\n')
        with connection.makefile('rb') as stream:
            try:
                for speed in (.5,2,3,1):
                    ack=set_skeleton_response_speed('192.168.201.2',speed)
                    frames=[];deadline=time.monotonic()+15
                    while len(frames)<5 and time.monotonic()<deadline:
                        frame=json.loads(stream.readline())
                        if frame.get('skeleton_response_speed')!=speed:continue
                        response=frame.get('boundary_response') or {}
                        if response.get('retained_max_rate_px_s')!=40*speed:continue
                        assert response['observed_max_rate_px_s']==120*speed
                        assert abs(response['retained_tau_s']-.2/speed)<1e-9
                        frames.append(frame)
                    assert len(frames)==5, (speed,'No matching live frames')
                    results.append(dict(speed=speed,ack=ack,frames=frames))
            finally:set_skeleton_response_speed('192.168.201.2',1)
    epochs={(item['ack']['source_epoch'],item['ack']['processing_epoch']) for item in results}
    assert len(epochs)==1,'Speed changed source or processing epoch'
    output.write_text(json.dumps(dict(ok=True,results=results),indent=2),encoding='utf-8')
    print(json.dumps(dict(ok=True,speeds=[item['speed'] for item in results],frames=20)))


if __name__=='__main__':main()
