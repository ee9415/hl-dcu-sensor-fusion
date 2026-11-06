"""Reconnect readers before restarting a stopped PC publisher, then soak."""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'tools'))
from oak_recorder import CaptureSource
from video_file_sender import VideoFileSender, select_camera, set_processing_mode


def readers(ip):
    result = [CaptureSource(n,n,f'rtsp://{ip}:{p}/{n}')
              for p,n in ((8554,'yolopv2'),(8553,'bev'))]
    for source in result:
        source.start()
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device-ip', default='192.168.201.2')
    p.add_argument('--video', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    sources, sender, samples = [], None, []
    try:
        select_camera(args.device_ip)
        set_processing_mode(args.device_ip,'full')
        sources = readers(args.device_ip)
        sender = VideoFileSender(args.device_ip,args.video,loop=False,fps=10,start_s=120,end_s=125)
        sender.start()
        sender.thread.join(20)
        assert not sender.thread.is_alive(), 'Initial publisher did not finish'
        assert not sender.snapshot().get('error'), sender.snapshot()
        sender.close()
        time.sleep(12)
        for source in sources:
            source.close()
        sources = []
        time.sleep(10)
        # Match the UI sequence: readers connect to an idle PC source first.
        sources = readers(args.device_ip)
        time.sleep(10)
        sender = VideoFileSender(args.device_ip,args.video,loop=False,fps=10,start_s=120,end_s=170)
        sender.start()
        deadline = time.monotonic()+65
        while time.monotonic()<deadline and sender.thread.is_alive():
            samples.append(dict(time_s=time.monotonic(), streams=[s.info() for s in sources]))
            time.sleep(1)
        assert not sender.thread.is_alive(), 'Publisher did not finish'
        final = [s.info() for s in sources]
        assert not sender.snapshot().get('error'), sender.snapshot()
        assert len(samples)>=25, 'Insufficient continuous replay time'
        for index in range(2):
            early, late = samples[-21]['streams'][index], final[index]
            assert late['frames_received']-early['frames_received']>=20, (early,late)
            assert late['connected'] and late['last_frame_age_s']<3, late
        report=dict(ok=True, sender=sender.snapshot(), final=final, samples=samples)
        (args.output/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(json.dumps(dict(ok=True,final=final)),flush=True)
    finally:
        (args.output/'samples.json').write_text(json.dumps(samples,indent=2),encoding='utf-8')
        if sender:
            sender.close()
        for source in sources:
            source.close()
        select_camera(args.device_ip)


if __name__=='__main__':
    main()
