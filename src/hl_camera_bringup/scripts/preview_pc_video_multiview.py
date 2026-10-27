"""Open the real source-selection UI for an end-to-end device preview check."""
import argparse
import json
from pathlib import Path
import sys
import time
import tkinter as tk
import cv2

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools'))
from oak_recorder import RecorderApp,DEFAULT_DEVICE_SERIAL
from video_file_sender import select_camera


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device-ip',required=True)
    p.add_argument('--video',type=Path,required=True)
    p.add_argument('--seconds',type=int,default=90)
    args=p.parse_args()
    output=ROOT/'artifacts'/'pc_video_input_20261027';output.mkdir(parents=True,exist_ok=True)
    select_camera(args.device_ip)
    root=tk.Tk()
    app=RecorderApp(root,args.device_ip,DEFAULT_DEVICE_SERIAL,None,None,None,output/'preview_recordings')
    root.title('CAM01 멀티뷰 · PC 영상 입력 검증')
    app.input_mode.set('PC 영상 파일');app.input_file.set(str(args.video))
    started=time.monotonic();requested=False;reported=False
    def check():
        nonlocal requested,reported
        if not requested and str(app.source_apply.cget('state')) == 'normal':
            requested=True;app._apply_input_source()
        if app.file_sender and app.file_sender.snapshot()['sent']>=150 and not reported:
            report=dict(sender=app.file_sender.snapshot(),status=app.source_status.get(),
                streams={k:dict(connected=s.snapshot()[0],frames=s.frames_received)
                         for k,s in app.sources.items()},csv_started=app.csv_logger is not None)
            for key, source in app.sources.items():
                connected, frame = source.snapshot()
                if connected and frame is not None:
                    cv2.imwrite(str(output/f'pc_{key}.jpg'), frame)
            (output/'device_gui_check.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
            print(json.dumps(report,ensure_ascii=False),flush=True);reported=True
        if time.monotonic()-started>=args.seconds:
            print(json.dumps(dict(final_streams={k:dict(connected=s.snapshot()[0],frames=s.frames_received)
                                for k,s in app.sources.items()},status=app.status_text.get(),
                                sender=app.file_sender.snapshot() if app.file_sender else None),ensure_ascii=False),flush=True)
            app._close()
            select_camera(args.device_ip)
        else:root.after(100,check)
    root.after(200,app.connect_video);root.after(400,check)
    try:root.mainloop()
    finally:
        if app.file_sender:app.file_sender.close()
        select_camera(args.device_ip)


if __name__=='__main__':main()
