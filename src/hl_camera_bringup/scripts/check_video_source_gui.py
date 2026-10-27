"""Exercise the real multiview source controls against a loopback receiver."""
import json
from pathlib import Path
import sys
import tempfile
import time
import tkinter as tk

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools'))
from oak_recorder import RecorderApp,DEFAULT_DEVICE_SERIAL
from video_transport import VideoInputServer


def main():
    output=ROOT/'artifacts'/'pc_video_input_20261027';output.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(dir=output) as folder:
        path=Path(folder)/'gui_input.avi'
        writer=cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*'MJPG'),10,(320,180))
        assert writer.isOpened()
        for i in range(20):writer.write(np.full((180,320,3),30+i,np.uint8))
        writer.release()
        hub=VideoInputServer(host='127.0.0.1')
        root=tk.Tk()
        app=RecorderApp(root,'127.0.0.1',DEFAULT_DEVICE_SERIAL,None,None,None,output/'recordings')
        def pump(condition,seconds=5):
            end=time.monotonic()+seconds
            while not condition() and time.monotonic()<end:
                root.update();time.sleep(.02)
            assert condition(),app.source_status.get()
        try:
            root.update();app.connect_requested=True
            app.source_apply.configure(state='normal')
            app.input_mode.set('PC 영상 파일');app.input_file.set(str(path))
            app._apply_input_source()
            pump(lambda:app.active_input=='pc' and hub.received>=3)
            pc_status=app.source_status.get()
            app._pause_input();pump(lambda:app.file_sender.snapshot()['state']=='일시정지')
            app._pause_input();pump(lambda:app.file_sender.snapshot()['state']=='전송 중')
            app.input_mode.set('카메라');app._apply_input_source()
            pump(lambda:hub.mode=='camera' and app.active_input=='camera')
            widgets=[app.source_apply,app.source_pause]
            report=dict(pc_status=pc_status,camera_status=app.source_status.get(),
                widgets_mapped=all(w.winfo_ismapped() for w in widgets),
                controls_inside_window=all(w.winfo_rootx()+w.winfo_width()<=root.winfo_rootx()+root.winfo_width()
                                           for w in widgets),
                csv_created=app.csv_logger is not None,passed=True)
            assert report['widgets_mapped'] and report['controls_inside_window']
            assert not report['csv_created']
            (output/'gui_check.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
            print(json.dumps(report,ensure_ascii=False),flush=True)
        finally:
            app._close();hub.close()


if __name__=='__main__':main()
