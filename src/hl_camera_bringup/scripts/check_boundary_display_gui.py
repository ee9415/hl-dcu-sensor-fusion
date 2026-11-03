"""Real Tk checkbox/control round-trip against a local server, no camera changes."""
import json
from pathlib import Path
import sys
import time
import tkinter as tk
from types import SimpleNamespace
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools'))
import oak_recorder as recorder
from video_transport import VideoInputServer
from video_file_sender import set_boundary_display,set_lane_overlay


def main():
    root=tk.Tk();root.withdraw()
    server=VideoInputServer(0,'127.0.0.1')
    app=recorder.RecorderApp(root,'127.0.0.1',recorder.DEFAULT_DEVICE_SERIAL,
                            None,None,None,ROOT/'artifacts/boundary_display_20261028/gui')
    result={}
    try:
        assert app.boundary_visible.get()
        assert app.boundary_display_check.cget('text')=='차선·가상 경계선 표시'
        app.connect_requested=True
        app.boundary_display_check.configure(state=tk.NORMAL)
        def local(ip,visible):
            return set_boundary_display(ip,visible,server.server.server_address[1])
        with patch.object(recorder,'set_boundary_display',side_effect=local):
            for visible in (False,True):
                app.boundary_visible.set(visible)
                app._apply_boundary_display()
                deadline=time.monotonic()+3
                while app.boundary_display_pending and time.monotonic()<deadline:
                    root.update();app._update_input_status();time.sleep(.01)
                assert not app.boundary_display_pending
                assert server.boundary_visible is visible
                assert app.confirmed_boundary_visible is visible
                assert server.processing_epoch==server.epoch==0
        with patch.object(recorder,'set_boundary_display',side_effect=OSError('test connection failure')):
            app.boundary_visible.set(False);app._apply_boundary_display()
            deadline=time.monotonic()+3
            while app.boundary_display_pending and time.monotonic()<deadline:
                root.update();app._update_input_status();time.sleep(.01)
            assert app.boundary_visible.get() is True
            assert not app.boundary_display_pending
        def local_preview(ip,mode,threshold):
            return set_lane_overlay(ip,mode,threshold,server.server.server_address[1])
        with patch.object(recorder,'set_lane_overlay',side_effect=local_preview):
            for label,mode,threshold in (('차선 점수 RGB','lane_score',.3),
                                          ('임계값 마스크 RGB','lane_mask',.8),
                                          ('원본 RGB','raw_rgb',.5),
                                          ('점수 구간색 (검정 배경)','lane_bands',.05),
                                          ('원본 + 구간색 겹침','lane_bands_rgb',.05),
                                          ('원본·구간색·겹침 3분할','lane_compare',.05),
                                          ('일반 인식 결과','normal',.5)):
                app.lane_preview_view.set(label);app.lane_preview_threshold.set(threshold)
                app._apply_lane_preview()
                deadline=time.monotonic()+3
                while app.lane_preview_pending and time.monotonic()<deadline:
                    root.update();app._update_input_status();time.sleep(.01)
                assert not app.lane_preview_pending
                assert (server.lane_overlay_mode,server.lane_overlay_threshold)==(mode,threshold)
                assert app.confirmed_lane_preview==(mode,threshold)
                assert server.processing_epoch==server.epoch==0
            # Starting the reference family gets .05, but switching between
            # its panels preserves a threshold the user has already adjusted.
            for label,requested,expected,choice_event in (
                ('점수 구간색 (검정 배경)',.9,.05,True),
                ('점수 구간색 (검정 배경)',.7,.7,False),
                ('원본 + 구간색 겹침',.7,.7,True),
                ('원본·구간색·겹침 3분할',.7,.7,True),
                ('일반 인식 결과',.5,.5,False)):
                app.lane_preview_view.set(label);app.lane_preview_threshold.set(requested)
                app._apply_lane_preview(SimpleNamespace(widget=app.lane_preview_choice) if choice_event else None)
                deadline=time.monotonic()+3
                while app.lane_preview_pending and time.monotonic()<deadline:
                    root.update();app._update_input_status();time.sleep(.01)
                assert not app.lane_preview_pending
                assert app.lane_preview_threshold.get()==expected
                assert server.lane_overlay_threshold==expected
        with patch.object(recorder,'set_lane_overlay',side_effect=OSError('test preview failure')):
            app.lane_preview_view.set('임계값 마스크 RGB');app.lane_preview_threshold.set(.9)
            app._apply_lane_preview()
            deadline=time.monotonic()+3
            while app.lane_preview_pending and time.monotonic()<deadline:
                root.update();app._update_input_status();time.sleep(.01)
            assert app.lane_preview_view.get()=='일반 인식 결과'
            assert app.lane_preview_threshold.get()==.5
        app.lane_preview_threshold.set(2)
        app._apply_lane_preview()
        assert not app.lane_preview_pending
        app.lane_preview_threshold.set(.5)
        result=dict(passed=True,checkbox=app.boundary_display_check.cget('text'),
                    on_off_roundtrip=True,error_reverts=True,input_epoch=server.epoch,
                    processing_epoch=server.processing_epoch,lane_preview_roundtrip=True,
                    preview_error_reverts=True,preview_range_validation=True,
                    reference_default_and_preserved_threshold=True)
        out=ROOT/'artifacts/boundary_display_20261028/gui_report.json'
        out.write_text(json.dumps(result,indent=2),encoding='utf8')
        print(json.dumps(result,ensure_ascii=False))
    finally:
        app._close();server.close()


if __name__=='__main__':main()
