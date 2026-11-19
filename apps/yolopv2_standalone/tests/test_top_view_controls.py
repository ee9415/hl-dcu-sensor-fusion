"""Display-only perspective math, protocol snapshots and disconnected controls."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import sys,tempfile,unittest
from unittest.mock import Mock,patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'tools'))
from lane_boundaries import BoundaryConfig,projection_for
from top_view_controls import top_view_config,validate_top_view
from video_transport import VideoInputServer
from video_file_sender import set_top_view
from oak_recorder import RecorderApp
import settings_store

class TopControlTests(unittest.TestCase):
    def setUp(self):
        self.base=replace(BoundaryConfig(),near_left_u=.075724,near_right_u=.798382,near_v=1.,
            far_left_u=.382381,far_right_u=.567136,far_v=.62)

    def test_defaults_preserve_old_projection_and_near_anchors(self):
        self.assertEqual(top_view_config(self.base),self.base)
        for spread in (-30,0,100):
            for ahead in (0,50,100):
                c=top_view_config(self.base,spread,ahead)
                self.assertEqual((c.near_left_u,c.near_right_u,c.near_v),
                    (self.base.near_left_u,self.base.near_right_u,self.base.near_v))
                self.assertTrue(np.isfinite(projection_for((180,320),c)).all())
                self.assertLess(c.far_left_u,c.far_right_u)
        self.assertAlmostEqual(top_view_config(self.base,0,100).far_v,.56)

    def test_positive_correction_narrows_far_output_and_keeps_near_width(self):
        def mapped(c,points):
            v=np.linalg.inv(projection_for((180,320),c))@np.column_stack((points,np.ones(len(points)))).T
            return (v[:2]/v[2]).T
        points=np.array([[self.base.far_left_u*319,.62*179],[self.base.far_right_u*319,.62*179],
            [self.base.near_left_u*319,179],[self.base.near_right_u*319,179]])
        old=mapped(self.base,points);new=mapped(top_view_config(self.base,50,0),points)
        self.assertLess(np.ptp(new[:2,0]),np.ptp(old[:2,0]))
        np.testing.assert_allclose(new[2:],old[2:],atol=1e-9)

    def test_invalid_values_are_rejected(self):
        for value in (True,None,'0',float('nan'),float('inf'),-31,101):
            with self.assertRaises(ValueError):validate_top_view(value,0)
        with self.assertRaises(ValueError):validate_top_view(0,-1)

    def test_protocol_preserves_publisher_epochs_and_historical_metadata(self):
        hub=VideoInputServer(0,'127.0.0.1')
        try:
            _,old=hub.take();hub.remember(1,old)
            result=set_top_view('127.0.0.1',40,70,hub.server.server_address[1])
            _,new=hub.take()
            self.assertEqual((new['top_view_spread'],new['top_view_ahead']),(40,70))
            for key in ('source_epoch','processing_epoch','skeleton_response_epoch'):
                self.assertEqual(new[key],old[key])
            self.assertEqual(hub.metadata(1)['top_view_spread'],0)
            again=set_top_view('127.0.0.1',40,70,hub.server.server_address[1])
            self.assertEqual(again['top_view_epoch'],result['top_view_epoch'])
            with self.assertRaises(ValueError):hub.handle(None,dict(op='top_view',spread=0,ahead=0),b'x')
            self.assertEqual(hub._status()['top_view_spread'],40)
        finally:hub.close()

    def test_offline_apply_saves_without_network_and_reset_restores_baseline(self):
        class Var:
            def __init__(self,value):self.value=value
            def get(self):return self.value
            def set(self,value):self.value=value
        app=RecorderApp.__new__(RecorderApp)
        app.top_view_spread=Var(40);app.top_view_ahead=Var(70);app.top_view_status=Var('')
        app.top_view_pending=False;app.confirmed_top_view=(0,0);app.connect_requested=False
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'settings.json'
            def save():
                settings_store.update(path,{'top_test':app.confirmed_top_view});return True
            app._save_preferences=save
            with patch('oak_recorder.set_top_view') as remote:
                app._apply_top_view();remote.assert_not_called()
                self.assertEqual(settings_store.read(path)['top_test'],[40,70])
                self.assertIn('미적용',app.top_view_status.get())
                app._reset_top_view()
                self.assertEqual(settings_store.read(path)['top_test'],[0,0])

    def test_preferences_keep_legacy_defaults_and_validate_new_values(self):
        data=dict(display=dict(full_processing=True,boundary_visible=True,lane_mode='normal',threshold=.5),
            selected=dict(rgb=True,yolopv2=True),recording=dict(output_dir='out',session_name='test',
                resolution='원본 스트림',fps='10',verify_output=True))
        settings_store.preferences(data)
        data['display'].update(top_view_spread=40,top_view_ahead=70)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'settings.json';settings_store.update(path,{'preferences':settings_store.preferences(data)})
            self.assertEqual(settings_store.preferences(settings_store.read(path)['preferences'])['display']['top_view_ahead'],70)
        data['display']['top_view_ahead']=101
        with self.assertRaises(ValueError):settings_store.preferences(data)
