"""Independent guidance drawing control, offline preference and protocol tests."""
from pathlib import Path
import sys,tempfile,unittest,json
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'tools'))
from video_transport import VideoInputServer
from video_file_sender import set_guidance_display
from oak_recorder import RecorderApp
import settings_store

class GuidanceDisplayTests(unittest.TestCase):
    def test_toggle_does_not_reset_perception_or_old_frame_metadata(self):
        hub=VideoInputServer(0,'127.0.0.1')
        try:
            port=hub.server.server_address[1];_,before=hub.take();hub.remember(1,before)
            off=set_guidance_display('127.0.0.1',False,port)
            _,after=hub.take()
            for key in ('source_epoch','processing_epoch','boundary_display_epoch','top_view_epoch'):
                self.assertEqual(before[key],after[key])
            self.assertFalse(after['guidance_visible']);self.assertTrue(after['boundary_visible'])
            self.assertTrue(hub.metadata(1)['guidance_visible'])
            self.assertEqual(set_guidance_display('127.0.0.1',False,port)['guidance_display_epoch'],off['guidance_display_epoch'])
            self.assertTrue(set_guidance_display('127.0.0.1',True,port)['guidance_visible'])
            for bad in (None,1,'true'):
                with self.assertRaises(ValueError):hub.handle(None,dict(op='guidance_display',visible=bad),b'')
            with self.assertRaises(ValueError):hub.handle(None,dict(op='guidance_display',visible=True),b'x')
        finally:hub.close()

    def test_offline_checkbox_saves_without_network(self):
        class Var:
            def __init__(self,value):self.value=value
            def get(self):return self.value
            def set(self,value):self.value=value
        app=RecorderApp.__new__(RecorderApp)
        app.guidance_display_pending=False;app.confirmed_guidance_visible=True
        app.guidance_visible=Var(False);app.guidance_display_status=Var('');app.connect_requested=False
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'settings.json'
            def save():
                settings_store.update(path,{'guidance_test':app.confirmed_guidance_visible});return True
            app._save_preferences=save
            with patch('oak_recorder.set_guidance_display') as remote:
                app._apply_guidance_display();remote.assert_not_called()
            self.assertFalse(settings_store.read(path)['guidance_test'])

    def test_legacy_preferences_and_bool_validation(self):
        data=dict(display=dict(full_processing=True,boundary_visible=True,lane_mode='normal',threshold=.5),
            selected=dict(rgb=True,yolopv2=True),recording=dict(output_dir='out',session_name='test',
                resolution='원본 스트림',fps='10',verify_output=True))
        settings_store.preferences(data)
        data['display']['guidance_visible']=False
        self.assertFalse(settings_store.preferences(data)['display']['guidance_visible'])
        data['display']['guidance_visible']=1
        with self.assertRaises(ValueError):settings_store.preferences(data)
