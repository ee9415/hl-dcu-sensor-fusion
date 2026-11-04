import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'tools'))
import settings_store as store
import camera_settings
import video_input_settings


class UnifiedSettingsTests(unittest.TestCase):
    def test_optional_contour_preference_legacy_and_roundtrip(self):
        data=dict(display=dict(full_processing=True,boundary_visible=True,lane_mode='normal',threshold=.5),
                  selected=dict(rgb=True,yolopv2=True),recording=dict(output_dir='out',session_name='test',
                    resolution='원본 스트림',fps='10',verify_output=True))
        self.assertNotIn('contour_visible',store.preferences(data)['display'])
        data['display']['contour_visible']=True
        store.update(self.path,{'preferences':store.preferences(data)})
        self.assertTrue(store.preferences(store.read(self.path)['preferences'])['display']['contour_visible'])
        data['display']['contour_visible']='true'
        with self.assertRaises(ValueError):store.preferences(data)

    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.path=Path(self.temp.name)/'settings.json'

    def test_camera_reset_preserves_other_sections(self):
        store.update(self.path,{'credentials':{'ip':'cipher'},'preferences':{'marker':1}})
        data=camera_settings.defaults();data['reset']=True
        camera_settings.save(self.path,data)
        self.assertEqual(store.read(self.path)['credentials'],{'ip':'cipher'})
        self.assertEqual(store.read(self.path)['preferences'],{'marker':1})

    def test_input_update_preserves_camera(self):
        camera_settings.save(self.path,camera_settings.defaults('192.0.2.1','abc'))
        data=dict(mode='카메라',file='',loop=True,start='0',end='',fps=10)
        video_input_settings.save(self.path,data)
        self.assertEqual(video_input_settings.load(self.path),data)
        self.assertEqual(camera_settings.load(self.path)['cameras'][0]['serial'],'abc')
        self.assertEqual([p.name for p in self.path.parent.glob('*.json')],['settings.json'])

    def test_migrate_once_preserves_existing_new_file(self):
        folder=self.path.parent
        (folder/'cameras.json').write_text(json.dumps(camera_settings.defaults('192.0.2.1','abc')))
        (folder/'credentials.json').write_text('{"ip":"cipher"}')
        with patch.dict(store.os.environ,{'APPDATA':str(folder/'empty')}):
            store.migrate(self.path)
            store.update(self.path,{'marker':'keep'})
            store.migrate(self.path)
        self.assertEqual(store.read(self.path)['credentials'],{'ip':'cipher'})
        self.assertEqual(store.read(self.path)['marker'],'keep')
        self.assertTrue(list((folder/'before_unified_settings').iterdir()))

    def test_corrupt_document_not_overwritten(self):
        self.path.write_text('broken')
        with self.assertRaises(ValueError):store.update(self.path,{'marker':1})
        self.assertEqual(self.path.read_text(),'broken')

    def test_atomic_replace_failure_preserves_original(self):
        store.update(self.path,{'marker':1});original=self.path.read_bytes()
        with patch.object(Path,'replace',side_effect=OSError('failure')):
            with self.assertRaises(OSError):store.update(self.path,{'marker':2})
        self.assertEqual(self.path.read_bytes(),original)
        self.assertFalse(list(self.path.parent.glob('*.tmp')))
