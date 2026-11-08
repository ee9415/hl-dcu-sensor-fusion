import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch
import zipfile

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/hl_camera_bringup/scripts'))
from deploy_code_release import build_bundle
from release_runtime import ReleaseStore, launch, manifest_for, validate_manifest


class CodeReleaseTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(dir=ROOT/'artifacts')
        self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        self.app=self.root/'app';self.app.mkdir()
        for name,data in {'main.py':'print(1)','telemetry_server.py':'x=1',
            'settings.json':'{"x":1}','release_runtime.py':'# launcher',
            'backend-run.sh':'# service','requirements.txt':'numpy==1',
            'install-runtime.sh':'# runtime','oakapp.toml':'[base_image]\nimage_tag="fixed"'}.items():
            (self.app/name).write_text(data)
        self.store=ReleaseStore(self.root/'persistent',self.app)

    def bundle(self,version='one'):
        path=self.root/(version+'.zip')
        return path,build_bundle(self.app,version,path)

    def test_bundle_excludes_libraries_launcher_and_model(self):
        (self.app/'models').mkdir();(self.app/'models/model.bin').write_bytes(b'model')
        path,manifest=self.bundle()
        with zipfile.ZipFile(path) as archive:
            self.assertEqual(set(archive.namelist()),{'main.py','telemetry_server.py','settings.json','manifest.json'})
        self.assertLess(path.stat().st_size,3000)
        self.assertEqual(self.store.stage(path),manifest)

    def test_runtime_change_rejected_without_activation(self):
        path,_=self.bundle()
        (self.app/'requirements.txt').write_text('numpy==2')
        incompatible=ReleaseStore(self.store.root,self.app)
        with self.assertRaises(ValueError):incompatible.stage(path)
        self.assertIsNone(self.store.read_state()['active'])

    def test_corrupt_or_traversing_bundle_rejected(self):
        path,manifest=self.bundle()
        with zipfile.ZipFile(path,'a') as archive:archive.writestr('../escape.py','x=1')
        with self.assertRaises(ValueError):self.store.stage(path)
        self.assertFalse((self.store.root/'escape.py').exists())
        path,_=self.bundle('two')
        with zipfile.ZipFile(path,'r') as archive:
            contents={name:archive.read(name) for name in archive.namelist()}
        contents['main.py']=b'print(99)'
        with zipfile.ZipFile(path,'w') as archive:
            for name,data in contents.items():archive.writestr(name,data)
        with self.assertRaises(ValueError):self.store.stage(path)

    def test_release_survives_reopen_and_rollback_to_embedded(self):
        path,manifest=self.bundle();self.store.stage(path)
        self.store.activate(manifest['release'])
        reopened=ReleaseStore(self.store.root,self.app)
        self.assertEqual(reopened.read_state()['pending']['release'],manifest['release'])
        with self.assertRaises(ValueError):reopened.rollback()
        reopened.write_state(dict(active=manifest['release'],previous=None,pending=None))
        reopened.rollback()
        self.assertIsNone(reopened.read_state()['active'])
        self.assertEqual(reopened.read_state()['previous'],manifest['release'])

    def test_failed_startup_automatically_restores_previous(self):
        path,manifest=self.bundle();self.store.stage(path);self.store.activate(manifest['release'])
        failed=Mock();failed.poll.return_value=1
        recovery=Mock();recovery.poll.return_value=0;recovery.wait.return_value=0
        with patch('release_runtime.subprocess.Popen',side_effect=[failed,recovery]) as spawn, \
                patch('release_runtime.signal.signal'):
            self.assertEqual(launch(self.store),0)
        self.assertIsNone(self.store.read_state()['active'])
        self.assertIsNone(self.store.read_state()['pending'])
        self.assertIn('failed startup',self.store.read_state()['last_failure'])
        self.assertEqual(spawn.call_args.kwargs['env']['OAK_CODE_RELEASE'],'embedded')

    def test_successful_startup_confirms_release(self):
        path,manifest=self.bundle();self.store.stage(path);self.store.activate(manifest['release'])
        child=Mock();child.poll.side_effect=[None,0];child.wait.return_value=0
        with patch('release_runtime.subprocess.Popen',return_value=child), \
                patch('release_runtime.signal.signal'),patch('release_runtime.healthy',return_value=True):
            self.assertEqual(launch(self.store),0)
        state=self.store.read_state()
        self.assertEqual(state['active'],manifest['release']);self.assertIsNone(state['pending'])

    def test_interrupted_confirmation_does_not_retry_candidate(self):
        path,manifest=self.bundle();self.store.stage(path);self.store.activate(manifest['release'])
        state=self.store.read_state();state['pending']['attempted']=True;self.store.write_state(state)
        child=Mock();child.poll.return_value=0;child.wait.return_value=0
        with patch('release_runtime.subprocess.Popen',return_value=child) as spawn,patch('release_runtime.signal.signal'):
            launch(self.store)
        self.assertEqual(spawn.call_args.kwargs['env']['OAK_CODE_RELEASE'],'embedded')
        self.assertIsNone(self.store.read_state()['pending'])
