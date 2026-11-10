"""코드 전용 배포(deploy_code_release / release_runtime) 테스트.

번들 생성, 런타임 호환성 검사, 무결성 검증, 활성화/롤백, 기동 실패 시 자동 복구를 확인한다.
"""
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
        """임시 앱 폴더에 최소 파일 구성을 만들고 ReleaseStore를 준비한다."""
        self.temp=tempfile.TemporaryDirectory(dir=ROOT/'artifacts')
        self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        self.app=self.root/'app';self.app.mkdir()
        for name,data in {'main.py':'print(1)','telemetry_server.py':'x=1',
            'settings.json':'{"x":1}','release_runtime.py':'# launcher',
            'backend-run.sh':'# service','requirements.txt':'numpy==1',
            'install-runtime.sh':'# runtime','oakapp.toml':'[base_image]\nimage_tag="fixed"'}.items():
            (self.app/name).write_text(data)
        (self.app/'runtime-base.toml').write_text((self.app/'oakapp.toml').read_text())
        self.store=ReleaseStore(self.root/'persistent',self.app)

    def bundle(self,version='one'):
        """현재 앱 폴더로 배포 번들(zip)을 만들고 (경로, manifest)를 반환한다."""
        path=self.root/(version+'.zip')
        return path,build_bundle(self.app,version,path)

    def test_bundle_excludes_libraries_launcher_and_model(self):
        """번들에는 앱 코드와 manifest만 들어가고 런처/런타임 파일/모델은 제외된다."""
        (self.app/'models').mkdir();(self.app/'models/model.bin').write_bytes(b'model')
        path,manifest=self.bundle()
        with zipfile.ZipFile(path) as archive:
            self.assertEqual(set(archive.namelist()),{'main.py','telemetry_server.py','settings.json','manifest.json'})
        self.assertLess(path.stat().st_size,3000)
        self.assertEqual(self.store.stage(path),manifest)

    def test_runtime_change_rejected_without_activation(self):
        """requirements 등 런타임 구성이 다른 장치에서는 번들 스테이징이 거부되고 활성 릴리스도 바뀌지 않는다."""
        path,_=self.bundle()
        (self.app/'requirements.txt').write_text('numpy==2')
        incompatible=ReleaseStore(self.store.root,self.app)
        with self.assertRaises(ValueError):incompatible.stage(path)
        self.assertIsNone(self.store.read_state()['active'])

    def test_contract_works_without_oakctl_build_configuration(self):
        """oakapp.toml이 없어도 runtime-base.toml로 동일한 런타임 식별값을 계산한다."""
        expected=self.store.runtime
        (self.app/'oakapp.toml').unlink()
        self.assertEqual(ReleaseStore(self.store.root,self.app).runtime,expected)

    def test_base_image_drift_rejected_on_build_host(self):
        """빌드 호스트의 oakapp.toml 베이스 이미지가 runtime-base.toml과 다르면 번들 생성을 거부한다."""
        (self.app/'oakapp.toml').write_text('[base_image]\nimage_tag="different"')
        with self.assertRaises(ValueError):self.bundle()

    def test_corrupt_or_traversing_bundle_rejected(self):
        """경로 탈출(../) 항목이나 해시가 맞지 않는(변조된) 번들은 거부된다."""
        path,manifest=self.bundle()
        with zipfile.ZipFile(path,'a') as archive:archive.writestr('../escape.py','x=1')
        with self.assertRaises(ValueError):self.store.stage(path)
        self.assertFalse((self.store.root/'escape.py').exists())
        path,_=self.bundle('two')
        with zipfile.ZipFile(path,'r') as archive:
            contents={name:archive.read(name) for name in archive.namelist()}
        # manifest는 그대로 두고 코드만 바꿔 해시 불일치를 만든다.
        contents['main.py']=b'print(99)'
        with zipfile.ZipFile(path,'w') as archive:
            for name,data in contents.items():archive.writestr(name,data)
        with self.assertRaises(ValueError):self.store.stage(path)

    def test_release_survives_reopen_and_rollback_to_embedded(self):
        """활성화 상태는 재시작 후에도 유지되고, 롤백하면 내장(embedded) 코드로 돌아간다."""
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
        """새 릴리스 기동이 실패하면 자동으로 내장 코드로 복구 실행하고 실패 사유를 남긴다."""
        path,manifest=self.bundle();self.store.stage(path);self.store.activate(manifest['release'])
        # 첫 프로세스는 즉시 종료(실패), 두 번째(복구) 프로세스는 정상 종료
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
        """헬스 체크를 통과하면 pending 릴리스가 active로 확정된다."""
        path,manifest=self.bundle();self.store.stage(path);self.store.activate(manifest['release'])
        child=Mock();child.poll.side_effect=[None,0];child.wait.return_value=0
        with patch('release_runtime.subprocess.Popen',return_value=child), \
                patch('release_runtime.signal.signal'),patch('release_runtime.healthy',return_value=True):
            self.assertEqual(launch(self.store),0)
        state=self.store.read_state()
        self.assertEqual(state['active'],manifest['release']);self.assertIsNone(state['pending'])

    def test_interrupted_confirmation_does_not_retry_candidate(self):
        """이미 시도했던(attempted) 후보가 확정 전에 중단됐다면 재시도하지 않고 내장 코드로 기동한다."""
        path,manifest=self.bundle();self.store.stage(path);self.store.activate(manifest['release'])
        state=self.store.read_state();state['pending']['attempted']=True;self.store.write_state(state)
        child=Mock();child.poll.return_value=0;child.wait.return_value=0
        with patch('release_runtime.subprocess.Popen',return_value=child) as spawn,patch('release_runtime.signal.signal'):
            launch(self.store)
        self.assertEqual(spawn.call_args.kwargs['env']['OAK_CODE_RELEASE'],'embedded')
        self.assertIsNone(self.store.read_state()['pending'])
