"""camera_settings(카메라 IP/시리얼 설정 파일 cameras.json) 저장·로드·백업·마이그레이션 테스트."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools'))
import camera_settings as settings


class CameraSettingsTests(unittest.TestCase):
    def test_unchanged_save_does_not_destroy_previous_backup(self):
        """내용이 바뀌지 않은 저장은 .bak 백업을 덮어쓰지 않는다."""
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'cameras.json';data=settings.defaults('192.0.2.1','abc')
            settings.save(path,data)
            data['cameras'][0]['ip']='192.0.2.2';settings.save(path,data)
            backup=path.with_suffix('.json.bak').read_bytes()
            settings.save(path,data)
            self.assertEqual(path.with_suffix('.json.bak').read_bytes(),backup)

    def test_roundtrip_and_previous_copy(self):
        """저장 후 로드 결과가 같고, 변경 저장 시 직전 내용이 .bak으로 남는다."""
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'cameras.json';data=settings.defaults('192.0.2.1','abc','python')
            settings.save(path,data);first=settings.load(path)
            data['cameras'][1].update(ip='192.0.2.2',serial='def',selected=True)
            settings.save(path,data)
            self.assertEqual(settings.load(path)['cameras'][1]['serial'],'def')
            self.assertEqual(json.loads(path.with_suffix('.json.bak').read_text()),first)

    def test_invalid_duplicate_does_not_overwrite_saved_profile(self):
        """IP가 중복된 잘못된 설정은 거부되고 기존 파일은 그대로 남는다."""
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'cameras.json';data=settings.defaults('192.0.2.1','abc')
            settings.save(path,data);before=path.read_bytes()
            data['cameras'][1]['ip']='192.0.2.1'
            with self.assertRaises(ValueError):settings.save(path,data)
            self.assertEqual(path.read_bytes(),before)

    def test_replace_failure_preserves_original_and_cleans_temporary(self):
        """원자적 교체(replace) 중 실패해도 원본은 보존되고 임시(.tmp) 파일은 정리된다."""
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'cameras.json';data=settings.defaults('192.0.2.1','abc')
            settings.save(path,data);before=path.read_bytes();data['cameras'][0]['ip']='192.0.2.2'
            with patch.object(Path,'replace',side_effect=OSError('replace failed')):
                with self.assertRaises(OSError):settings.save(path,data)
            self.assertEqual(path.read_bytes(),before)
            self.assertFalse(list(path.parent.glob('*.tmp')))

    def test_reset_is_a_saved_empty_profile(self):
        """초기화(reset)는 IP/시리얼이 모두 빈 프로필로 저장된다."""
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'cameras.json';data=settings.defaults();data['reset']=True
            settings.save(path,data);loaded=settings.load(path)
            self.assertTrue(loaded['reset'])
            self.assertTrue(all(not c['ip'] and not c['serial'] for c in loaded['cameras']))

    def test_migration_prefers_latest_real_recordings_and_ignores_virtual(self):
        """기존 녹화 session.json에서 설정을 가져올 때 가장 최근의 실제 카메라 녹화를 쓰고, 가상 입력은 무시한다."""
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            for stamp,kind,ip,serial in [('20260101','camera','192.0.2.2','old'),
                                         ('20260102','camera','192.0.2.3','latest'),
                                         ('20260103','virtual','192.0.2.4','fake')]:
                path=root/('six_camera_'+stamp)/'cam02/session.json';path.parent.mkdir(parents=True)
                path.write_text(json.dumps(dict(camera='CAM02',input_kind=kind,ip=ip,frames=10,source_status=dict(mxid=serial))))
            data,count=settings.migrate_recordings(root,settings.defaults())
            self.assertEqual(count,1)
            self.assertEqual(data['cameras'][1]['serial'],'latest')
            self.assertEqual(data['cameras'][1]['ip'],'192.0.2.3')

    def test_corrupt_file_is_reported_without_rewriting(self):
        """손상된 JSON은 오류로 알리고 파일을 다시 쓰지 않는다."""
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'cameras.json';path.write_text('{bad')
            with self.assertRaises(ValueError):settings.load(path)
            self.assertEqual(path.read_text(),'{bad')


if __name__=='__main__':unittest.main()
