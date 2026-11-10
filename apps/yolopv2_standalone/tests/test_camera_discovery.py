"""camera_discovery(OAK4/OAK-D 카메라 자동 탐색) 출력 파싱 및 폴백 동작 테스트."""
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools'))
import camera_discovery as discovery


class CameraDiscoveryTests(unittest.TestCase):
    def test_oak4_notice_and_json(self):
        """oakctl 출력 앞에 업데이트 안내 문구가 섞여 있어도 JSON 부분을 찾아 파싱한다."""
        data='Update available\n'+json.dumps(dict(items=[dict(serial_number=123,ip_addresses=['192.0.2.2'],model='OAK4')]))
        result=discovery.parse_oak4(data)
        self.assertEqual(result[0]['ip'],'192.0.2.2')
        self.assertEqual(result[0]['serial'],'123')

    def test_depthai_marker_tolerates_sdk_logs(self):
        """DepthAI 탐색 결과는 OAKDISCOVERY 마커 뒤 JSON만 읽고, 마커가 없으면 오류로 처리한다."""
        result=discovery.parse_oakd('SDK warning\nOAKDISCOVERY '+json.dumps([dict(serial='abc')]))
        self.assertEqual(result[0]['serial'],'abc')
        with self.assertRaises(ValueError):discovery.parse_oakd('No devices marker')

    def test_wsl_empty_local_search_uses_windows_and_keeps_oak4(self):
        """WSL에서 로컬 탐색 결과가 비어 있으면 Windows 측 탐색으로 폴백한다."""
        device=dict(family='oakd',serial='abc',ip='192.0.2.5')
        def run(command):
            # oakctl·depthai 모두 빈 결과를 돌려주는 가짜 명령 실행기
            return '{"items":[]}' if 'device' in command else 'OAKDISCOVERY []'
        with patch.object(discovery,'run',side_effect=run),patch.object(discovery,'windows_fallback',return_value=[device]) as fallback,patch.dict(discovery.os.environ,{'WSL_DISTRO_NAME':'Ubuntu'}):
            found,warnings=discovery.discover_cameras('oakctl','python')
        self.assertEqual(found,[device]);self.assertFalse(warnings);fallback.assert_called_once()

    def test_partial_failure_does_not_discard_other_family(self):
        """한 계열(oakctl/OAK4) 탐색이 실패해도 다른 계열(OAK-D) 결과는 유지하고 경고만 남긴다."""
        device=dict(family='oakd',serial='abc',ip='192.0.2.5')
        def run(command):
            # oakctl 호출만 실패시키고 depthai 탐색은 성공시킨다.
            if 'device' in command:raise RuntimeError('oakctl missing')
            return 'OAKDISCOVERY '+json.dumps([device])
        with patch.object(discovery,'run',side_effect=run),patch.object(discovery,'windows_fallback') as fallback:
            found,warnings=discovery.discover_cameras('oakctl','python')
        self.assertEqual(found,[device]);self.assertTrue(warnings);fallback.assert_not_called()


if __name__=='__main__':unittest.main()
