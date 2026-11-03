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
        data='Update available\n'+json.dumps(dict(items=[dict(serial_number=123,ip_addresses=['192.0.2.2'],model='OAK4')]))
        result=discovery.parse_oak4(data)
        self.assertEqual(result[0]['ip'],'192.0.2.2')
        self.assertEqual(result[0]['serial'],'123')

    def test_depthai_marker_tolerates_sdk_logs(self):
        result=discovery.parse_oakd('SDK warning\nOAKDISCOVERY '+json.dumps([dict(serial='abc')]))
        self.assertEqual(result[0]['serial'],'abc')
        with self.assertRaises(ValueError):discovery.parse_oakd('No devices marker')

    def test_wsl_empty_local_search_uses_windows_and_keeps_oak4(self):
        device=dict(family='oakd',serial='abc',ip='192.0.2.5')
        def run(command):
            return '{"items":[]}' if 'device' in command else 'OAKDISCOVERY []'
        with patch.object(discovery,'run',side_effect=run),patch.object(discovery,'windows_fallback',return_value=[device]) as fallback,patch.dict(discovery.os.environ,{'WSL_DISTRO_NAME':'Ubuntu'}):
            found,warnings=discovery.discover_cameras('oakctl','python')
        self.assertEqual(found,[device]);self.assertFalse(warnings);fallback.assert_called_once()

    def test_partial_failure_does_not_discard_other_family(self):
        device=dict(family='oakd',serial='abc',ip='192.0.2.5')
        def run(command):
            if 'device' in command:raise RuntimeError('oakctl missing')
            return 'OAKDISCOVERY '+json.dumps([device])
        with patch.object(discovery,'run',side_effect=run),patch.object(discovery,'windows_fallback') as fallback:
            found,warnings=discovery.discover_cameras('oakctl','python')
        self.assertEqual(found,[device]);self.assertTrue(warnings);fallback.assert_not_called()


if __name__=='__main__':unittest.main()
