import json
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
import oak_app_control as control


class AppControlTests(unittest.TestCase):
    def query(self, status):
        payload = [dict(identifier=control.IDENTIFIER, app_id='target-id', status=status),
                   dict(identifier='other-app', app_id='other', status='running')]
        with patch.object(control, 'command', return_value='Update notice\n'+json.dumps(payload)):
            return control.status('oakctl', '192.0.2.1')

    def test_running(self):
        self.assertEqual(self.query('running')['state'], 'running')

    def test_stopped(self):
        self.assertEqual(self.query('stopped')['state'], 'stopped')

    def test_missing(self):
        with patch.object(control, 'command', return_value='[]'):
            self.assertIsNone(control.status('oakctl', '192.0.2.1')['app_id'])

    def test_unknown_not_shown_as_stopped(self):
        with self.assertRaises(RuntimeError):
            self.query('starting')

    def test_corrupt(self):
        with patch.object(control, 'command', return_value='bad response'):
            with self.assertRaises(RuntimeError):
                control.status('oakctl', '192.0.2.1')

    def test_stop_target_only(self):
        with patch.object(control, 'command') as command:
            control.stop('oakctl', '192.0.2.1', 'target-id', 'secret')
            self.assertEqual(command.call_args.args[2], ['stop', 'target-id'])

    def test_timeout_does_not_expose_password(self):
        with patch.object(control.subprocess, 'run', side_effect=subprocess.TimeoutExpired(['--password','secret'], 30)):
            with self.assertRaises(RuntimeError) as error:
                control.status('oakctl', '192.0.2.1', 'secret')
            self.assertNotIn('secret', str(error.exception))
