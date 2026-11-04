import json
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
import oak_app_control as control


class AppControlTests(unittest.TestCase):
    def test_switch_starts_installed_app_and_disables_autostart(self):
        apps = [dict(identifier=control.IDENTIFIER, app_id='ai', status='running'),
                dict(identifier=control.RGB_IDENTIFIER, app_id='rgb', status='stopped'),
                dict(identifier='unrelated', app_id='other', status='running')]
        with (patch.object(control, 'list_apps', return_value=apps),
              patch.object(control, 'command') as command):
            self.assertEqual(control.start_installed('cli', 'ip', control.RGB_IDENTIFIER), 'rgb')
            self.assertEqual([c.args[2] for c in command.call_args_list],
                             [['stop', 'ai'], ['start', 'rgb', '--enable', 'false']])

    def test_development_app_is_not_accepted_as_installed(self):
        apps = [dict(identifier=control.RGB_IDENTIFIER, app_id=control.DEVELOPMENT_ID, status='running')]
        with (patch.object(control, 'list_apps', return_value=apps), patch.object(control, 'command') as command):
            with self.assertRaisesRegex(RuntimeError, '먼저 설치'):
                control.start_installed('cli', 'ip', control.RGB_IDENTIFIER)
            command.assert_not_called()

    def test_installed_app_takes_priority_over_development_app(self):
        apps = [dict(identifier=control.IDENTIFIER, app_id=control.DEVELOPMENT_ID, status='running'),
                dict(identifier=control.IDENTIFIER, app_id='installed', status='stopped')]
        with patch.object(control, 'command', return_value=json.dumps(apps)):
            self.assertEqual(control.status('cli', 'ip')['app_id'], 'installed')

    def query(self, status):
        payload = [dict(identifier=control.IDENTIFIER, app_id='target-id', status=status),
                   dict(identifier='other-app', app_id='other', status='running')]
        with patch.object(control, 'command', return_value='Update notice\n'+json.dumps(payload)):
            return control.status('oakctl', '192.0.2.1')

    def test_running(self):
        self.assertEqual(self.query('running')['state'], 'running')

    def test_stopped(self):
        self.assertEqual(self.query('stopped')['state'], 'stopped')

    def test_installed_ready_app_is_not_reported_as_running(self):
        self.assertEqual(self.query('ready')['state'], 'stopped')

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
