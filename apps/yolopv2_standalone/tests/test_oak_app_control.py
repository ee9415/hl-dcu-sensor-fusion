"""oak_app_control(oakctl로 카메라 앱 상태 조회/시작/정지) 테스트."""
import json
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
import oak_app_control as control


class AppControlTests(unittest.TestCase):
    def test_one_running_install_can_coexist_with_stopped_rollback(self):
        """실행 중인 설치본 1개와 정지된 롤백본이 함께 있으면 실행 중인 쪽을 대상으로 고르고, 추가 명령은 보내지 않는다."""
        apps = [dict(identifier=control.IDENTIFIER, app_id='rollback', status='ready'),
                dict(identifier=control.IDENTIFIER, app_id='current', status='running')]
        with (patch.object(control, 'list_apps', return_value=apps),
              patch.object(control, 'command') as command):
            self.assertEqual(control.status('cli', 'ip')['app_id'], 'current')
            self.assertEqual(control.start_installed('cli', 'ip', control.IDENTIFIER), 'current')
            command.assert_not_called()

    def test_ambiguous_installs_still_require_explicit_choice(self):
        """같은 상태의 설치본이 둘 이상이면 임의로 고르지 않고 오류를 낸다."""
        for state in ('running', 'ready'):
            apps = [dict(identifier=control.IDENTIFIER, app_id=str(i), status=state) for i in range(2)]
            with (patch.object(control, 'list_apps', return_value=apps),
                  patch.object(control, 'command') as command):
                with self.assertRaises(RuntimeError):
                    control.status('cli', 'ip')
                with self.assertRaises(RuntimeError):
                    control.start_installed('cli', 'ip', control.IDENTIFIER)
                command.assert_not_called()

    def test_switch_starts_installed_app_and_disables_autostart(self):
        """다른 앱으로 전환할 때 실행 중인 우리 앱을 멈추고 대상 앱을 자동 시작 해제 상태로 시작한다. 무관한 앱은 건드리지 않는다."""
        apps = [dict(identifier=control.IDENTIFIER, app_id='ai', status='running'),
                dict(identifier=control.RGB_IDENTIFIER, app_id='rgb', status='stopped'),
                dict(identifier='unrelated', app_id='other', status='running')]
        with (patch.object(control, 'list_apps', return_value=apps),
              patch.object(control, 'command') as command):
            self.assertEqual(control.start_installed('cli', 'ip', control.RGB_IDENTIFIER), 'rgb')
            self.assertEqual([c.args[2] for c in command.call_args_list],
                             [['stop', 'ai'], ['start', 'rgb', '--enable', 'false']])

    def test_development_app_is_not_accepted_as_installed(self):
        """개발용(oakctl run) 앱은 설치본으로 인정하지 않는다."""
        apps = [dict(identifier=control.RGB_IDENTIFIER, app_id=control.DEVELOPMENT_ID, status='running')]
        with (patch.object(control, 'list_apps', return_value=apps), patch.object(control, 'command') as command):
            with self.assertRaisesRegex(RuntimeError, '먼저 설치'):
                control.start_installed('cli', 'ip', control.RGB_IDENTIFIER)
            command.assert_not_called()

    def test_installed_app_takes_priority_over_development_app(self):
        """개발용 앱과 설치본이 함께 있으면 설치본을 우선한다."""
        apps = [dict(identifier=control.IDENTIFIER, app_id=control.DEVELOPMENT_ID, status='running'),
                dict(identifier=control.IDENTIFIER, app_id='installed', status='stopped')]
        with patch.object(control, 'command', return_value=json.dumps(apps)):
            self.assertEqual(control.status('cli', 'ip')['app_id'], 'installed')

    def query(self, status):
        """주어진 status로 oakctl 응답(앞에 업데이트 안내 포함)을 흉내 내고 status() 결과를 반환한다."""
        payload = [dict(identifier=control.IDENTIFIER, app_id='target-id', status=status),
                   dict(identifier='other-app', app_id='other', status='running')]
        with patch.object(control, 'command', return_value='Update notice\n'+json.dumps(payload)):
            return control.status('oakctl', '192.0.2.1')

    def test_running(self):
        self.assertEqual(self.query('running')['state'], 'running')

    def test_stopped(self):
        self.assertEqual(self.query('stopped')['state'], 'stopped')

    def test_installed_ready_app_is_not_reported_as_running(self):
        """ready(설치만 됨) 상태는 running이 아니라 stopped로 보고한다."""
        self.assertEqual(self.query('ready')['state'], 'stopped')

    def test_missing(self):
        """앱이 없으면 app_id가 None이다."""
        with patch.object(control, 'command', return_value='[]'):
            self.assertIsNone(control.status('oakctl', '192.0.2.1')['app_id'])

    def test_unknown_not_shown_as_stopped(self):
        """알 수 없는 상태(starting 등)를 stopped로 오인하지 않고 오류로 처리한다."""
        with self.assertRaises(RuntimeError):
            self.query('starting')

    def test_corrupt(self):
        """JSON이 아닌 응답은 오류로 처리한다."""
        with patch.object(control, 'command', return_value='bad response'):
            with self.assertRaises(RuntimeError):
                control.status('oakctl', '192.0.2.1')

    def test_stop_target_only(self):
        """stop은 지정한 app_id만 정지한다."""
        with patch.object(control, 'command') as command:
            control.stop('oakctl', '192.0.2.1', 'target-id', 'secret')
            self.assertEqual(command.call_args.args[2], ['stop', 'target-id'])

    def test_timeout_does_not_expose_password(self):
        """명령 타임아웃 오류 메시지에 비밀번호가 노출되지 않는다."""
        with patch.object(control.subprocess, 'run', side_effect=subprocess.TimeoutExpired(['--password','secret'], 30)):
            with self.assertRaises(RuntimeError) as error:
                control.status('oakctl', '192.0.2.1', 'secret')
            self.assertNotIn('secret', str(error.exception))
