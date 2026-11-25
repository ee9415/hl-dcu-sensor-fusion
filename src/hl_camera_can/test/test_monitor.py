import json
import unittest
from test_core import ROOT
from hl_camera_can.monitor import CaptureLog, decode
from hl_camera_can.protocol import packet


class MonitorTests(unittest.TestCase):
    def test_active_decode_matches_wire_values(self):
        result = decode(packet(42, True, (.5, -.25, .125, -.0625), 4.567, 87))
        self.assertEqual(result['coefficients'], (.5, -.25, .125, -.0625))
        self.assertEqual((result['sequence'], result['mode'], result['valid_length_m']), (42, 'ACTIVE', 4.56))
        self.assertEqual(result['max_speed_kph'], 3.)
        self.assertTrue(result['crc_ok'])
        self.assertTrue(result['valid'])

    def test_corruption_and_wrong_length_are_visible(self):
        damaged = bytearray(packet(1)); damaged[10] ^= 1
        self.assertFalse(decode(damaged)['crc_ok'])
        self.assertFalse(decode(damaged)['valid'])
        self.assertEqual(decode(b'\x81')['length'], 1)
        self.assertFalse(decode(b'\x81')['valid'])

    def test_log_records_all_frames_sequence_wrap_and_errors(self):
        path = ROOT/'artifacts/camera_can/tests/monitor.jsonl'
        if path.exists():path.write_text('')
        log = CaptureLog(path)
        for seq in (255, 0, 0, 3):log.capture(packet(seq), 0)
        log.capture(packet(4), 0, 'expired')
        rows = [json.loads(line) for line in path.read_text().splitlines()]
        self.assertEqual([row['sequence_note'] for row in rows[:4]], ['first', 'OK', 'duplicate', 'gap +2'])
        self.assertEqual(log.sequence_errors, 2)
        self.assertEqual(log.invalid, 1)
        self.assertTrue(all(not row['physical_sent'] for row in rows))


if __name__ == '__main__':unittest.main()
