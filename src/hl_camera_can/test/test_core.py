from dataclasses import replace
import copy
import json
import math
from pathlib import Path
import re
import shutil
import struct
import subprocess
import unittest

import numpy as np

from hl_camera_can.path import Target, load_config, validate_config, ground_points, target_from_result
from hl_camera_can.bridge import PathGate
from hl_camera_can.protocol import crc8, packet, check_packet
from hl_camera_can.driver import socketcan_frame

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[1]


def config():
    return load_config(PACKAGE/'config/camera_can.json')


def good_target(sequence, when):
    return Target(capture_s=100+when, source_id='camera:session1', frame_sequence=sequence,
        coefficients=(.1, .01, .001, 0.), valid_length_m=5., observed_start_m=.5,
        confidence=90, valid=True, calibrated=True, timing_verified=True,
        capture_uncertainty_ms=20, reason='observed')


class ProtocolTests(unittest.TestCase):
    def test_j1850_standard_check_vector(self):
        self.assertEqual(crc8(b'123456789'), 0x4b)

    def test_active_layout_endian_speeds_and_reserved(self):
        data = packet(255, True, (1., -.25, .125, -.0625), 4.567, 87)
        self.assertEqual(data[:4], bytes.fromhex('d9 ff 02 00'))
        self.assertEqual(data[4:20].hex(), '0000803f000080be0000003e000080bd')
        self.assertEqual(data[20:24], bytes.fromhex('00 00 2c 01'))
        self.assertEqual(struct.unpack_from('<H', data, 24)[0], 456)
        self.assertEqual(data[26:31], bytes([87, 0, 0, 0, 0]))
        check_packet(data)
        damaged = bytearray(data); damaged[12] ^= 1
        with self.assertRaises(ValueError): check_packet(damaged)

    def test_inactive_zeroes_every_field_except_sequence_header_crc(self):
        data = packet(5, False, (1., 1., 1., 1.), 5., 90)
        self.assertEqual(data[:2], b'\x81\x05')
        self.assertEqual(data[2:31], bytes(29))
        check_packet(data)

    def test_invalid_active_values_are_rejected_not_clamped(self):
        for coefficients in [(math.nan, 0, 0, 0), (math.inf, 0, 0, 0), (3.01, 0, 0, 0), (0, 2.01, 0, 0)]:
            with self.assertRaises(ValueError): packet(0, True, coefficients, 1, 50)
        for length in [.49, 12.01, math.nan]:
            with self.assertRaises(ValueError): packet(0, True, (0,)*4, length, 50)
        for confidence in [0, 101, 2.5]:
            with self.assertRaises(ValueError): packet(0, True, (0,)*4, 1, confidence)

    def test_socketcan_uses_32_byte_length_not_dlc_13(self):
        data = packet(3)
        frame = socketcan_frame(data)
        self.assertEqual(len(frame), 72)
        identifier, length, flags, r0, r1, payload = struct.unpack('=IBBBB64s', frame)
        self.assertEqual((identifier, length, flags, r0, r1), (0x320, 32, 1, 0, 0))
        self.assertEqual(payload, data+bytes(32))

    @unittest.skipUnless(shutil.which('cc'), 'C compiler required for original AURIX CRC oracle')
    def test_packets_match_original_aurix_crc_function(self):
        # Compile the actual receiver's two CRC functions, not a Python port.
        source_path = ROOT/('halla_univ/곽병학 교수님/3. 원주미래산업진흥원/2026년 사업/'
            '2026년 미래모빌리티 기업지원 사업/4. 결과보고서/ECL 결과보고서/'
            '1113_fix/1024/0_Src/AppSw/Tricore/Can_AdasRx.c')
        if not source_path.exists():
            self.skipTest('1113_fix reference source is not present')
        text = source_path.read_text(encoding='utf-8')
        functions = [re.search(r'static uint8 '+name+r'\(.*?^\}', text, re.S | re.M).group()
                     for name in ('CanAdasRx_Crc8Update', 'CanAdasRx_CalculateCrc')]
        work = ROOT/'artifacts/camera_can/tests'
        work.mkdir(parents=True, exist_ok=True)
        source = work/'aurix_crc_oracle.c'
        source.write_text('#include <stdint.h>\n#include <stdio.h>\n'
            'typedef uint8_t uint8; typedef uint16_t uint16;\n'
            '#define TRAJ_FD_CRC8_POLY 0x1d\n#define TRAJ_FD_CRC8_INIT 0xff\n'
            '#define TRAJ_FD_CRC8_XOROUT 0xff\n'+'\n'.join(functions)+
            '\nint main(void){unsigned int x; uint8 data[31];'
            'for(int i=0;i<31;i++){if(scanf("%x",&x)!=1)return 1;data[i]=x;}'
            'printf("%u\\n",CanAdasRx_CalculateCrc(0x320,data,31));return 0;}\n', encoding='utf-8')
        exe = work/'aurix_crc_oracle'
        subprocess.run(['cc', '-std=c99', '-Wall', '-Werror', str(source), '-o', str(exe)], check=True)
        for sequence in [0, 1, 127, 255]:
            for active in [False, True]:
                data = packet(sequence, active, (.1, -.1, .02, -.001), 5.1, 80)
                result = subprocess.check_output([str(exe)], input=' '.join(f'{v:02x}' for v in data[:31]), text=True)
                self.assertEqual(int(result), data[31])


class GateTests(unittest.TestCase):
    def setUp(self):
        self.cfg = config()
        self.gate = PathGate(self.cfg, 0.)

    def warm(self):
        for seq, when in enumerate([.5, .6, .7]):
            self.gate.update(good_target(seq, when), 100+when, when)

    def test_startup_quiet_and_inactive_allow_receiver_resync(self):
        self.assertIsNone(self.gate.tick(100.1, .1))
        self.warm()
        self.gate.started = .4
        data = self.gate.tick(100.7, .7)
        self.assertEqual(data[0], 0x81)

    def test_requires_three_distinct_frames_then_expires_by_capture_time(self):
        for seq, when in enumerate([.5, .6]):
            self.gate.update(good_target(seq, when), 100+when, when)
            self.assertEqual(self.gate.tick(100+when, when)[0], 0x81)
        self.gate.update(good_target(2, .7), 100.7, .7)
        self.assertEqual(self.gate.tick(100.7, .7)[0], 0xd9)
        expired = self.gate.tick(101., 1.)
        self.assertEqual(expired[0], 0x81)
        self.assertEqual(expired[2:31], bytes(29))

    def test_duplicate_never_reactivates_or_renews_freshness(self):
        for _ in range(3):self.gate.update(good_target(1, .5), 100.5, .5)
        self.assertEqual(self.gate.streak, 1)
        self.assertEqual(self.gate.tick(100.6, .6)[0], 0x81)

    def test_frozen_ros_clock_still_expires_on_monotonic_clock(self):
        self.warm()
        self.assertEqual(self.gate.tick(100.7, .7)[0], 0xd9)
        self.assertEqual(self.gate.tick(100.7, 1.1)[0], 0x81)

    def test_invalid_observation_revokes_immediately_and_needs_three_new(self):
        self.warm()
        self.gate.update(replace(good_target(3, .8), valid=False, reason='lost_lane'), 100.8, .8)
        self.assertEqual(self.gate.tick(100.8, .8)[0], 0x81)
        for seq, when in [(4, .9), (5, 1.)]:
            self.gate.update(good_target(seq, when), 100+when, when)
            self.assertEqual(self.gate.tick(100+when, when)[0], 0x81)
        self.gate.update(good_target(6, 1.1), 101.1, 1.1)
        self.assertEqual(self.gate.tick(101.1, 1.1)[0], 0xd9)

    def test_future_old_and_uncertain_capture_are_invalid(self):
        for target in [good_target(1, .9), good_target(1, 0.),
                       replace(good_target(1, .5), capture_uncertainty_ms=1000)]:
            self.gate.update(target, 100.5, .5)
            self.assertEqual(self.gate.tick(100.5, .5)[0], 0x81)

    def test_bench_requires_explicit_bridge_permission(self):
        target = replace(good_target(1, .5), calibrated=False, timing_verified=False, bench_only=True)
        self.gate.update(target, 100.5, .5)
        self.assertIsNone(self.gate.target)

    def test_source_change_and_reorder_reset_warmup(self):
        self.warm()
        self.gate.update(replace(good_target(3, .8), source_id='new-session'), 100.8, .8)
        self.assertEqual(self.gate.streak, 1)
        self.gate.update(replace(good_target(2, .7), source_id='new-session'), 100.8, .8)
        self.assertIsNone(self.gate.target)

    def test_sequence_wraps_even_when_inactive(self):
        self.gate.sequence = 255
        self.assertEqual(self.gate.tick(101., 1.)[1], 255)
        self.assertEqual(self.gate.tick(101.05, 1.05)[1], 0)

    def test_clock_backwards_revokes_active_path(self):
        self.warm()
        self.gate.tick(100.7, .7)
        self.assertEqual(self.gate.tick(100.6, .75)[0], 0x81)


class GeometryTests(unittest.TestCase):
    def example(self):
        cfg = config()
        cfg['geometry'].update(calibrated=True, timing_verified=True, capture_uncertainty_ms=20,
            fx=100., fy=100., camera_height_m=.5, camera_forward_m=0., pitch_down_deg=20.)
        cfg['quality']['max_near_extrapolation_m'] = 1.2
        g = cfg['geometry']
        x = np.linspace(1., 5., 40)
        center = .1+.01*x+.001*x*x
        pitch = math.radians(20)
        def pixels(y):
            z = math.cos(pitch)*x+math.sin(pitch)*.5
            return np.column_stack((g['cx']-100*y/z,
                g['cy']+100*(-math.sin(pitch)*x+math.cos(pitch)*.5)/z)).tolist()
        result = dict(stamp=dict(sec=100, nanosec=0), source=dict(input_source='camera', source_epoch=1),
            session='s', frame_seq=1, stale=False, geometry_units='image_pixels_320x180',
            boundary_pair_valid=True, left_boundary_state='observed', right_boundary_state='observed',
            left_boundary_confidence=.95, right_boundary_confidence=.95,
            left_boundary_supported_image=pixels(center+.9), right_boundary_supported_image=pixels(center-.9))
        return cfg, result

    def test_metric_midpoint_sign_fit_and_supported_distance(self):
        cfg, result = self.example()
        target = target_from_result(result, cfg)
        self.assertTrue(target.valid, target.reason)
        np.testing.assert_allclose(target.coefficients, [.1, .01, .001, 0], atol=1e-4)
        self.assertAlmostEqual(target.valid_length_m, 5.)
        self.assertAlmostEqual(target.observed_start_m, 1.)

    def test_uncalibrated_default_never_produces_active_candidate(self):
        _, result = self.example()
        self.assertFalse(target_from_result(result, config()).valid)

    def test_held_lane_pc_input_and_excessive_near_gap_are_invalid(self):
        cfg, result = self.example()
        held = copy.deepcopy(result); held['left_boundary_state'] = 'held'
        pc = copy.deepcopy(result); pc['source']['input_source'] = 'pc'
        for changed in [held, pc]:self.assertFalse(target_from_result(changed, cfg).valid)
        cfg['quality']['max_near_extrapolation_m'] = .8
        self.assertFalse(target_from_result(result, cfg).valid)

    def test_bench_candidate_is_marked_and_never_claims_calibrated(self):
        cfg, result = self.example()
        cfg['geometry'].update(calibrated=False, timing_verified=False, bench_test_active=True)
        result['source']['input_source'] = 'pc'
        target = target_from_result(result, cfg)
        self.assertTrue(target.valid, target.reason)
        self.assertTrue(target.bench_only)
        self.assertFalse(target.calibrated)

    def test_bad_pixels_and_configuration_are_not_clamped(self):
        cfg, result = self.example()
        result['left_boundary_supported_image'][0][0] = float('nan')
        self.assertFalse(target_from_result(result, cfg).valid)
        cfg['can']['startup_quiet_ms'] = 100
        with self.assertRaises(ValueError): validate_config(cfg)

    def test_roll_pitch_yaw_projection_preserves_left_positive_vehicle_coordinates(self):
        cfg, _ = self.example()
        g = cfg['geometry']; g.update(roll_deg=10., yaw_left_deg=5.)
        r, p, y = np.deg2rad([10., 20., 5.])
        rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
        ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
        rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
        ray = (rz@ry@rx).T@np.array([3., .5, -.5])
        pixels = [[g['cx']-g['fx']*ray[1]/ray[0], g['cy']-g['fy']*ray[2]/ray[0]]]
        np.testing.assert_allclose(ground_points(pixels, g), [[3., .5]], atol=1e-9)


if __name__ == '__main__':
    unittest.main()
