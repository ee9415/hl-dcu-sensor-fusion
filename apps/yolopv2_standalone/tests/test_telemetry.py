import csv
from pathlib import Path
import sys
import tempfile
import time
import unittest

from telemetry_server import TelemetryServer

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'tools'))
from frame_csv_logger import FrameCsvLogger


class TelemetryTests(unittest.TestCase):
    def test_csv_roundtrip_and_gap(self):
        server = TelemetryServer(0)
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts') as directory:
            logger = FrameCsvLogger('127.0.0.1', directory, server.server.server_address[1])
            try:
                logger.start()
                deadline = time.monotonic()+5
                while not server.clients and time.monotonic() < deadline:
                    time.sleep(.01)
                self.assertTrue(server.clients)
                obj = dict(label=2, confidence=.9, xmin=.1, ymin=.2, xmax=.3,
                           ymax=.4, depth_valid=False, x_m=None, y_m=None, z_m=None)
                for seq in (1, 3, 3, 4):
                    server.publish(dict(frame_seq=seq, capture_wall_est_s=time.time()+(1 if seq == 4 else -.1),
                                        capture_monotonic_est_s=time.monotonic()+(1 if seq == 4 else -.1),
                                        coefficients=None, objects=[obj] if seq == 1 else [],
                                        centerline_valid=False, boundary_valid=seq != 4,
                                        boundary_pair_valid=seq == 1,
                                        left_boundary_valid=seq != 4, right_boundary_valid=seq == 1,
                                        boundary_state='inferred' if seq != 4 else 'road_unavailable',
                                        left_boundary_state='inferred', right_boundary_state='inferred',
                                        vehicle_width_m=1.8, left_margin_m=.3, right_margin_m=.3,
                                        corridor_width_m=2.4,
                                        minimum_clearance_width_m=2.4,
                                        left_boundary_coefficients=[-1.2,0.,.004],
                                        right_boundary_coefficients=[1.2,-.001,.0041],
                                        normal_width_error_dev=.01,road_coverage=.95,corridor_feasible=True,
                                        boundary_width_min_dev=2.4 if seq == 1 else None,
                                        boundary_width_max_dev=2.4 if seq == 1 else None,
                                        clearance_sufficient=True if seq == 1 else None,
                                        boundary_units='uncalibrated_bev_development_units',
                                        boundary_forward_dev=[1., 2.] if seq != 4 else [],
                                        left_boundary_dev=[-1.2, -1.2] if seq != 4 else [],
                                        right_boundary_dev=[1.2, 1.2] if seq == 1 else ([None,None] if seq == 3 else [])))
                deadline = time.monotonic()+5
                while logger.frames < 3 and time.monotonic() < deadline:
                    time.sleep(.01)
                logger.close()
                with (logger.path/'frames.csv').open(encoding='utf-8-sig') as f:
                    rows = list(csv.DictReader(f))
                with (logger.path/'objects.csv').open(encoding='utf-8-sig') as f:
                    objects = list(csv.DictReader(f))
                self.assertEqual(len(rows), 3)
                self.assertEqual(logger.gaps, 1)
                self.assertEqual(rows[1]['object_count'], '0')
                self.assertEqual(rows[0]['c0'], '')
                self.assertEqual(rows[0]['boundary_state'], 'inferred')
                self.assertEqual(float(rows[0]['corridor_width_m']), 2.4)
                self.assertEqual(float(rows[0]['minimum_clearance_width_m']), 2.4)
                self.assertEqual(float(rows[0]['boundary_width_min_dev']), 2.4)
                self.assertEqual(rows[2]['boundary_width_max_dev'], '')
                import json
                self.assertEqual(json.loads(rows[0]['left_boundary_dev']), [-1.2, -1.2])
                self.assertEqual(json.loads(rows[0]['left_boundary_coefficients']),[-1.2,0.,.004])
                self.assertEqual(json.loads(rows[0]['right_boundary_coefficients']),[1.2,-.001,.0041])
                self.assertEqual(rows[0]['corridor_feasible'],'True')
                self.assertEqual(float(rows[0]['normal_width_error_dev']),.01)
                self.assertEqual(json.loads(rows[2]['right_boundary_dev']), [])
                self.assertEqual(json.loads(rows[1]['right_boundary_dev']), [None,None])
                self.assertEqual(rows[1]['left_boundary_valid'], 'True')
                self.assertEqual(rows[1]['right_boundary_valid'], 'False')
                self.assertEqual(rows[1]['boundary_pair_valid'], 'False')
                self.assertEqual(rows[1]['clearance_sufficient'], '')
                self.assertEqual(objects[0]['z_m'], '')
                self.assertEqual(rows[0]['latency_status'], 'estimated_not_ptp')
                self.assertGreater(float(rows[0]['receive_delay_est_ms']), 50)
                self.assertEqual(rows[2]['receive_delay_est_ms'], '')
                self.assertEqual(rows[2]['latency_status'], 'invalid_clock_or_asymmetry')
            finally:
                logger.close()
                server.server.shutdown()
                server.server.server_close()
