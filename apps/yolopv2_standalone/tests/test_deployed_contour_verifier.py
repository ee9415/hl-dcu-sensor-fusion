"""Check that deployment evidence cannot hide jumps or source resets."""
import importlib.util
from pathlib import Path
import unittest


path = Path(__file__).resolve().parents[3]/'src/hl_camera_bringup/scripts/verify_deployed_contour.py'
spec = importlib.util.spec_from_file_location('verify_deployed_contour',path)
verifier = importlib.util.module_from_spec(spec)
spec.loader.exec_module(verifier)


def frame(time, state, shift=0., index=0, reset=False):
    """검증기 입력용 텔레메트리 프레임 하나를 만든다. shift만큼 좌측 경계선을 x 방향으로 이동시킨다."""
    return dict(boundary_model='mask_contour_temporal',source_epoch=1,
                nn_timestamp_s=time,frame_seq=index,source_frame_index=index,
                left_boundary_state=state,left_boundary_image=[[60.+shift,y] for y in range(90,170,5)],
                boundary_detection_diagnostics=[dict(reset=reset),dict(reset=reset)])


class DeploymentEvidenceTests(unittest.TestCase):
    def test_observed_to_missing_jump_is_measured(self):
        """observed → inferred 전환 시 경계선 이동량(2px)을 측정하고, 한도 이내면 위반이 아니다."""
        report = verifier.analyze([frame(0.,'observed'),frame(.1,'inferred',2.,1)])
        self.assertEqual(len(report['loss_transitions']),1)
        self.assertAlmostEqual(report['max_transition_px'],2.)
        self.assertFalse(report['limit_violations'])

    def test_jump_over_bound_fails(self):
        """전환 시 이동량(10px)이 한도를 넘으면 위반으로 보고된다."""
        report = verifier.analyze([frame(0.,'observed'),frame(.1,'held',10.,1)])
        self.assertEqual(len(report['limit_violations']),1)

    def test_reset_is_not_misreported_as_continuity(self):
        """추적기 reset 프레임은 연속 전환으로 집계하지 않는다."""
        report = verifier.analyze([frame(0.,'observed'),frame(.1,'held',10.,1,reset=True)])
        self.assertFalse(report['loss_transitions'])
        self.assertEqual(report['missed_side_samples'],0)

    def test_skipped_source_frames_do_not_claim_an_uninterrupted_contract(self):
        """소스 프레임이 건너뛰어졌으면 연속성 위반 대신 누락 구간 이동량으로 따로 보고한다."""
        report = verifier.analyze([frame(0.,'observed'),frame(.3,'held',15.,3)])
        self.assertEqual(report['max_missed_movement_px'],15.)
        self.assertFalse(report['limit_violations'])


if __name__ == '__main__':
    unittest.main()
