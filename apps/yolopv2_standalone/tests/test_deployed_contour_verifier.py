"""Check that deployment evidence cannot hide jumps or source resets."""
import importlib.util
from pathlib import Path
import unittest


path = Path(__file__).resolve().parents[3]/'src/hl_camera_bringup/scripts/verify_deployed_contour.py'
spec = importlib.util.spec_from_file_location('verify_deployed_contour',path)
verifier = importlib.util.module_from_spec(spec)
spec.loader.exec_module(verifier)


def frame(time, state, shift=0., index=0, reset=False):
    return dict(boundary_model='mask_contour_temporal',source_epoch=1,
                nn_timestamp_s=time,frame_seq=index,source_frame_index=index,
                left_boundary_state=state,left_boundary_image=[[60.+shift,y] for y in range(90,170,5)],
                boundary_detection_diagnostics=[dict(reset=reset),dict(reset=reset)])


class DeploymentEvidenceTests(unittest.TestCase):
    def test_observed_to_missing_jump_is_measured(self):
        report = verifier.analyze([frame(0.,'observed'),frame(.1,'inferred',2.,1)])
        self.assertEqual(len(report['loss_transitions']),1)
        self.assertAlmostEqual(report['max_transition_px'],2.)
        self.assertFalse(report['limit_violations'])

    def test_jump_over_bound_fails(self):
        report = verifier.analyze([frame(0.,'observed'),frame(.1,'held',10.,1)])
        self.assertEqual(len(report['limit_violations']),1)

    def test_reset_is_not_misreported_as_continuity(self):
        report = verifier.analyze([frame(0.,'observed'),frame(.1,'held',10.,1,reset=True)])
        self.assertFalse(report['loss_transitions'])
        self.assertEqual(report['missed_side_samples'],0)

    def test_skipped_source_frames_do_not_claim_an_uninterrupted_contract(self):
        report = verifier.analyze([frame(0.,'observed'),frame(.3,'held',15.,3)])
        self.assertEqual(report['max_missed_movement_px'],15.)
        self.assertFalse(report['limit_violations'])


if __name__ == '__main__':
    unittest.main()
