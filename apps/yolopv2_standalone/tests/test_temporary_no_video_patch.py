import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[3]
PATH = ROOT/'src/hl_camera_bringup/scripts/temporary_no_video_patch.py'
SPEC = importlib.util.spec_from_file_location('temporary_no_video_patch', PATH)
patch = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(patch)


class NoVideoDiagnosticPatchTests(unittest.TestCase):
    def test_preserves_processing_and_removes_video(self):
        main = patch.patch_main((ROOT/'apps/yolopv2_standalone/main.py').read_text(encoding='utf-8'))
        node = patch.patch_node((ROOT/'apps/yolopv2_standalone/yolopv2_node.py').read_text(encoding='utf-8'))
        for name in ('self._boundary_tracker.update(', 'segmentation_masks(', 'self.drivable.send(',
                     'self.lane.send(', 'self._telemetry.publish(', 'self._stop_line_tracker.update('):
            self.assertIn(name, node)
        for name in ('self.result.send(', 'self.bev.send(', 'render_boundary_bev(boundaries)'):
            self.assertNotIn(name, node)
        self.assertIn('postprocess.model_size = MODEL_SIZE', main)
        self.assertNotIn('StreamOutput).build(', main)
        compile(main, 'main.py', 'exec')
        compile(node, 'yolopv2_node.py', 'exec')

    def test_missing_marker_fails_closed(self):
        with self.assertRaises(ValueError):
            patch.patch_main('unrelated content')
        with self.assertRaises(ValueError):
            patch.patch_node('unrelated content')


if __name__ == '__main__':
    unittest.main()
