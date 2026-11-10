"""temporary_no_video_patch(영상 출력을 제거한 진단용 빌드 패치 스크립트) 테스트."""
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
        """패치 후에도 인지 처리·텔레메트리 호출은 남고, 결과/BEV 영상 출력 경로만 제거되며 코드가 컴파일된다."""
        main = patch.patch_main((ROOT/'apps/yolopv2_standalone/main.py').read_text(encoding='utf-8'))
        node = patch.patch_node((ROOT/'apps/yolopv2_standalone/yolopv2_node.py').read_text(encoding='utf-8'))
        for name in ('self._boundary_tracker.update_contours(', 'segmentation_masks(', 'self.drivable.send(',
                     'self.lane.send(', 'self._telemetry.publish(', 'self._stop_line_tracker.update(',
                     'self._guidance.update('):
            self.assertIn(name, node)
        for name in ('self.result.send(', 'self.bev.send(', 'render_boundary_bev(boundaries)'):
            self.assertNotIn(name, node)
        self.assertNotIn('self._top_view.render(', node)
        self.assertIn('postprocess.model_size = MODEL_SIZE', main)
        self.assertNotIn('StreamOutput).build(', main)
        compile(main, 'main.py', 'exec')
        compile(node, 'yolopv2_node.py', 'exec')

    def test_missing_marker_fails_closed(self):
        """패치 기준 문자열을 찾지 못하면 조용히 넘어가지 않고 오류를 낸다."""
        with self.assertRaises(ValueError):
            patch.patch_main('unrelated content')
        with self.assertRaises(ValueError):
            patch.patch_node('unrelated content')


if __name__ == '__main__':
    unittest.main()
