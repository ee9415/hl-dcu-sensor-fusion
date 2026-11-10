"""Bit-exact equivalence of performance rewrites against the original code."""
import importlib
import sys
import types
import unittest
import warnings
from unittest.mock import MagicMock, patch

import numpy as np

from image_boundaries import nanmedian_columns


def import_node():
    """가짜 depthai 모듈로 yolopv2_node를 새로 import 한다."""
    class Host:
        def createOutput(self, **kwargs):
            return MagicMock()
    dai = types.SimpleNamespace(node=types.SimpleNamespace(HostNode=Host, ThreadedHostNode=Host),
                                Node=MagicMock(), DatatypeEnum=MagicMock(), ImgFrame=MagicMock(),
                                LengthUnit=MagicMock())
    dai.NNData = type('NNData', (), {})
    dai.ImgDetection = type('ImgDetection', (), {})
    with patch.dict(sys.modules, {'depthai': dai}):
        sys.modules.pop('yolopv2_node', None)
        return importlib.import_module('yolopv2_node')


def legacy_draw_text(font, frame, text, x, y, color, scale=2):
    # Verbatim per-cell loop from before the glyph-mask cache.
    """캐시 도입 전 원래 글자 그리기 구현(비교 기준)."""
    height, width = frame.shape[:2]
    cursor_x = max(x, 0)
    for character in text:
        glyph = font.get(character, font["?"])
        for row_index, row in enumerate(glyph):
            for column_index, selected in enumerate(row):
                if selected != "1":
                    continue
                left = cursor_x + column_index * scale
                top = y + row_index * scale
                if left >= width or top >= height:
                    continue
                right = min(left + scale, width)
                bottom = min(top + scale, height)
                if right > 0 and bottom > 0:
                    frame[max(top, 0) : bottom, max(left, 0) : right] = color
        cursor_x += 6 * scale


class NanmedianColumnsTests(unittest.TestCase):
    def same(self, values):
        """nanmedian_columns 결과가 np.nanmedian과 dtype·비트 단위까지 같은지 확인한다."""
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            expected = np.nanmedian(values, axis=0)
        actual = nanmedian_columns(values)
        self.assertEqual(actual.dtype, expected.dtype)
        # Bitwise, including NaN positions and signed zero.
        self.assertEqual(actual.tobytes(), expected.tobytes())

    def test_random_nan_patterns_are_bitwise_identical(self):
        """임의의 NaN 패턴에서도 결과가 비트 단위로 같다."""
        rng = np.random.default_rng(20261027)
        for rows in (1, 2, 3, 4, 5, 8):
            for _ in range(300):
                values = rng.uniform(0, 320, (rows, 53))
                if rng.random() < .5:
                    values = np.round(values*rng.choice([1, 2, 4]))/rng.choice([1, 2, 4])
                values[rng.random(values.shape) < rng.uniform(0, .8)] = np.nan
                self.same(values)

    def test_ties_halves_and_all_nan_columns(self):
        """동률, 짝수 개 평균, 부호 있는 0, 전부 NaN인 열 등 경계 사례."""
        self.same(np.array([[1., np.nan, 5., np.nan], [1., 2., 6., np.nan], [3., 2., np.nan, np.nan]]))
        self.same(np.array([[0.1, 0.3], [0.2, 0.7]]))
        self.same(np.array([[-0., 0.], [0., -0.]]))
        self.same(np.full((3, 4), np.nan))

    def test_regional_detector_shape(self):
        # Rows are segments sampled on the 53-row grid, as in _detect_regions.
        """실제 _detect_regions 입력과 비슷한 모양(53행 격자 세그먼트)에서도 같다."""
        rng = np.random.default_rng(7)
        for _ in range(500):
            aligned = np.full((int(rng.integers(2, 4)), 53), np.nan)
            for row in aligned:
                start = int(rng.integers(0, 50))
                stop = int(rng.integers(start+1, 54))
                row[start:stop] = 160+rng.normal(0, 1.5, stop-start).cumsum()
            active = np.isfinite(aligned).any(axis=0)
            self.same(aligned[:, active])


class DrawTextTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """yolopv2_node 모듈을 한 번만 불러온다."""
        cls.node = import_node()

    def test_matches_per_cell_loop_with_clipping(self):
        """여러 크기·위치·배율·문자열에서 새 _draw_text가 기존 구현과 픽셀 단위로 같다(화면 밖 잘림 포함)."""
        node = self.node
        draw = node.Yolopv2Postprocess._draw_text
        rng = np.random.default_rng(1)
        alphabet = ''.join(node.FONT_5X7) + '#_-:%'
        texts = ['BOUNDARIES TRACKING REF BOTH', 'L 0.85 OBSERVED R 0.41 PARTIAL',
                 'STOP UNAVAILABLE 0.00', 'Z=12.3m', 'Z=?', '', 'lowercase?']
        texts += [''.join(rng.choice(list(alphabet), int(rng.integers(1, 30)))) for _ in range(60)]
        for shape in ((540, 960, 3), (360, 640, 3), (20, 30, 3)):
            height, width = shape[:2]
            for text in texts:
                for scale in (1, 2, 3):
                    x = int(rng.integers(-40, width+10))
                    y = int(rng.integers(-30, height+10))
                    color = tuple(int(v) for v in rng.integers(0, 256, 3))
                    base = rng.integers(0, 256, shape, dtype=np.uint8)
                    expected, actual = base.copy(), base.copy()
                    legacy_draw_text(node.FONT_5X7, expected, text, x, y, color, scale)
                    draw(actual, text, x, y, color, scale)
                    self.assertEqual(actual.tobytes(), expected.tobytes(), (shape, text, x, y, scale))

    def test_edges_and_cache_is_read_only(self):
        """모서리/화면 밖 위치에서도 같고, 글리프 마스크 캐시는 읽기 전용이며 재사용된다."""
        node = self.node
        for x, y in ((0, 0), (-5, -13), (950, 530), (959, 539), (960, 0), (0, 540), (-100, -100)):
            expected = np.zeros((540, 960, 3), np.uint8)
            actual = expected.copy()
            legacy_draw_text(node.FONT_5X7, expected, 'BOUNDARIES 0.85', x, y, (0, 220, 255))
            node.Yolopv2Postprocess._draw_text(actual, 'BOUNDARIES 0.85', x, y, (0, 220, 255))
            self.assertEqual(actual.tobytes(), expected.tobytes(), (x, y))
        mask = node.Yolopv2Postprocess._glyph_mask(node.FONT_5X7['A'], 2)
        self.assertFalse(mask.flags.writeable)
        self.assertIs(mask, node.Yolopv2Postprocess._glyph_mask(node.FONT_5X7['A'], 2))


if __name__ == '__main__':
    unittest.main()
