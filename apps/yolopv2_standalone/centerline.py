from __future__ import annotations

from dataclasses import dataclass

import numpy as np


BEV_SIZE = (640, 480)
FORWARD_RANGE_M = 40.0
LATERAL_RANGE_M = 12.0


@dataclass(frozen=True)
class CenterlineEstimate:
    valid: bool
    coefficients: tuple[float, float, float, float]
    residual_m: float
    forward_range_m: tuple[float, float]
    bev_curve: np.ndarray
    image_curve: np.ndarray
    bev_left: np.ndarray
    bev_right: np.ndarray


class CenterlineTemporalFilter:
    def __init__(
        self,
        *,
        alpha: float = 0.25,
        max_curve_jump_px: float = 75.0,
        hold_frames: int = 3,
    ) -> None:
        self.alpha = alpha
        self.max_curve_jump_px = max_curve_jump_px
        self.hold_frames = hold_frames
        self._estimate: CenterlineEstimate | None = None
        self._rejected_frames = 0

    def update(self, measured: CenterlineEstimate) -> CenterlineEstimate:
        previous = self._estimate
        if not measured.valid:
            self._rejected_frames += 1
            if previous is not None and self._rejected_frames <= self.hold_frames:
                return previous
            self._estimate = None
            return measured

        if previous is None or not previous.valid:
            self._estimate = measured
            self._rejected_frames = 0
            return measured

        curve_jump = float(
            np.percentile(
                np.abs(measured.bev_curve[:, 0] - previous.bev_curve[:, 0]),
                75.0,
            )
        )
        if curve_jump > self.max_curve_jump_px:
            self._rejected_frames += 1
            if self._rejected_frames <= self.hold_frames:
                return previous
            self._estimate = measured
            self._rejected_frames = 0
            return measured

        keep = 1.0 - self.alpha
        smoothed = CenterlineEstimate(
            valid=True,
            coefficients=tuple(
                keep * old + self.alpha * new
                for old, new in zip(
                    previous.coefficients,
                    measured.coefficients,
                    strict=True,
                )
            ),
            residual_m=measured.residual_m,
            forward_range_m=measured.forward_range_m,
            bev_curve=(
                keep * previous.bev_curve + self.alpha * measured.bev_curve
            ).astype(np.float32),
            image_curve=(
                keep * previous.image_curve + self.alpha * measured.image_curve
            ).astype(np.float32),
            bev_left=measured.bev_left,
            bev_right=measured.bev_right,
        )
        self._estimate = smoothed
        self._rejected_frames = 0
        return smoothed


def _homography(source: np.ndarray, destination: np.ndarray) -> np.ndarray:
    rows = []
    values = []
    for (x, y), (u, v) in zip(source, destination, strict=True):
        rows.append((x, y, 1.0, 0.0, 0.0, 0.0, -u * x, -u * y))
        values.append(u)
        rows.append((0.0, 0.0, 0.0, x, y, 1.0, -v * x, -v * y))
        values.append(v)
    solution = np.linalg.solve(
        np.asarray(rows, dtype=np.float64),
        np.asarray(values, dtype=np.float64),
    )
    return np.append(solution, 1.0).reshape(3, 3)


def _transform(points: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    if not len(points):
        return np.empty((0, 2), dtype=np.float64)
    homogeneous = np.column_stack(
        (np.asarray(points, dtype=np.float64), np.ones(len(points)))
    )
    transformed = homogeneous @ matrix.T
    divisor = transformed[:, 2:3]
    divisor[np.abs(divisor) < 1e-9] = np.nan
    return transformed[:, :2] / divisor


def _road_segment(row: np.ndarray, reference_x: float) -> tuple[int, int] | None:
    selected = np.flatnonzero(row != 0)
    if not len(selected):
        return None
    breaks = np.flatnonzero(np.diff(selected) > 1) + 1
    segments = np.split(selected, breaks)
    candidates = [segment for segment in segments if len(segment) >= 6]
    if not candidates:
        return None

    containing = [
        segment
        for segment in candidates
        if int(segment[0]) <= reference_x <= int(segment[-1])
    ]
    if containing:
        chosen = max(containing, key=len)
    else:
        chosen = min(
            candidates,
            key=lambda segment: abs(
                (float(segment[0]) + float(segment[-1])) * 0.5 - reference_x
            ),
        )
        chosen_center = (float(chosen[0]) + float(chosen[-1])) * 0.5
        if abs(chosen_center - reference_x) > len(row) * 0.18:
            return None
    return int(chosen[0]), int(chosen[-1])


def _invalid_estimate() -> CenterlineEstimate:
    empty = np.empty((0, 2), dtype=np.float32)
    return CenterlineEstimate(
        valid=False,
        coefficients=(float("nan"),) * 4,
        residual_m=float("nan"),
        forward_range_m=(0.0, 0.0),
        bev_curve=empty,
        image_curve=empty,
        bev_left=empty,
        bev_right=empty,
    )


def estimate_centerline(
    drivable_mask: np.ndarray,
    *,
    bev_size: tuple[int, int] = BEV_SIZE,
    forward_range_m: float = FORWARD_RANGE_M,
    lateral_range_m: float = LATERAL_RANGE_M,
) -> CenterlineEstimate:
    """Fit lateral(forward) = c0 + c1*x + c2*x^2 + c3*x^3."""
    mask = np.asarray(drivable_mask)
    if mask.ndim != 2 or mask.size == 0:
        raise ValueError(f"Drivable mask must be a non-empty 2-D array, got {mask.shape}")

    source_height, source_width = mask.shape
    bev_width, bev_height = bev_size
    source_quad = np.asarray(
        (
            (0.04 * (source_width - 1), 0.98 * (source_height - 1)),
            (0.96 * (source_width - 1), 0.98 * (source_height - 1)),
            (0.62 * (source_width - 1), 0.50 * (source_height - 1)),
            (0.38 * (source_width - 1), 0.50 * (source_height - 1)),
        ),
        dtype=np.float64,
    )
    destination_quad = np.asarray(
        (
            (0.0, bev_height - 1.0),
            (bev_width - 1.0, bev_height - 1.0),
            (bev_width - 1.0, 0.0),
            (0.0, 0.0),
        ),
        dtype=np.float64,
    )
    image_to_bev = _homography(source_quad, destination_quad)
    bev_to_image = np.linalg.inv(image_to_bev)

    left_points = []
    right_points = []
    center_points = []
    reference_x = (source_width - 1) * 0.5
    started = False
    missing_rows = 0
    first_row = int(round(0.50 * (source_height - 1)))
    last_row = int(round(0.98 * (source_height - 1)))
    for row_index in range(last_row, first_row - 1, -1):
        segment = _road_segment(mask[row_index], reference_x)
        if segment is None:
            if started:
                missing_rows += 1
                if missing_rows > 6:
                    break
            continue
        left, right = segment
        if not started and not (
            left <= source_width * 0.60 and right >= source_width * 0.40
        ):
            continue
        started = True
        missing_rows = 0
        center = (left + right) * 0.5
        reference_x = center
        left_points.append((left, row_index))
        right_points.append((right, row_index))
        center_points.append((center, row_index))

    if len(center_points) < 12:
        return _invalid_estimate()

    bev_left = _transform(np.asarray(left_points), image_to_bev)
    bev_right = _transform(np.asarray(right_points), image_to_bev)
    bev_center = _transform(np.asarray(center_points), image_to_bev)
    finite = np.all(np.isfinite(bev_center), axis=1)
    inside = (
        (bev_center[:, 1] >= 0.0)
        & (bev_center[:, 1] <= bev_height - 1.0)
    )
    selected = finite & inside
    bev_left = bev_left[selected]
    bev_right = bev_right[selected]
    bev_center = bev_center[selected]
    if len(bev_center) < 12:
        return _invalid_estimate()

    forward_m = (1.0 - bev_center[:, 1] / (bev_height - 1.0)) * forward_range_m
    lateral_m = (
        bev_center[:, 0] / (bev_width - 1.0) - 0.5
    ) * lateral_range_m
    order = np.argsort(forward_m)
    forward_m = forward_m[order]
    lateral_m = lateral_m[order]
    if float(np.ptp(forward_m)) < 5.0:
        return _invalid_estimate()

    polynomial = np.polynomial.Polynomial.fit(
        forward_m,
        lateral_m,
        3,
        domain=(0.0, forward_range_m),
    ).convert()
    coefficients = np.pad(polynomial.coef, (0, max(0, 4 - len(polynomial.coef))))[:4]
    fitted = polynomial(forward_m)
    residual_m = float(np.sqrt(np.mean((lateral_m - fitted) ** 2)))
    valid = bool(np.all(np.isfinite(coefficients)) and residual_m <= 1.5)
    if not valid:
        return _invalid_estimate()

    near_m = max(0.0, float(np.min(forward_m)))
    far_m = min(forward_range_m, float(np.max(forward_m)))
    samples_m = np.linspace(near_m, far_m, 80)
    curve_lateral_m = polynomial(samples_m)
    curve_x = (
        curve_lateral_m / lateral_range_m + 0.5
    ) * (bev_width - 1.0)
    curve_y = (1.0 - samples_m / forward_range_m) * (bev_height - 1.0)
    bev_curve = np.column_stack((curve_x, curve_y))
    image_curve = _transform(bev_curve, bev_to_image)

    return CenterlineEstimate(
        valid=True,
        coefficients=tuple(float(value) for value in coefficients),
        residual_m=residual_m,
        forward_range_m=(near_m, far_m),
        bev_curve=bev_curve.astype(np.float32),
        image_curve=image_curve.astype(np.float32),
        bev_left=bev_left.astype(np.float32),
        bev_right=bev_right.astype(np.float32),
    )


def _interpolated_boundary(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    rounded_y = np.rint(points[:, 1]).astype(np.int32)
    unique_y = np.unique(rounded_y)
    x_values = np.asarray(
        [np.median(points[rounded_y == y, 0]) for y in unique_y],
        dtype=np.float64,
    )
    return unique_y, x_values


def _draw_polyline(
    frame: np.ndarray,
    points: np.ndarray,
    color: tuple[int, int, int],
    thickness: int = 3,
) -> None:
    height, width = frame.shape[:2]
    sampled_x = []
    sampled_y = []
    for start, end in zip(points[:-1], points[1:], strict=True):
        distance = int(max(abs(end[0] - start[0]), abs(end[1] - start[1]))) + 1
        sampled_x.append(np.linspace(start[0], end[0], distance))
        sampled_y.append(np.linspace(start[1], end[1], distance))
    if not sampled_x:
        return

    base_x = np.rint(np.concatenate(sampled_x)).astype(np.int32)
    base_y = np.rint(np.concatenate(sampled_y)).astype(np.int32)
    offsets = np.asarray(
        [
            (offset_x, offset_y)
            for offset_y in range(-thickness, thickness + 1)
            for offset_x in range(-thickness, thickness + 1)
            if offset_x * offset_x + offset_y * offset_y <= thickness * thickness
        ],
        dtype=np.int32,
    )
    draw_x = (base_x[:, None] + offsets[None, :, 0]).ravel()
    draw_y = (base_y[:, None] + offsets[None, :, 1]).ravel()
    inside = (
        (draw_x >= 0)
        & (draw_x < width)
        & (draw_y >= 0)
        & (draw_y < height)
    )
    frame[draw_y[inside], draw_x[inside]] = color


def render_bev(estimate: CenterlineEstimate) -> np.ndarray:
    width, height = BEV_SIZE
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:] = (14, 22, 30)
    for forward_m in range(0, int(FORWARD_RANGE_M) + 1, 5):
        y = int(round((1.0 - forward_m / FORWARD_RANGE_M) * (height - 1)))
        frame[max(y - 1, 0) : min(y + 1, height)] = (43, 58, 68)
    for lateral_m in range(-6, 7):
        x = int(round((lateral_m / LATERAL_RANGE_M + 0.5) * (width - 1)))
        frame[:, max(x - 1, 0) : min(x + 1, width)] = (43, 58, 68)

    if not estimate.valid:
        return frame

    left_y, left_x = _interpolated_boundary(estimate.bev_left)
    right_y, right_x = _interpolated_boundary(estimate.bev_right)
    first_y = max(int(left_y.min()), int(right_y.min()), 0)
    last_y = min(int(left_y.max()), int(right_y.max()), height - 1)
    rows = np.arange(first_y, last_y + 1)
    left = np.interp(rows, left_y, left_x)
    right = np.interp(rows, right_y, right_x)
    start = np.clip(np.minimum(left, right), 0, width - 1)[:, None]
    end = np.clip(np.maximum(left, right), 0, width - 1)[:, None]
    columns = np.arange(width)[None, :]
    road = (columns >= start) & (columns <= end)
    road_region = frame[rows]
    road_region[road] = (24, 82, 46)
    frame[rows] = road_region

    _draw_polyline(frame, estimate.bev_left, (0, 180, 255), thickness=2)
    _draw_polyline(frame, estimate.bev_right, (0, 180, 255), thickness=2)
    _draw_polyline(frame, estimate.bev_curve, (255, 255, 0), thickness=4)
    return frame


def draw_centerline_overlay(
    frame: np.ndarray,
    estimate: CenterlineEstimate,
    source_mask_size: tuple[int, int],
) -> None:
    if not estimate.valid:
        return
    mask_width, mask_height = source_mask_size
    frame_height, frame_width = frame.shape[:2]
    points = estimate.image_curve.copy().astype(np.float64)
    points[:, 0] *= frame_width / mask_width
    points[:, 1] *= frame_height / mask_height
    _draw_polyline(frame, points, (255, 255, 0), thickness=4)
