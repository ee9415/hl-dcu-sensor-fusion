"""Tracked/virtual lane midpoint in vehicle meters; confidence gates activation."""
from dataclasses import dataclass, replace
import json
import math

import numpy as np

from .protocol import FRAME_ID, validate_path


def load_config(path):
    with open(path, encoding='utf-8') as stream:
        config = json.load(stream)
    validate_config(config)
    return config


def validate_config(config):
    g, q, c = (config[name] for name in ('geometry', 'quality', 'can'))
    c.setdefault('windows_monitor_only', False)
    q.setdefault('boundary_state_weights', dict(observed=1., inferred=.65, held=.55, predicted=.4, aligning=.25))
    q.setdefault('held_confidence_tau_s', .5)
    if (q['held_confidence_tau_s'] <= 0 or not math.isfinite(q['held_confidence_tau_s']) or
            any(not math.isfinite(v) or not 0 <= v <= 1 for v in q['boundary_state_weights'].values())):
        raise ValueError('Invalid boundary state confidence weights')
    for values in (g, q, c):
        for name, value in values.items():
            if isinstance(value, (int, float)) and not math.isfinite(value):
                raise ValueError('Nonfinite setting: '+name)
    for name in ('calibrated', 'timing_verified', 'bench_test_active'):
        if type(g[name]) is not bool:
            raise ValueError('Expected boolean: '+name)
    for name in ('allow_bench_active', 'allow_vcan', 'windows_allow_virtual', 'windows_monitor_only'):
        if type(c[name]) is not bool:
            raise ValueError('Expected boolean: '+name)
    if (g['width'], g['height']) != (320, 180):
        raise ValueError('Current boundary coordinates must be 320x180')
    if not (g['fx'] > 0 and g['fy'] > 0 and g['camera_height_m'] > 0 and
            0 < g['pitch_down_deg'] < 90 and abs(g['yaw_left_deg']) < 90 and abs(g['roll_deg']) < 90):
        raise ValueError('Invalid camera geometry')
    if type(g['capture_uncertainty_ms']) is not int or not 0 <= g['capture_uncertainty_ms'] <= 2**32-1:
        raise ValueError('Invalid capture uncertainty')
    if not (0 < q['max_image_age_ms'] <= 5000 and
            type(q['reactivation_frames']) is int and q['reactivation_frames'] >= 3 and
            type(q['min_points']) is int and q['min_points'] >= 4 and
            type(q['min_confidence']) is int and 1 <= q['min_confidence'] <= 100 and
            0 < q['min_boundary_confidence'] <= 1 and
            0 < q['min_observed_span_m'] <= q['max_length_m'] <= 12 and
            0 <= q['max_near_extrapolation_m'] <= 3 and
            0 < q['min_lane_width_m'] < q['max_lane_width_m'] and
            q['max_fit_rmse_m'] > 0 and q['max_origin_fit_uncertainty_m'] > 0 and
            q['max_lane_width_std_m'] > 0):
        raise ValueError('Invalid path quality configuration')
    if not (0 <= c['min_speed_kph'] <= c['max_speed_kph'] <= 3 and
            c['startup_quiet_ms'] > 150 and c['startup_inactive_ms'] >= 150 and
            0 < c['max_tx_age_ms'] < 150 and isinstance(c['interface'], str) and c['interface']):
        raise ValueError('Invalid CAN configuration')
    if (c['driver_backend'] not in ('windows_tcp', 'socketcan') or
            (c['windows_monitor_only'] and c['driver_backend'] != 'windows_tcp') or
            not isinstance(c['windows_host'], str) or not c['windows_host'] or
            type(c['windows_port']) is not int or not 1 <= c['windows_port'] <= 65535 or
            not isinstance(c['windows_key_file'], str)):
        raise ValueError('Invalid Windows CAN configuration')


@dataclass(frozen=True)
class Target:
    capture_s: float = 0.
    source_id: str = ''
    frame_sequence: int = 0
    coefficients: tuple = (0., 0., 0., 0.)
    valid_length_m: float = 0.
    observed_start_m: float = 0.
    fit_rmse_m: float = 0.
    confidence: int = 0
    valid: bool = False
    calibrated: bool = False
    timing_verified: bool = False
    bench_only: bool = False
    capture_uncertainty_ms: int = 0
    frame_id: str = FRAME_ID
    reason: str = 'no_input'


def ground_points(points, geometry):
    uv = np.asarray(points, dtype=float)
    if (uv.ndim != 2 or uv.shape[1] != 2 or not np.isfinite(uv).all() or
            np.any(uv < 0) or np.any(uv[:, 0] >= geometry['width']) or
            np.any(uv[:, 1] >= geometry['height'])):
        raise ValueError('invalid_observed_pixels')
    pitch, yaw, roll = np.deg2rad([geometry['pitch_down_deg'], geometry['yaw_left_deg'], geometry['roll_deg']])
    horizontal = (uv[:, 0]-geometry['cx'])/geometry['fx']
    vertical = (uv[:, 1]-geometry['cy'])/geometry['fy']
    roll_y = -math.cos(roll)*horizontal+math.sin(roll)*vertical
    roll_z = -math.sin(roll)*horizontal-math.cos(roll)*vertical
    forward = math.cos(pitch)+math.sin(pitch)*roll_z
    left = roll_y
    down = math.sin(pitch)-math.cos(pitch)*roll_z
    if np.any(down <= 1e-6):
        raise ValueError('observations_at_or_above_horizon')
    scale = geometry['camera_height_m']/down
    x = geometry['camera_forward_m']+scale*(math.cos(yaw)*forward-math.sin(yaw)*left)
    y = geometry['camera_left_m']+scale*(math.sin(yaw)*forward+math.cos(yaw)*left)
    xy = np.column_stack((x, y))
    return xy[np.argsort(x)]


def target_from_result(result, config):
    g, q = config['geometry'], config['quality']
    try:
        stamp, source = result['stamp'], result['source']
        if (type(stamp['sec']) is not int or type(stamp['nanosec']) is not int or
                not 0 <= stamp['nanosec'] < 10**9 or type(result['frame_seq']) is not int or
                not 0 <= result['frame_seq'] < 2**64 or not isinstance(result['session'], str) or
                not result['session']):
            raise ValueError('invalid_frame_metadata')
        capture = stamp['sec']+stamp['nanosec']/1e9
        base = Target(capture_s=capture,
            source_id=json.dumps([result['session'], source.get('input_source'),
                source.get('source_epoch'), source.get('source_media_epoch'), source.get('processing_epoch')]),
            frame_sequence=int(result['frame_seq']), calibrated=g['calibrated'],
            timing_verified=g['timing_verified'],
            bench_only=not (g['calibrated'] and g['timing_verified']),
            capture_uncertainty_ms=g['capture_uncertainty_ms'])
        if not math.isfinite(capture) or capture <= 0:
            raise ValueError('invalid_capture_time')
    except (ValueError, TypeError, KeyError, OverflowError):
        return Target(reason='invalid_frame_metadata')
    try:
        if result.get('stale') or not result.get('boundary_pair_valid'):
            raise ValueError('no_current_lane_pair')
        if result.get('geometry_units') != 'image_pixels_320x180':
            raise ValueError('unsupported_pixel_coordinate_system')
        if not (g['calibrated'] and g['timing_verified']) and not g['bench_test_active']:
            raise ValueError('calibration_or_capture_timing_unverified')
        if source.get('input_source') != 'camera' and not g['bench_test_active']:
            raise ValueError('pc_input_is_bench_only')
        if source.get('input_source') != 'camera':
            base = replace(base, bench_only=True)
        if result.get('continuity_error'):
            raise ValueError('frame_continuity_error')
        states = [result.get(side+'_boundary_state') for side in ('left', 'right')]
        scores = [result[side+'_boundary_confidence'] for side in ('left', 'right')]
        if not all(math.isfinite(s) and 0 <= s <= 1 for s in scores):
            raise ValueError('boundary_confidence')
        diagnostics = result.get('boundary_detection_diagnostics') or []
        weighted = []
        for i, (state, score) in enumerate(zip(states, scores)):
            weight = q['boundary_state_weights'].get(state, 0.)
            if state in ('held', 'predicted') and i < len(diagnostics):
                age = diagnostics[i].get('missing_s')
                if age is not None:
                    if not math.isfinite(age) or age < 0:raise ValueError('invalid_boundary_age')
                    weight *= math.exp(-age/q['held_confidence_tau_s'])
            weighted.append(score*weight)
        evidence_confidence = sum(weighted)/2
        base = replace(base, confidence=math.floor(100*evidence_confidence))
        if not evidence_confidence:
            raise ValueError('no_boundary_evidence')
        # Reuse the tracker's complete modeled boundaries, including virtual
        # sides. Their states affect confidence instead of categorical rejection.
        points = [result.get(side+'_boundary_image', result.get(side+'_boundary_supported_image', []))
                  for side in ('left', 'right')]
        visible = []
        for samples in points:
            pixels = np.asarray(samples, dtype=float)
            if pixels.ndim != 2 or pixels.shape[1] != 2 or not np.isfinite(pixels).all():
                raise ValueError('invalid_boundary_pixels')
            # Tracked curves can extend outside the image; use their in-frame
            # portion rather than clamping coordinates or rejecting the whole curve.
            inside = ((pixels[:, 0] >= 0) & (pixels[:, 0] < g['width']) &
                      (pixels[:, 1] >= 0) & (pixels[:, 1] < g['height']))
            visible.append(pixels[inside])
        points = visible
        if any(len(p) < q['min_points'] for p in points):
            raise ValueError('insufficient_observed_points')
        left, right = (ground_points(p, g) for p in points)
        if any(np.any(np.diff(p[:, 0]) <= 1e-6) for p in (left, right)):
            raise ValueError('nonmonotonic_ground_projection')
        near = max(0., left[0, 0], right[0, 0])
        far = min(q['max_length_m'], left[-1, 0], right[-1, 0])
        if near > q['max_near_extrapolation_m'] or far-near < q['min_observed_span_m']:
            raise ValueError('observed_range_or_near_extrapolation')
        x = np.linspace(near, far, max(q['min_points'], 32))
        yl, yr = (np.interp(x, p[:, 0], p[:, 1]) for p in (left, right))
        width = yl-yr
        if (np.any(width < q['min_lane_width_m']) or np.any(width > q['max_lane_width_m']) or
                np.std(width) > q['max_lane_width_std_m']):
            raise ValueError('lane_width')
        center = (yl+yr)/2
        design = np.polynomial.polynomial.polyvander(x, 3)
        coeff, _, rank, _ = np.linalg.lstsq(design, center, rcond=None)
        if rank != 4 or np.linalg.cond(design) > 1e6:
            raise ValueError('ill_conditioned_fit')
        rmse = float(np.sqrt(np.mean((design@coeff-center)**2)))
        # This is a development quality score, not a calibrated probability.
        origin_uncertainty = max(rmse, .01)*float(np.linalg.norm(np.linalg.pinv(design)[0]))
        if rmse > q['max_fit_rmse_m'] or origin_uncertainty > q['max_origin_fit_uncertainty_m']:
            raise ValueError('fit_or_extrapolation_error')
        confidence = math.floor(100*evidence_confidence*math.exp(-rmse/q['max_fit_rmse_m'])*
            (1-.25*near/max(q['max_near_extrapolation_m'], .001)))
        base = replace(base, confidence=confidence)
        if confidence < q['min_confidence']:
            raise ValueError('path_confidence')
        validate_path(tuple(coeff), far, confidence)
        return replace(base, coefficients=tuple(float(c) for c in coeff), valid_length_m=float(far),
            observed_start_m=float(near), fit_rmse_m=rmse, confidence=confidence,
            valid=True, reason=('bench_only:' if base.bench_only else '')+'lane_midpoint:'+ '/'.join(states))
    except (ValueError, TypeError, KeyError, OverflowError, np.linalg.LinAlgError) as exc:
        return replace(base, reason=str(exc))
