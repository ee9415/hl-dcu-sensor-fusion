"""Independent 20 Hz transmitter state machine with capture-age fail-closed logic."""
import math

from .path import Target
from .protocol import FRAME_ID, packet, validate_path


class PathGate:
    def __init__(self, config, monotonic_s):
        self.config = config
        self.started = monotonic_s
        self.target = None
        self.received = 0.
        self.age_at_receive = 0.
        self.streak = 0
        self.sequence = 0
        self.reason = 'startup_quiet'
        self.last_ros = None

    def invalidate(self, reason):
        self.target = None
        self.streak = 0
        self.reason = reason

    def _validate(self, target, ros_s):
        q, c = self.config['quality'], self.config['can']
        if not target.valid:
            raise ValueError(target.reason or 'invalid_path')
        if target.frame_id != FRAME_ID or not target.source_id or target.frame_sequence < 0:
            raise ValueError('path_frame_or_identity')
        if not (target.calibrated and target.timing_verified) and not target.bench_only:
            raise ValueError('unverified_path')
        if target.bench_only and not c['allow_bench_active']:
            raise ValueError('bench_active_disabled')
        validate_path(target.coefficients, target.valid_length_m, target.confidence)
        if not (math.isfinite(target.observed_start_m) and
                0 <= target.observed_start_m <= q['max_near_extrapolation_m'] and
                target.valid_length_m-target.observed_start_m >= q['min_observed_span_m'] and
                target.valid_length_m <= q['max_length_m'] and
                math.isfinite(target.fit_rmse_m) and 0 <= target.fit_rmse_m <= q['max_fit_rmse_m'] and
                target.confidence >= q['min_confidence']):
            raise ValueError('path_quality')
        if not (math.isfinite(ros_s) and math.isfinite(target.capture_s) and target.capture_s > 0 and
                type(target.capture_uncertainty_ms) is int and target.capture_uncertainty_ms >= 0):
            raise ValueError('invalid_capture_time')
        age = ros_s-target.capture_s
        if age < -.005:
            raise ValueError('capture_time_in_future')
        if max(0., age)*1000+target.capture_uncertainty_ms >= q['max_image_age_ms']:
            raise ValueError('capture_expired')
        return max(0., age)

    def update(self, target, ros_s, monotonic_s):
        try:
            age = self._validate(target, ros_s)
            old = self.target
            if old and old.source_id == target.source_id:
                if target.frame_sequence == old.frame_sequence:
                    return  # Repeated ROS deliveries never renew freshness or warmup.
                if target.frame_sequence < old.frame_sequence or target.capture_s <= old.capture_s:
                    raise ValueError('reordered_path')
                if (target.frame_sequence != old.frame_sequence+1 or
                        target.capture_s-old.capture_s >= self.config['quality']['max_image_age_ms']/1000):
                    self.streak = 0
            else:
                self.streak = 0
            self.target = target
            self.received, self.age_at_receive = monotonic_s, age
            self.streak += 1
            self.reason = 'warming_up'
        except (ValueError, TypeError, OverflowError) as exc:
            self.invalidate(str(exc))

    def tick(self, ros_s, monotonic_s):
        c, q = self.config['can'], self.config['quality']
        elapsed_ms = (monotonic_s-self.started)*1000
        if elapsed_ms < c['startup_quiet_ms']:
            return None
        if self.last_ros is not None and ros_s < self.last_ros-.005:
            self.invalidate('ros_clock_moved_backwards')
        self.last_ros = ros_s
        target = self.target
        active = False
        if target is not None:
            try:
                self._validate(target, ros_s)
                conservative_age = max(ros_s-target.capture_s,
                    self.age_at_receive+monotonic_s-self.received)
                if conservative_age*1000+target.capture_uncertainty_ms >= q['max_image_age_ms']:
                    raise ValueError('capture_expired')
                if elapsed_ms < c['startup_quiet_ms']+c['startup_inactive_ms']:
                    self.reason = 'startup_inactive'
                elif self.streak < q['reactivation_frames']:
                    self.reason = 'warming_up'
                else:
                    active = True
                    self.reason = 'bench_active' if target.bench_only else 'active'
            except (ValueError, TypeError, OverflowError) as exc:
                self.invalidate(str(exc))
        data = packet(self.sequence, active,
            coefficients=target.coefficients if active else (0.,)*4,
            length=target.valid_length_m if active else 0.,
            confidence=target.confidence if active else 0,
            min_speed=c['min_speed_kph'], max_speed=c['max_speed_kph'])
        self.sequence = (self.sequence+1) & 255
        return data

    def remaining_s(self, ros_s, monotonic_s):
        if self.target is None:
            return 0.
        age = max(ros_s-self.target.capture_s,
            self.age_at_receive+monotonic_s-self.received)
        return max(0., (self.config['quality']['max_image_age_ms']-
            self.target.capture_uncertainty_ms)/1000-age)
