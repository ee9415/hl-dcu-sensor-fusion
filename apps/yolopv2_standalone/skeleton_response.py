"""Live response tuning without changing skeleton identity or history."""
from dataclasses import replace
import math


def validate_speed(speed):
    if type(speed) not in (int, float) or not math.isfinite(speed) or not .5 <= speed <= 3:
        raise ValueError('스켈레톤 반응 속도는 0.5~3.0배로 입력해 주세요')
    return float(speed)


def response_config(base, speed):
    speed = validate_speed(speed)
    return replace(base,
        skeleton_observed_tau_s=base.skeleton_observed_tau_s/speed,
        skeleton_observed_fine_tau_s=base.skeleton_observed_fine_tau_s/speed,
        skeleton_observed_max_rate_px_s=base.skeleton_observed_max_rate_px_s*speed,
        skeleton_retained_tau_s=base.skeleton_retained_tau_s/speed,
        skeleton_retained_max_rate_px_s=base.skeleton_retained_max_rate_px_s*speed)
