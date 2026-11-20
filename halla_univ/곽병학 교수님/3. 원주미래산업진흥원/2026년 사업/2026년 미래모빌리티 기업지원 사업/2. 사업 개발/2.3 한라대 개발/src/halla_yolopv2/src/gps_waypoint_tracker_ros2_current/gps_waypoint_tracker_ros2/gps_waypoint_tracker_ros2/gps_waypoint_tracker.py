#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 + matplotlib · GPS waypoint 추종 & 조향각 퍼블리시 (정리판)

입력: (param) fix_topic  (sensor_msgs/msg/NavSatFix, default=/gps1/fix)
출력:
  /filtered_steering_angle1 (std_msgs/msg/Float32)  → LPF + "부호 반전 유지"
  /current_waypoint_index1 (std_msgs/msg/Int32)     → 현재 목표 웨이포인트 인덱스
  /heading_valid1 (std_msgs/msg/Bool)               → 1 m 이동 기반 헤딩 확정 여부

파라미터:
  csv_path     (str)   : 웨이포인트 CSV 경로
  fix_topic    (str)   : NavSatFix 토픽      (기본: /gps1/fix)
  enable_plot  (bool)  : matplotlib 표시     (기본: true)
  dt           (float) : 필터 샘플 주기 [s]  (기본: 0.05 = 50ms)
  tau          (float) : 1차 LPF 시간상수[s] (기본: 0.25)
"""

import math
import os
import threading
import time

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float32, Int32

# ====== 상수 ======
EARTH_RADIUS   = 6378137.0     # [m]
MIN_DISTANCE   = 2.8           # [m] 웨이포인트 간 최소 간격
TARGET_RADIUS  = 2.8           # [m] 목표 웨이포인트 도달 반경
ANGLE_LIMIT_DEG = 27.0         # [deg] 조향 제한
MIN_HEADING_BASELINE_M = 1.0   # [m] GPS 노이즈를 버릴 최소 진행거리
MAX_HEADING_BASELINE_AGE_SEC = 5.0  # [s] 정지 중 느린 RTK drift 제거
MAX_HEADING_ROUTE_ANGLE_DEG = 85.0  # [deg] 코스 정방향 확인 (< 90deg)
START_LOOKAHEAD_DISTANCE_M = 10.0  # [m] 코스 밖 시작 시 전방 합류 목표
GPS_STEP_FIXED_MARGIN_M = 0.45  # [m] RTK 지터/추정 여유
GPS_STEP_SPEED_MARGIN_KPH = 3.0  # [km/h] 인코더 대비 GPS 속도 여유
# GGA quality 4 (RTK Fixed) and 5 (RTK Float) are both accepted for driving.
# Keep a generous but finite covariance ceiling so a badly degraded Float
# solution cannot remain drive-ready merely because it still reports GBAS.
RTK_DRIVE_MAX_HORIZONTAL_VARIANCE_M2 = 0.09
RTK_FIXED_HOLD_SEC = 2.0
ROUTE_ALIGNMENT_MAX_CROSS_TRACK_M = 0.50
ROUTE_ALIGNMENT_MAX_TANGENT_ERROR_DEG = 5.0
ROUTE_ALIGNMENT_HOLD_SEC = 1.0
ROUTE_SEGMENT_LOOKBACK = 10
ROUTE_SEGMENT_LOOKAHEAD = 2

# ====== 전역 상태 ======
filtered_steering_angle = 0.0
current_x, current_y, current_fix_time = [], [], []
current_waypoint_index = 0
waypoint_index_initialized = False
route_completed = False
heading_vector = None
heading_valid = False
heading_fault = False
rtk_fixed = False
rtk_fixed_candidate_since = None
route_alignment_ready = False
route_alignment_since = None
route_cross_track_error_m = math.inf
vehicle_speed_kph = 0.0
last_vehicle_speed_time = None
HEADING_MIN_MOTION_SPEED_KPH = 0.3
SPEED_FEEDBACK_TIMEOUT_SEC = 0.25

# 필터 파라미터(파라미터 서버에서 로드)
DT   = 0.05   # [s]   기본 50ms
TAU  = 0.25   # [s]
ALPHA = None  # dt/(tau+dt)로 계산

# ROS 노드와 퍼블리셔 (Node 생성 이후 실제 인스턴스 생성)
ros_node = None
steering_pub = None
waypoint_index_pub = None
route_completion_pub = None
heading_valid_pub = None
rtk_fixed_pub = None
route_alignment_pub = None
route_cross_track_pub = None


# ====== 유틸 ======
def loginfo(message, *args):
    """ROS1 스타일의 서식 문자열을 ROS2 logger로 기록."""
    if args:
        message = message % args
    ros_node.get_logger().info(message)


def lat_lon_to_meters(lat, lon):
    """(deg, deg) -> (m, m) Web Mercator 근사 (소구역에서 오차 작음)"""
    x = EARTH_RADIUS * lon * math.pi / 180.0
    y = EARTH_RADIUS * math.log(math.tan((90.0 + lat) * math.pi / 360.0))
    return x, y


def distance_in_meters(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)


def navsat_fix_is_rtk_float_or_fixed(
    message,
    maximum_horizontal_variance=RTK_DRIVE_MAX_HORIZONTAL_VARIANCE_M2,
):
    """Accept this receiver's RTK Float or RTK Fixed solution.

    The NTRIP node maps both GGA quality 4 (Fixed) and 5 (Float) to
    STATUS_GBAS_FIX.  Other GPS/DGPS qualities use different NavSatStatus
    values.  Require GBAS, known finite covariance, and a bounded horizontal
    variance so an invalid or badly degraded solution remains inhibited.
    """
    try:
        variance_x = float(message.position_covariance[0])
        variance_y = float(message.position_covariance[4])
        covariance_type = int(message.position_covariance_type)
        status = int(message.status.status)
    except (AttributeError, IndexError, TypeError, ValueError):
        return False
    if status != NavSatStatus.STATUS_GBAS_FIX:
        return False
    if covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
        return False
    if not math.isfinite(variance_x) or not math.isfinite(variance_y):
        return False
    limit = max(0.0, float(maximum_horizontal_variance))
    return 0.0 <= variance_x <= limit and 0.0 <= variance_y <= limit


def nearest_route_segment_metrics(
    x,
    y,
    waypoints,
    target_index,
    lookback=ROUTE_SEGMENT_LOOKBACK,
    lookahead=ROUTE_SEGMENT_LOOKAHEAD,
):
    """Return cross-track distance and tangent near the active route index."""
    if waypoints is None or waypoints.ndim != 2 or waypoints.shape[0] != 2:
        raise ValueError("waypoints must have shape (2, N)")
    if waypoints.shape[1] < 2:
        raise ValueError("at least two waypoints are required")
    point = np.array([float(x), float(y)], dtype=float)
    final_segment = waypoints.shape[1] - 2
    first = max(0, int(target_index) - max(0, int(lookback)))
    last = min(final_segment, int(target_index) + max(0, int(lookahead)))
    if first > last:
        first = last = min(max(0, int(target_index)), final_segment)

    best_distance = math.inf
    best_tangent = None
    best_segment = None
    for segment_index in range(first, last + 1):
        start = waypoints[:, segment_index].astype(float)
        tangent = (
            waypoints[:, segment_index + 1].astype(float) - start
        )
        length_squared = float(np.dot(tangent, tangent))
        if length_squared <= 0.0:
            continue
        projection = clamp_scalar(
            float(np.dot(point - start, tangent)) / length_squared,
            0.0,
            1.0,
        )
        closest = start + projection * tangent
        distance = float(np.linalg.norm(point - closest))
        if distance < best_distance:
            best_distance = distance
            best_tangent = tangent
            best_segment = segment_index
    if best_tangent is None:
        raise ValueError("route contains no non-zero segment")
    return best_distance, best_tangent, best_segment


def clamp_scalar(value, minimum, maximum):
    return max(float(minimum), min(float(maximum), float(value)))


def select_forward_start_index(
    x,
    y,
    waypoints,
    lookahead_distance=START_LOOKAHEAD_DISTANCE_M,
):
    """Start from a larger-index waypoint far enough ahead to merge smoothly.

    The recorded course direction is increasing waypoint indices.  Selecting
    a short look-ahead target prevents a vehicle outside the course centreline
    from making a near-90-degree turn toward the closest lateral point.
    """
    if waypoints is None or waypoints.ndim != 2 or waypoints.shape[0] != 2:
        raise ValueError("waypoints must have shape (2, N)")
    if waypoints.shape[1] == 0:
        raise ValueError("waypoints must not be empty")

    distances_squared = (
        (waypoints[0] - float(x)) ** 2
        + (waypoints[1] - float(y)) ** 2
    )
    nearest_index = int(np.argmin(distances_squared))
    final_index = waypoints.shape[1] - 1
    forward_index = final_index
    required_distance = max(0.0, float(lookahead_distance))
    for index in range(nearest_index + 1, waypoints.shape[1]):
        if distance_in_meters(
            x,
            y,
            waypoints[0][index],
            waypoints[1][index],
        ) >= required_distance:
            forward_index = index
            break
    if nearest_index == final_index:
        forward_index = final_index
    return nearest_index, forward_index


def final_waypoint_reached(x, y, waypoints, waypoint_index, radius):
    """Return True only inside the final waypoint's arrival radius."""
    if waypoints is None or waypoints.ndim != 2 or waypoints.shape[0] != 2:
        raise ValueError("waypoints must have shape (2, N)")
    if waypoints.shape[1] == 0:
        raise ValueError("waypoints must not be empty")

    final_index = waypoints.shape[1] - 1
    if int(waypoint_index) != final_index:
        return False
    return distance_in_meters(
        x,
        y,
        waypoints[0][final_index],
        waypoints[1][final_index],
    ) < float(radius)


def _pick_column(df_cols, candidates):
    """df 컬럼 목록(df_cols)에서 candidates 중 첫 번째로 일치하는 이름을 반환 (소문자 매칭). 없으면 None."""
    for c in candidates:
        if c in df_cols:
            return c
    return None


def build_reduced_waypoints(csv_path):
    """
    CSV 컬럼 자동 인식:
      - 위/경도(도): (latitude|lat), (longitude|lon)
      - 동/북   (m): (east|easting|x), (north|northing|y)
    어떤 모드를 썼는지 INFO 로깅.
    """
    # 경로 정규화 + 확장
    csv_path = os.path.expanduser(csv_path)

    # 구분자/BOM 자동 처리
    df = pd.read_csv(csv_path, encoding='utf-8-sig', engine='python', sep=None)
    # 컬럼 전처리: 소문자 + 좌우공백 제거
    df.columns = [str(c).strip().lower() for c in df.columns]

    cols = set(df.columns)
    # 후보군 정의
    lat_cands  = ['latitude', 'lat']
    lon_cands  = ['longitude', 'lon']
    east_cands = ['east', 'easting', 'x']
    north_cands= ['north', 'northing', 'y']

    lat_col  = _pick_column(cols, lat_cands)
    lon_col  = _pick_column(cols, lon_cands)
    east_col = _pick_column(cols, east_cands)
    north_col= _pick_column(cols, north_cands)

    if lat_col and lon_col:
        # 위/경도(도) → m 변환
        lat = pd.to_numeric(df[lat_col], errors='coerce').to_numpy()
        lon = pd.to_numeric(df[lon_col], errors='coerce').to_numpy()
        pts = [lat_lon_to_meters(a, b) for a, b in zip(lat, lon)]
        wx, wy = zip(*pts)
        loginfo("Waypoints from lat/lon columns: (%s, %s)", lat_col, lon_col)
    elif east_col and north_col:
        # 이미 m 좌표
        wx = pd.to_numeric(df[east_col],  errors='coerce').to_numpy()
        wy = pd.to_numeric(df[north_col], errors='coerce').to_numpy()
        loginfo("Waypoints from east/north columns: (%s, %s)", east_col, north_col)
    else:
        # 사용 가능한 컬럼 보여주고 실패
        available = ", ".join(sorted(cols))
        raise ValueError(
            "CSV에서 사용할 좌표 컬럼을 찾지 못했습니다.\n"
            f"- 허용되는 조합 1) lat/lon  2) east/north (또는 x/y)\n"
            f"- 현재 컬럼: {available}"
        )

    # 간격 축소(>= MIN_DISTANCE)
    reduced_x = [float(wx[0])]
    reduced_y = [float(wy[0])]
    for x, y in zip(wx[1:], wy[1:]):
        if distance_in_meters(reduced_x[-1], reduced_y[-1], float(x), float(y)) >= MIN_DISTANCE:
            reduced_x.append(float(x))
            reduced_y.append(float(y))

    return np.array([reduced_x, reduced_y])  # shape: (2, N)


def signed_angle_deg(v1, v2):
    """두 벡터 사이 부호 있는 각도(도). 반시계 양수, 시계 음수."""
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)
    if n1 == 0 or n2 == 0:
        return 0.0
    dot = float(np.dot(v1, v2)) / (n1 * n2)
    dot = max(min(dot, 1.0), -1.0)
    ang = math.degrees(math.acos(dot))
    cross = v1[0]*v2[1] - v1[1]*v2[0]
    return ang if cross >= 0 else -ang


def rolling_heading_vector(
    xs,
    ys,
    timestamps,
    minimum_distance,
    maximum_age_sec,
):
    """Return the current chord from the nearest sample at least N metres back.

    Consecutive RTK fixes can move a few centimetres while the vehicle is
    stationary. A rolling one-metre chord rejects that noise while updating on
    every GPS callback once the vehicle is moving.
    """
    minimum_distance = float(minimum_distance)
    maximum_age_sec = float(maximum_age_sec)
    if minimum_distance <= 0.0:
        raise ValueError("minimum_distance must be positive")
    if maximum_age_sec <= 0.0:
        raise ValueError("maximum_age_sec must be positive")
    if len(xs) != len(ys) or len(xs) != len(timestamps):
        raise ValueError("x/y/time histories must have equal length")
    if len(xs) < 2:
        return None

    current = np.array([float(xs[-1]), float(ys[-1])], dtype=float)
    for index in range(len(xs) - 2, -1, -1):
        age_sec = float(timestamps[-1]) - float(timestamps[index])
        if age_sec > maximum_age_sec:
            break
        displacement = current - np.array(
            [float(xs[index]), float(ys[index])],
            dtype=float,
        )
        if float(np.linalg.norm(displacement)) >= minimum_distance:
            return displacement
    return None


def gps_step_is_plausible(
    step_distance_m,
    elapsed_sec,
    feedback_speed_kph,
    fixed_margin_m=GPS_STEP_FIXED_MARGIN_M,
    speed_margin_kph=GPS_STEP_SPEED_MARGIN_KPH,
):
    """Check a GPS step against encoder-confirmed vehicle motion.

    A single RTK solution jump previously looked like more than one metre of
    reverse motion and latched the heading safety fault.  The fixed allowance
    covers normal RTK jitter, while the distance allowance grows with encoder
    speed and callback interval.  Continuous bad fixes are not accepted, so
    the steering heartbeat naturally times out and the CAN bridge stops.
    """
    values = (
        step_distance_m,
        elapsed_sec,
        feedback_speed_kph,
        fixed_margin_m,
        speed_margin_kph,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return False
    if elapsed_sec <= 0.0 or step_distance_m < 0.0:
        return False
    allowed_speed_mps = (
        abs(float(feedback_speed_kph)) + max(0.0, float(speed_margin_kph))
    ) / 3.6
    maximum_step_m = max(0.0, float(fixed_margin_m)) + (
        allowed_speed_mps * float(elapsed_sec)
    )
    return float(step_distance_m) <= maximum_step_m


# === 시간상수 기반 LPF (첫 번째 코드 방식) + 부호 반전 유지 ===
def lpf_and_invert(current_angle_deg):
    """
    1차 저역필터:
      y[k] = (1-ALPHA)*y[k-1] + ALPHA*x[k],  where ALPHA = DT/(TAU+DT)
    이후 '부호 반전 유지'를 적용해 반환.
    """
    global filtered_steering_angle, ALPHA
    filtered_steering_angle = (1.0 - ALPHA) * filtered_steering_angle + ALPHA * current_angle_deg
    return -filtered_steering_angle  # 부호 반전 유지


# ====== 콜백 ======
reduced_waypoints = None  # main에서 로드


def vehicle_speed_callback(msg: Float32):
    """Keep heading estimation tied to encoder-confirmed motion."""
    global vehicle_speed_kph, last_vehicle_speed_time
    speed = float(msg.data)
    if not math.isfinite(speed):
        return
    vehicle_speed_kph = speed
    last_vehicle_speed_time = time.monotonic()


def gps_callback(msg: NavSatFix):
    global current_waypoint_index, reduced_waypoints
    global waypoint_index_initialized, route_completed
    global filtered_steering_angle
    global heading_vector, heading_valid, heading_fault
    global rtk_fixed, rtk_fixed_candidate_since
    global route_alignment_ready, route_alignment_since
    global route_cross_track_error_m
    global vehicle_speed_kph, last_vehicle_speed_time

    if (
        msg.status.status < 0
        or not math.isfinite(msg.latitude)
        or not math.isfinite(msg.longitude)
        or not -90.0 <= msg.latitude <= 90.0
        or not -180.0 <= msg.longitude <= 180.0
    ):
        heading_vector = None
        heading_valid = False
        rtk_fixed = False
        rtk_fixed_candidate_since = None
        route_alignment_ready = False
        route_alignment_since = None
        route_cross_track_error_m = math.inf
        filtered_steering_angle = 0.0
        current_x.clear()
        current_y.clear()
        current_fix_time.clear()
        heading_message = Bool()
        heading_message.data = False
        heading_valid_pub.publish(heading_message)
        fixed_message = Bool()
        fixed_message.data = False
        rtk_fixed_pub.publish(fixed_message)
        alignment_message = Bool()
        alignment_message.data = False
        route_alignment_pub.publish(alignment_message)
        ros_node.get_logger().warning(
            "Invalid NavSatFix; heading invalidated and drive command will time out"
        )
        return

    now = time.monotonic()
    raw_rtk_ready = navsat_fix_is_rtk_float_or_fixed(msg)
    if raw_rtk_ready:
        if rtk_fixed_candidate_since is None:
            rtk_fixed_candidate_since = now
        new_rtk_fixed = (
            now - rtk_fixed_candidate_since >= RTK_FIXED_HOLD_SEC
        )
    else:
        rtk_fixed_candidate_since = None
        new_rtk_fixed = False
    if new_rtk_fixed != rtk_fixed:
        ros_node.get_logger().info(
            f"RTK Float/Fixed gate changed: {new_rtk_fixed} "
            f"(covariance xx={float(msg.position_covariance[0]):.6f} "
            f"yy={float(msg.position_covariance[4]):.6f})"
        )
    rtk_fixed = new_rtk_fixed
    fixed_message = Bool()
    fixed_message.data = bool(rtk_fixed)
    rtk_fixed_pub.publish(fixed_message)

    # Do not issue even a zero-steer ACTIVE heartbeat unless an RTK Float or
    # Fixed solution has remained valid for the hold time.  The historical
    # topic name /rtk_fixed1 is retained for launch compatibility.
    if not rtk_fixed:
        heading_vector = None
        heading_valid = False
        route_alignment_ready = False
        route_alignment_since = None
        route_cross_track_error_m = math.inf
        filtered_steering_angle = 0.0
        current_x.clear()
        current_y.clear()
        current_fix_time.clear()
        heading_message = Bool()
        heading_message.data = False
        heading_valid_pub.publish(heading_message)
        alignment_message = Bool()
        alignment_message.data = False
        route_alignment_pub.publish(alignment_message)
        return

    # 현재 위치(m) 누적.  Reject an impossible single-fix jump
    # while the encoder confirms motion.  One rejected 10 Hz fix does not age
    # the steering heartbeat past 0.2 s; persistent bad fixes still stop it.
    x, y = lat_lon_to_meters(msg.latitude, msg.longitude)
    now = time.monotonic()
    speed_feedback_fresh = (
        last_vehicle_speed_time is not None
        and now - last_vehicle_speed_time <= SPEED_FEEDBACK_TIMEOUT_SEC
    )
    vehicle_moving = (
        speed_feedback_fresh
        and abs(vehicle_speed_kph) >= HEADING_MIN_MOTION_SPEED_KPH
    )
    if vehicle_moving and current_x and current_y and current_fix_time:
        step_distance_m = distance_in_meters(
            current_x[-1],
            current_y[-1],
            x,
            y,
        )
        step_elapsed_sec = now - current_fix_time[-1]
        if not gps_step_is_plausible(
            step_distance_m,
            step_elapsed_sec,
            vehicle_speed_kph,
        ):
            ros_node.get_logger().warning(
                f"GPS step rejected: {step_distance_m:.2f}m in "
                f"{step_elapsed_sec:.2f}s at encoder "
                f"{vehicle_speed_kph:.2f}km/h"
            )
            # Revoke high-speed promotion immediately.  Do not leave the
            # previous alignment sample valid until the bridge watchdog ages
            # out when this fix was rejected as physically impossible.
            route_alignment_ready = False
            route_alignment_since = None
            route_cross_track_error_m = math.inf
            alignment_message = Bool()
            alignment_message.data = False
            route_alignment_pub.publish(alignment_message)
            cross_track_message = Float32()
            cross_track_message.data = math.inf
            route_cross_track_pub.publish(cross_track_message)
            return
    current_x.append(x)
    current_y.append(y)
    current_fix_time.append(now)

    # A vehicle may start in the middle of the recorded course.  Initialize
    # exactly once from the closest point, then target the next (larger) index
    # so the route always proceeds in its recorded direction.
    if not waypoint_index_initialized:
        nearest_index, current_waypoint_index = select_forward_start_index(
            x,
            y,
            reduced_waypoints,
        )
        waypoint_index_initialized = True
        nearest_distance = distance_in_meters(
            x,
            y,
            reduced_waypoints[0][nearest_index],
            reduced_waypoints[1][nearest_index],
        )
        loginfo(
            "Mid-course start: nearest waypoint=%d (%.2fm), "
            "forward target=%d",
            nearest_index,
            nearest_distance,
            current_waypoint_index,
        )

    # 현재 목표 웨이포인트
    tx = reduced_waypoints[0][current_waypoint_index]
    ty = reduced_waypoints[1][current_waypoint_index]

    # 도달 판정
    if distance_in_meters(x, y, tx, ty) < TARGET_RADIUS:
        if current_waypoint_index < reduced_waypoints.shape[1] - 1:
            current_waypoint_index += 1

    # Publish continuously so late-starting volatile subscribers also receive
    # the current index. The publisher remains TRANSIENT_LOCAL as well.
    waypoint_index_msg = Int32()
    waypoint_index_msg.data = current_waypoint_index
    waypoint_index_pub.publish(waypoint_index_msg)

    # Completion is latched only after the vehicle actually enters the final
    # waypoint radius.  Merely switching the target index to the final point
    # is not enough, because that happens one segment before the course end.
    if not route_completed and final_waypoint_reached(
        x,
        y,
        reduced_waypoints,
        current_waypoint_index,
        TARGET_RADIUS,
    ):
        route_completed = True
        loginfo(
            "Route completed at final waypoint=%d; requesting drive stop",
            current_waypoint_index,
        )

    completion_message = Bool()
    completion_message.data = bool(route_completed)
    route_completion_pub.publish(completion_message)

    # Do not generate any newer steering command after completion.  The CAN
    # bridge independently gates drive on the latched completion topic.
    if route_completed:
        return

    # Use the newly selected target immediately after advancing the index.
    tx = reduced_waypoints[0][current_waypoint_index]
    ty = reduced_waypoints[1][current_waypoint_index]

    (
        route_cross_track_error_m,
        route_tangent_vector,
        route_segment_index,
    ) = nearest_route_segment_metrics(
        x,
        y,
        reduced_waypoints,
        current_waypoint_index,
    )
    cross_track_message = Float32()
    cross_track_message.data = float(route_cross_track_error_m)
    route_cross_track_pub.publish(cross_track_message)

    # A rejected direction is a run-latched safety fault.  Never resume the
    # zero-steering crawl merely because the vehicle has subsequently stopped;
    # the tracker must be restarted after the cause has been checked.
    if heading_fault:
        heading_vector = None
        heading_valid = False
        route_alignment_ready = False
        route_alignment_since = None
        heading_message = Bool()
        heading_message.data = False
        heading_valid_pub.publish(heading_message)
        alignment_message = Bool()
        alignment_message.data = False
        route_alignment_pub.publish(alignment_message)
        return

    # RTK Float can wander by more than a metre while the vehicle is parked.
    # Do not turn that drift into a heading. A fresh zero-steering heartbeat
    # still lets the bridge begin its speed-limited crawl; encoder-confirmed
    # motion then starts a clean one-metre heading chord.
    now = time.monotonic()
    speed_feedback_fresh = (
        last_vehicle_speed_time is not None
        and now - last_vehicle_speed_time <= SPEED_FEEDBACK_TIMEOUT_SEC
    )
    vehicle_moving = (
        speed_feedback_fresh
        and abs(vehicle_speed_kph) >= HEADING_MIN_MOTION_SPEED_KPH
    )
    if not vehicle_moving:
        current_x[:] = [x]
        current_y[:] = [y]
        current_fix_time[:] = [now]
        heading_vector = None
        heading_valid = False
        route_alignment_ready = False
        route_alignment_since = None
        filtered_steering_angle = 0.0
        heading_message = Bool()
        heading_message.data = False
        heading_valid_pub.publish(heading_message)
        alignment_message = Bool()
        alignment_message.data = False
        route_alignment_pub.publish(alignment_message)
        steering_msg = Float32()
        steering_msg.data = 0.0
        steering_pub.publish(steering_msg)
        return

    candidate_vector = rolling_heading_vector(
        current_x,
        current_y,
        current_fix_time,
        MIN_HEADING_BASELINE_M,
        MAX_HEADING_BASELINE_AGE_SEC,
    )
    if candidate_vector is not None and not heading_fault:
        target_vector = np.array([tx - x, ty - y])
        route_angle_deg = abs(
            signed_angle_deg(candidate_vector, target_vector)
        )
        if route_angle_deg <= MAX_HEADING_ROUTE_ANGLE_DEG:
            heading_vector = candidate_vector
            loginfo(
                "Heading baseline accepted: %.2fm, route angle=%.1fdeg",
                float(np.linalg.norm(heading_vector)),
                route_angle_deg,
            )
        else:
            # Opposite/across-course motion is latched for this tracker run.
            # Do not emit a fresh zero command: the bridge heartbeat timeout
            # changes 0x314 to INACTIVE instead of continuing to crawl.
            heading_vector = None
            heading_fault = True
            filtered_steering_angle = 0.0
            ros_node.get_logger().warning(
                f"Heading rejected: route angle {route_angle_deg:.1f}deg "
                f"exceeds {MAX_HEADING_ROUTE_ANGLE_DEG:.1f}deg; "
                "steering heartbeat stopped"
            )

    heading_valid = heading_vector is not None and not heading_fault
    heading_message = Bool()
    heading_message.data = bool(heading_valid)
    heading_valid_pub.publish(heading_message)

    if heading_fault:
        route_alignment_ready = False
        route_alignment_since = None
        alignment_message = Bool()
        alignment_message.data = False
        route_alignment_pub.publish(alignment_message)
        return

    previous_alignment_ready = route_alignment_ready
    tangent_error_deg = math.inf
    if heading_valid:
        tangent_error_deg = abs(
            signed_angle_deg(heading_vector, route_tangent_vector)
        )
    alignment_sample_ok = (
        heading_valid
        and route_cross_track_error_m <= ROUTE_ALIGNMENT_MAX_CROSS_TRACK_M
        and tangent_error_deg <= ROUTE_ALIGNMENT_MAX_TANGENT_ERROR_DEG
    )
    if alignment_sample_ok:
        if route_alignment_since is None:
            route_alignment_since = now
        route_alignment_ready = (
            now - route_alignment_since >= ROUTE_ALIGNMENT_HOLD_SEC
        )
    else:
        route_alignment_since = None
        route_alignment_ready = False
    alignment_message = Bool()
    alignment_message.data = bool(route_alignment_ready)
    route_alignment_pub.publish(alignment_message)
    if route_alignment_ready and not previous_alignment_ready:
        loginfo(
            "Route aligned: segment=%d cross-track=%.2fm "
            "tangent error=%.1fdeg",
            route_segment_index,
            route_cross_track_error_m,
            tangent_error_deg,
        )

    # A fresh zero command permits only the bridge's 1 km/h crawl while the
    # vehicle establishes a reliable heading.  No GPS fix means no fresh zero,
    # so the bridge's existing command timeout still stops the vehicle.
    if not heading_valid:
        filtered_steering_angle = 0.0
        steering_msg = Float32()
        steering_msg.data = 0.0
        steering_pub.publish(steering_msg)
        return

    curr = np.array([x, y])
    tgt_vec = np.array([tx, ty]) - curr
    angle_deg = signed_angle_deg(heading_vector, tgt_vec)
    angle_deg = max(min(angle_deg, ANGLE_LIMIT_DEG), -ANGLE_LIMIT_DEG)
    smooth_inv = lpf_and_invert(angle_deg)

    loginfo("Raw=%.2f°, Filtered(Inverted)=%.2f° (dt=%.3fs, tau=%.3fs, alpha=%.3f)",
            angle_deg, smooth_inv, DT, TAU, ALPHA)
    steering_msg = Float32()
    steering_msg.data = float(smooth_inv)
    steering_pub.publish(steering_msg)


# ====== 시각화 ======
def update_plot(_):
    ax = plt.gca()
    ax.clear()

    # 웨이포인트 + 반경
    ax.scatter(reduced_waypoints[0], reduced_waypoints[1], s=10, marker='o', label='Waypoints')
    for i, (wx, wy) in enumerate(zip(reduced_waypoints[0], reduced_waypoints[1]), 1):
        ax.add_patch(plt.Circle((wx, wy), TARGET_RADIUS, fill=False, linestyle='--'))
        ax.text(wx, wy, str(i), fontsize=8, ha='center', va='center')

    # 현재 위치 & 벡터
    if current_x and current_y:
        cx, cy = current_x[-1], current_y[-1]
        ax.scatter(cx, cy, s=50, marker='x', label='Current')

        tx = reduced_waypoints[0][current_waypoint_index]
        ty = reduced_waypoints[1][current_waypoint_index]
        ax.arrow(cx, cy, tx - cx, ty - cy, head_width=0.5, head_length=0.5)

        # 이동 경로(최근 200점만)
        k = max(0, len(current_x) - 200)
        for i in range(k + 1, len(current_x)):
            ax.arrow(current_x[i-1], current_y[i-1],
                     current_x[i]-current_x[i-1], current_y[i]-current_y[i-1],
                     head_width=0.2, head_length=0.2, length_includes_head=True)

    ax.set_aspect('equal', 'box')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.grid(True)
    ax.legend(loc='best')
    ax.set_title('Filtered steering (inverted) · dt=50 ms, tau=0.25 s; limit ±30°')


# ====== ROS2 노드 ======
class GpsWaypointTracker(Node):
    def _publish_current_waypoint_index(self):
        message = Int32()
        message.data = int(current_waypoint_index)
        waypoint_index_pub.publish(message)

        completion_message = Bool()
        completion_message.data = bool(route_completed)
        route_completion_pub.publish(completion_message)

        heading_message = Bool()
        heading_message.data = bool(heading_valid)
        heading_valid_pub.publish(heading_message)

        fixed_message = Bool()
        fixed_message.data = bool(rtk_fixed)
        rtk_fixed_pub.publish(fixed_message)

        alignment_message = Bool()
        alignment_message.data = bool(route_alignment_ready)
        route_alignment_pub.publish(alignment_message)

        if math.isfinite(route_cross_track_error_m):
            cross_track_message = Float32()
            cross_track_message.data = float(route_cross_track_error_m)
            route_cross_track_pub.publish(cross_track_message)

    def __init__(self):
        super().__init__('gps_waypoint_tracker')

        global ros_node, steering_pub, waypoint_index_pub, route_completion_pub
        global heading_valid_pub, rtk_fixed_pub
        global route_alignment_pub, route_cross_track_pub
        global reduced_waypoints, current_waypoint_index
        global waypoint_index_initialized, route_completed, DT, TAU, ALPHA
        global filtered_steering_angle
        global heading_vector, heading_valid, heading_fault, rtk_fixed
        global rtk_fixed_candidate_since
        global route_alignment_ready, route_alignment_since
        global route_cross_track_error_m
        global vehicle_speed_kph, last_vehicle_speed_time

        ros_node = self

        # 파라미터
        self.declare_parameter(
            'csv_path',
            os.path.expanduser(
                '/home/jigu/catkin_ws/src/rtk_waypoint_tracker/config/'
                'raw_track_latlon_18.csv'
            ),
        )
        self.declare_parameter('fix_topic', '/gps1/fix')
        self.declare_parameter('enable_plot', True)
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('tau', 0.25)

        csv_path = self.get_parameter('csv_path').value
        fix_topic = self.get_parameter('fix_topic').value
        self.enable_plot = self.get_parameter('enable_plot').value
        DT = float(self.get_parameter('dt').value)
        TAU = float(self.get_parameter('tau').value)
        ALPHA = DT / (TAU + DT)

        loginfo(
            "LPF params: dt=%.3fs, tau=%.3fs, alpha=%.3f",
            DT,
            TAU,
            ALPHA,
        )

        # 퍼블리셔
        steering_pub = self.create_publisher(
            Float32,
            '/filtered_steering_angle1',
            10,
        )
        waypoint_index_qos = QoSProfile(depth=10)
        waypoint_index_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        waypoint_index_pub = self.create_publisher(
            Int32,
            '/current_waypoint_index1',
            waypoint_index_qos,
        )
        route_completion_pub = self.create_publisher(
            Bool,
            '/route_completed1',
            waypoint_index_qos,
        )
        heading_valid_pub = self.create_publisher(
            Bool,
            '/heading_valid1',
            waypoint_index_qos,
        )
        rtk_fixed_pub = self.create_publisher(
            Bool,
            '/rtk_fixed1',
            waypoint_index_qos,
        )
        route_alignment_pub = self.create_publisher(
            Bool,
            '/route_alignment_ready1',
            waypoint_index_qos,
        )
        route_cross_track_pub = self.create_publisher(
            Float32,
            '/route_cross_track_error1',
            waypoint_index_qos,
        )

        # 웨이포인트 구성
        reduced_waypoints = build_reduced_waypoints(csv_path)
        loginfo(
            "Waypoints loaded: %d (reduced spacing >= %.1fm)",
            reduced_waypoints.shape[1],
            MIN_DISTANCE,
        )

        # Publish waypoint 0 immediately. TRANSIENT_LOCAL lets a bridge that
        # starts later receive this initial value as its first PassIndex input.
        current_waypoint_index = 0
        waypoint_index_initialized = False
        route_completed = False
        filtered_steering_angle = 0.0
        current_x.clear()
        current_y.clear()
        current_fix_time.clear()
        heading_vector = None
        heading_valid = False
        heading_fault = False
        rtk_fixed = False
        rtk_fixed_candidate_since = None
        route_alignment_ready = False
        route_alignment_since = None
        route_cross_track_error_m = math.inf
        vehicle_speed_kph = 0.0
        last_vehicle_speed_time = None
        initial_waypoint_message = Int32()
        initial_waypoint_message.data = current_waypoint_index
        waypoint_index_pub.publish(initial_waypoint_message)
        initial_completion_message = Bool()
        initial_completion_message.data = route_completed
        route_completion_pub.publish(initial_completion_message)
        initial_heading_message = Bool()
        initial_heading_message.data = heading_valid
        heading_valid_pub.publish(initial_heading_message)
        initial_fixed_message = Bool()
        initial_fixed_message.data = rtk_fixed
        rtk_fixed_pub.publish(initial_fixed_message)
        initial_alignment_message = Bool()
        initial_alignment_message.data = route_alignment_ready
        route_alignment_pub.publish(initial_alignment_message)
        loginfo("Initial waypoint index published: %d", current_waypoint_index)
        # Keep publishing at a low rate so the value is robust to node startup
        # order even on DDS implementations that do not replay the first
        # transient-local sample to a late subscriber.
        self.waypoint_index_timer = self.create_timer(
            0.5,
            self._publish_current_waypoint_index,
        )

        # Subscribe only after waypoint data is ready, avoiding callbacks that
        # race with reduced_waypoints initialization. Encoder-confirmed motion
        # gates heading estimation so stationary RTK drift cannot establish a
        # false direction.
        self.vehicle_speed_subscription = self.create_subscription(
            Float32,
            '/aurix/feedback/speed_kph',
            vehicle_speed_callback,
            10,
        )
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            fix_topic,
            gps_callback,
            10,
        )
        loginfo("Subscribed NavSatFix: %s", fix_topic)


# ====== 메인 ======
def main(args=None):
    rclpy.init(args=args)

    node = None
    executor = None
    spin_thread = None

    try:
        node = GpsWaypointTracker()

        if node.enable_plot:
            # matplotlib GUI는 메인 스레드에서 실행하고 ROS2 콜백은
            # 별도 executor 스레드에서 계속 처리한다.
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            spin_thread = threading.Thread(
                target=executor.spin,
                daemon=True,
            )
            spin_thread.start()

            fig = plt.figure()
            ani = animation.FuncAnimation(
                fig,
                update_plot,
                interval=300,
            )
            plt.show()
        else:
            loginfo(
                "Headless mode: plotting disabled "
                "(enable_plot=false)."
            )
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if spin_thread is not None and spin_thread.is_alive():
            spin_thread.join(timeout=2.0)
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
