# 4. Calibration 절차

## 4.1 절차 개요

```
① 브라켓 장착 및 Datum 정렬
        │
        ▼
② 수작업 기계 정렬 (Pitch / Yaw / Roll)
        │
        ▼
③ Intrinsic Calibration (카메라 내부 파라미터)
        │
        ▼
④ Extrinsic Calibration (센서 간 상대 위치)
   - Camera ↔ Camera
   - Camera ↔ LiDAR
        │
        ▼
⑤ TF 파라미터 반영
        │
        ▼
⑥ 브라켓 최종 고정 (규정 토크 체결)
        │
        ▼
⑦ 검증 (Reprojection Error 확인)
```

## 4.2 사전 준비

### 4.2.1 필요 장비

| 항목 | 용도 |
|------|------|
| Checkerboard (800×600 mm, 8×6, 80 mm) | Intrinsic / Extrinsic |
| AprilTag 보드 (A2 크기 이상) | LiDAR-Camera Extrinsic |
| 수평계 (0.1° 분해능) | Pitch/Roll 수작업 정렬 |
| 레이저 포인터 또는 조준선 | Yaw 수작업 정렬 |
| 토크 드라이버 | 최종 체결 |
| 줄자 / 캘리퍼스 | 기계 치수 확인 |

### 4.2.2 환경 조건

- 실내 또는 바람 없는 환경 (Checkerboard 흔들림 방지)
- 균일 조명 (그림자 없는 확산광 권장)
- 차량 정지 상태 (엔진 OFF 권장)

## 4.3 Step 1 — 브라켓 장착 및 Datum 정렬

1. 브라켓 베이스 플레이트를 차량 구조물 기준면(Datum)에 밀착시킨다.
2. Dowel Pin을 삽입하여 위치를 결정한다.
3. 체결 볼트를 손으로만 임시 체결한다 (조정 여지 유지).
4. 수평계로 브라켓 베이스 수평 여부를 확인한다.

## 4.4 Step 2 — 수작업 기계 정렬

목표는 Extrinsic Calibration 결과가 Software 허용 범위(→ 05_software_calibration.md) 이내로
들어오도록 물리적으로 센서를 조정하는 것이다.

### Pitch 정렬

1. 카메라를 켜고 영상을 실시간으로 확인한다.
2. 지평선이 영상 수평 중앙에 오도록 Pitch를 조정한다.
3. 수평계를 카메라 상면에 올려 각도를 확인한다.
4. 목표 Pitch 각도(카메라별 초기값 참조)로 맞춘다.

### Yaw 정렬

1. 차량 전방(후방) 중앙에 레이저 포인터 또는 기준 테이프 라인을 설치한다.
2. 카메라 영상에서 기준선이 영상 수직 중앙에 오도록 Yaw를 조정한다.
3. 영상 내 격자(Grid Overlay) 사용 권장.

### Roll 정렬

1. 영상 내 수직 직선 구조물(벽, 기둥)을 기준으로 확인한다.
2. 기울어짐이 0.5° 미만이 되도록 Roll을 미세 조정한다.

## 4.5 Step 3 — Intrinsic Calibration

카메라별 내부 파라미터(초점거리, 주점, 왜곡 계수)를 구한다.
OAK-D Pro PoE는 팩토리 Calibration 값이 내장되어 있으나,
현장 재Calibration을 통해 더 정확한 값을 얻을 수 있다.

### ROS2 기반 수행 방법

```bash
# camera_calibration 패키지 사용
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.080 \
  image:=/cam_front_center_near/image_raw \
  camera:=/cam_front_center_near
```

- Checkerboard를 다양한 위치·거리·각도에서 50장 이상 촬영
- RMS Reprojection Error 목표: < 0.5 pixel

### 저장

```bash
# 결과 파일 저장 위치
calibration/intrinsic/<camera_id>_intrinsic.yaml
```

## 4.6 Step 4 — Extrinsic Calibration

### 4.6.1 Camera ↔ Camera

인접 카메라 간 상대 변환(Translation + Rotation)을 구한다.

```bash
# kalibr 사용 (권장)
kalibr_calibrate_cameras \
  --bag calibration.bag \
  --topics /cam_front_center_near/image_raw /cam_front_center_far/image_raw \
  --models pinhole-radtan pinhole-radtan \
  --target april_6x6.yaml
```

또는 OpenCV stereo calibration:
```bash
ros2 run sensor_fusion_calibration stereo_extrinsic \
  --camera_a cam_front_center_near \
  --camera_b cam_front_center_far
```

### 4.6.2 Camera ↔ LiDAR

```bash
# lidar_camera_calibration 패키지 사용
ros2 launch lidar_camera_calibration calibration.launch.py \
  lidar_topic:=/livox/lidar \
  camera_topic:=/cam_front_center_near/image_raw \
  camera_info_topic:=/cam_front_center_near/camera_info
```

- AprilTag 보드를 LiDAR 시야와 카메라 시야가 겹치는 위치에 배치
- 최소 5가지 이상 자세에서 데이터 수집

### 4.6.3 결과 해석

| 항목 | 허용 기준 |
|------|---------|
| Translation 오차 | < 5 mm |
| Rotation 오차 | < 0.5° |
| Reprojection RMSE | < 2.0 pixel |

## 4.7 Step 5 — TF 파라미터 반영

Calibration 결과를 ROS2 TF로 반영한다.

### 파라미터 파일 구조

```yaml
# calibration/tf/sensor_extrinsic.yaml

sensor_mount_link:
  cam_front_center_near:
    x: 1.250      # 단위: m
    y: 0.000
    z: 0.820
    roll:  0.000  # 단위: rad
    pitch: 0.000
    yaw:   0.000

  cam_front_center_far:
    x: 1.250
    y: 0.000
    z: 0.920
    roll:  0.000
    pitch: -0.087  # -5° in rad
    yaw:   0.000

  lidar_top:
    x: 0.000
    y: 0.000
    z: 1.500
    roll:  0.000
    pitch: 0.000
    yaw:   0.000
```

### Launch 파일 반영

```python
# launch/sensor_tf.launch.py
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['1.25', '0', '0.82', '0', '0', '0',
                       'sensor_mount_link', 'cam_front_center_near']
        ),
        # ... 나머지 센서
    ])
```

## 4.8 Step 6 — 최종 고정

TF 파라미터 반영 후 검증이 완료되면 브라켓을 최종 고정한다.

1. 각 볼트를 규정 토크로 체결한다 (→ 02_mechanical_design.md 4절 참조).
2. 필요 시 Loctite 243을 나사부에 도포 후 체결한다.
3. 체결 완료 후 다시 한번 센서 각도를 수평계로 검증한다.

## 4.9 Step 7 — 검증

최종 고정 후 결과를 검증한다.

```bash
# RViz2로 카메라-LiDAR 정합 확인
ros2 launch sensor_fusion visualization.launch.py
```

확인 항목:
- LiDAR 포인트가 카메라 영상 내 객체에 정확히 투영되는지
- 인접 카메라 간 오버랩 영역에서 객체 위치 일치 여부
- Reprojection Error가 허용 범위 내인지

## 4.10 절차 개선안 (원래 절차 대비)

원래 제안된 절차는 Intrinsic Calibration 단계가 빠져 있었다.
개선된 절차는 다음과 같다.

| 단계 | 원래 절차 | 개선 절차 |
|------|---------|---------|
| 1 | 브라켓 장착 | 브라켓 장착 |
| 2 | 수작업 정렬 | 수작업 정렬 |
| 3 | Checkerboard 촬영 | **Intrinsic Calibration** |
| 4 | Extrinsic 계산 | Extrinsic 계산 (Intrinsic 기반) |
| 5 | TF 반영 | TF 반영 |
| 6 | 최종 고정 | **검증 후** 최종 고정 |
| 7 | — | **Reprojection 검증** |

핵심 개선: Intrinsic 먼저 → Extrinsic 계산 시 정확도 향상.
최종 고정 전 검증 추가 → 불량 상태 고정 방지.
