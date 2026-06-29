# 5. Software Calibration 적용 방법

## 5.1 기계 정렬과 Software Calibration의 역할 분담

| 구분 | 담당 | 허용 오차 | 특성 |
|------|------|---------|------|
| 기계 정렬 | 브라켓 조정 | 수mm / 수도 단위 | 반복성 제공, 재보정 최소화 |
| Intrinsic Calibration | 소프트웨어 | 픽셀 단위 | 렌즈 왜곡·주점 보정 |
| Extrinsic Calibration | 소프트웨어 | 수mm / 0.1~0.5° | 센서 간 상대 위치 정밀 보정 |

기계 정렬이 목표 범위 내로 들어오면 Software Calibration이 나머지 오차를 흡수한다.
기계 오차가 Software 보정 한계를 초과하면 재장착이 필요하다.

## 5.2 기계 정렬 목표 허용 오차

아래 범위 내로 기계 정렬을 맞추는 것을 목표로 한다.
이 범위를 만족하면 Software Calibration이 나머지를 정밀하게 보정 가능하다.

| 자유도 | 기계 정렬 목표 | Software 보정 가능 범위 |
|--------|--------------|----------------------|
| Pitch  | ±1.0°        | ±10° (정밀도 저하 없음) |
| Yaw    | ±1.0°        | ±10° (정밀도 저하 없음) |
| Roll   | ±0.5°        | ±5°                  |
| X (전후) | ±5 mm     | 제한 없음 (단순 이동)   |
| Y (좌우) | ±5 mm     | 제한 없음 (단순 이동)   |
| Z (상하) | ±5 mm     | 제한 없음 (단순 이동)   |

**기계 정렬 목표를 ±1° 이내로 설정하는 이유:**
- 10 m 거리에서 Yaw 1°는 약 175 mm 투영 오차이므로 기계 정렬 단계에서 줄여야 함
- Software 보정 가능 범위는 넓지만, 오차가 클수록 Calibration 재현성 저하

## 5.3 Extrinsic Calibration 파라미터

### 5.3.1 파라미터 의미

```
T_sensor_mount_to_cam = [tx, ty, tz, roll, pitch, yaw]

tx  : sensor_mount_link → 카메라 X축 이동 (m)
ty  : sensor_mount_link → 카메라 Y축 이동 (m)
tz  : sensor_mount_link → 카메라 Z축 이동 (m)
roll  : X축 회전 (rad)
pitch : Y축 회전 (rad)
yaw   : Z축 회전 (rad)
```

### 5.3.2 초기 추정값 (수작업 측정)

Calibration 전, 줄자·캘리퍼스로 측정한 초기값을 파라미터에 입력한다.
이 초기값이 정확할수록 Calibration 알고리즘 수렴이 빠르다.

```yaml
# calibration/tf/initial_guess.yaml
cam_front_center_near:
  tx: 1.250   # 차량 전방 1.25 m
  ty: 0.000
  tz: 0.820   # 지면에서 0.82 m
  roll: 0.0
  pitch: 0.0
  yaw: 0.0
```

## 5.4 ROS2에서의 적용 방법

### 5.4.1 Static Transform Publisher 방식

가장 단순한 방법. URDF 또는 launch 파일에 직접 기입한다.

```python
# launch/sensor_tf.launch.py
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='tf_cam_front_near',
    arguments=[
        '1.250', '0.000', '0.820',   # tx ty tz
        '0.000', '0.000', '0.000',   # roll pitch yaw (rad)
        'sensor_mount_link',
        'cam_front_center_near'
    ]
)
```

### 5.4.2 YAML 파라미터 로드 방식 (권장)

Calibration 결과를 YAML 파일로 저장하고 launch 시 로드한다.
파라미터 변경 시 코드 수정 없이 YAML만 수정하면 된다.

```python
# launch/sensor_tf.launch.py
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile

Node(
    package='sensor_fusion',
    executable='extrinsic_tf_publisher',
    parameters=[
        ParameterFile('calibration/tf/sensor_extrinsic.yaml')
    ]
)
```

### 5.4.3 동적 업데이트 방식 (실험적)

운용 중 Calibration 결과 업데이트가 필요한 경우:

```bash
ros2 param set /extrinsic_tf_publisher cam_front_center_near.yaw 0.017
```

단, 운용 중 TF 변경은 다운스트림 알고리즘에 영향을 주므로 주의가 필요하다.

## 5.5 Reprojection Error 기반 품질 평가

### 5.5.1 평가 방법

Calibration 완료 후 LiDAR 포인트를 카메라 이미지에 투영하여 오차를 측정한다.

```bash
ros2 run sensor_fusion reprojection_eval \
  --lidar_topic /livox/lidar \
  --camera_topic /cam_front_center_near/image_raw \
  --camera_info /cam_front_center_near/camera_info \
  --extrinsic calibration/tf/sensor_extrinsic.yaml
```

### 5.5.2 품질 기준

| 등급 | Reprojection RMSE | 판정 |
|------|-------------------|------|
| A | < 1.0 pixel | 우수 |
| B | 1.0 ~ 2.0 pixel | 양호 |
| C | 2.0 ~ 5.0 pixel | 재Calibration 권장 |
| D | > 5.0 pixel | 기계 정렬 재확인 필요 |

## 5.6 Calibration 주기 권장

| 상황 | 권장 조치 |
|------|---------|
| 센서 교체 후 | 즉시 재Calibration |
| 브라켓 탈착·재장착 후 | Reprojection Error 확인 → C 이상이면 재Calibration |
| 주행 거리 500 km 이상 | 주기적 검증 |
| 충격 또는 사고 후 | 즉시 재Calibration |
| 계절 변화 (30°C 이상 온도차) | 열팽창 영향 확인 |

## 5.7 Calibration 결과 관리

```
calibration/
├── intrinsic/
│   ├── cam_front_center_near_intrinsic.yaml
│   ├── cam_front_center_far_intrinsic.yaml
│   └── ...
├── tf/
│   ├── sensor_extrinsic.yaml          ← 현재 사용 중인 값
│   ├── sensor_extrinsic_20260101.yaml ← 날짜별 백업
│   └── initial_guess.yaml
└── logs/
    ├── calibration_log_20260101.md
    └── reprojection_eval_20260101.csv
```

날짜별 백업을 유지하여 Calibration 이력을 추적한다.
이상 발생 시 이전 값으로 롤백 가능하게 한다.
