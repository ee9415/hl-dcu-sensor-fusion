# 1. Sensor Alignment 개요

## 1.1 목적

자율주행 차량의 Sensor Fusion 정확도는 각 센서의 Extrinsic Calibration 품질에 직접 의존한다.
Extrinsic Calibration은 센서 간 상대 위치와 자세를 정의하는 과정이며,
이 값이 부정확하면 카메라-LiDAR 투영 오류, 거리 추정 오차, 객체 위치 불일치가 발생한다.

기계적 정렬(Mechanical Alignment)은 Calibration 이전 단계에서 물리적으로 센서 위치를
기준에 맞추는 행위이다. 기계 정렬이 잘 될수록 Software Calibration의 보정 범위가 줄고,
반복 조립 후에도 재Calibration 없이 운용 가능한 범위가 넓어진다.

## 1.2 설계 철학

### 1.2.1 "Mechanical First" 원칙

Software로 보정할 수 있는 오차라도 기계적으로 먼저 줄이는 것을 우선한다.
Software Calibration은 보정 범위 내에서만 유효하며, 기계 오차가 클수록
추정 불확실도와 민감도가 증가한다.

```
기계 정렬 오차 작을수록 → Calibration 반복 주기 길어짐
                         → 센서 교체 후 복구 시간 단축
                         → Fusion 알고리즘 안정성 향상
```

### 1.2.2 반복 재현성 (Repeatability)

연구용 차량은 센서 탈착이 빈번하다. 따라서 재장착 후 동일 위치가 보장되어야 한다.
이를 위해 Datum(기준면) + Dowel Pin(위치결정핀) 조합을 사용한다.

### 1.2.3 Calibration 친화적 구조

Checkerboard 또는 AprilTag 기반 Extrinsic Calibration 절차가
현장에서 신속하게 수행될 수 있도록 브라켓 구조와 차량 배치를 고려한다.

## 1.3 대상 센서 및 배치 개요

| 센서 | 수량 | 위치 | TF 링크 |
|------|------|------|---------|
| OAK-D Pro PoE (전방 근거리) | 1 | 전방 중앙 하단 | `cam_front_center_near` |
| OAK-D Pro PoE (전방 원거리) | 1 | 전방 중앙 상단 | `cam_front_center_far` |
| OAK-D Pro PoE (좌후방) | 1 | 좌측 후방 | `cam_rear_left` |
| OAK-D Pro PoE (우후방) | 1 | 우측 후방 | `cam_rear_right` |
| OAK-D Pro PoE (후방 근거리) | 1 | 후방 중앙 하단 | `cam_rear_center_near` |
| OAK-D Pro PoE (후방 원거리) | 1 | 후방 중앙 상단 | `cam_rear_center_far` |
| Livox Mid-360 | 1 | 루프 또는 후드 | `lidar_top` |
| GNSS | 1 | 루프 | `gnss_link` |
| IMU | 1 | 차체 중앙 | `imu_link` |

모든 센서 TF는 `sensor_mount_link`를 공통 기준으로 사용한다.

## 1.4 정렬 오차가 Fusion에 미치는 영향

아래는 카메라 Yaw 오차와 LiDAR 투영 오류의 관계 예시이다 (10 m 전방 물체 기준).

| Yaw 오차 | 수평 투영 오류 |
|----------|--------------|
| 0.1°     | ~17 mm       |
| 0.5°     | ~87 mm       |
| 1.0°     | ~175 mm      |
| 2.0°     | ~349 mm      |

Pitch 오차는 카메라 지면 인식 높이에 영향을 주며, 0.5° 이상이면
차선 인식 알고리즘의 원근 변환(IPM)에서 눈에 띄는 왜곡이 발생한다.

## 1.5 전체 정렬 흐름

```
[브라켓 기계 제작]
        │
        ▼
[차량 장착 및 Datum 기준 위치 결정]
        │
        ▼
[수작업 각도 조정 (Pitch / Yaw / Roll)]
        │
        ▼
[Checkerboard / AprilTag 기반 Extrinsic Calibration]
        │
        ▼
[TF 파라미터 반영 (YAML / launch 파일)]
        │
        ▼
[브라켓 최종 고정 (잠금 볼트 체결)]
        │
        ▼
[운용 중 주기적 검증]
```
