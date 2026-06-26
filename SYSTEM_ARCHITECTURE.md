# SYSTEM_ARCHITECTURE.md

## 1. Purpose

이 문서는 HL DCU Sensor Fusion 프로젝트의 ROS2 시스템 구조를 정의한다.

현재 저장소에는 구현된 ROS2 패키지가 없으므로, 본 문서는 목표 구조와 확정이
필요한 항목을 분리하여 기록한다.

## 2. Current Architecture

현재 확인된 구조:

```text
hl-dcu-sensor-fusion/
├── README.md
├── AGENT.md
├── PROJECT_STATUS.md
├── CHANGELOG.md
├── SYSTEM_ARCHITECTURE.md
├── NETWORK.md
├── TF_TREE.md
├── TOPIC_LIST.md
├── SENSOR_CONFIGURATION.md
├── OPERATION_MANUAL.md
└── docs/
```

ROS2 실행 아키텍처는 아직 구현되지 않았다.

## 3. Recommended Package Structure

```text
src/
├── hl_sensor_bringup/
│   ├── launch/
│   └── rviz/
├── hl_sensor_description/
│   ├── urdf/
│   └── launch/
├── hl_sensor_config/
│   ├── camera/
│   ├── lidar/
│   ├── gnss/
│   └── imu/
├── hl_camera_bringup/
├── hl_lidar_bringup/
├── hl_gnss_bringup/
└── hl_imu_bringup/
```

## 4. Rationale

| 패키지 | 목적 | 근거 |
| --- | --- | --- |
| `hl_sensor_bringup` | 전체 센서 bringup 통합 | 운영자는 단일 launch 진입점을 필요로 함 |
| `hl_sensor_description` | URDF 및 static TF 관리 | TF 기준 구조를 코드와 분리하여 관리 |
| `hl_sensor_config` | 센서별 설정 파일 관리 | 장비 교체 및 현장 설정 변경을 추적 가능하게 함 |
| `hl_*_bringup` | 센서별 launch 계층 | 개별 센서 검증과 전체 시스템 검증을 분리 |

## 5. Open Decisions

| 항목 | 상태 |
| --- | --- |
| ROS Domain ID | 미정 |
| Namespace 규칙 | 미정 |
| 차량 기준 frame | 미정 |
| 센서 frame 이름 | 미정 |
| Driver 선정 | 미정 |
| Launch parameter schema | 미정 |
