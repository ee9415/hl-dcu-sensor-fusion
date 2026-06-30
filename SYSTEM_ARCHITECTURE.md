# SYSTEM_ARCHITECTURE.md

## 1. Purpose

이 문서는 HL DCU Sensor Fusion 프로젝트의 ROS2 시스템 구조를 정의한다.

현재 저장소에는 `hl_camera_bringup` 패키지가 구현되어 있으므로, 본 문서는
현재 구현된 카메라 개별검증 구조와 향후 목표 구조를 분리하여 기록한다.

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
├── docs/
└── src/
    └── hl_camera_bringup/
        ├── hl_camera_bringup/
        │   └── yolov6n_node.py
        ├── scripts/
        │   ├── yolov6n_infer_test.py
        │   └── yolov6n_display.py
        ├── blobs/
        ├── package.xml
        ├── setup.py
        └── setup.cfg
```

현재 ROS2 실행 아키텍처는 카메라 단일 노드 수준까지 구현되었다.
통합 bringup, launch 계층, TF/URDF, RViz 설정은 아직 구현되지 않았다.

## 3. Development Architecture

본 프로젝트의 개발 흐름은 센서별 개별검증, 4종 통합검증, Fusion 준비, Fusion
구현 순서로 진행한다.

```text
Phase 1: Individual Sensor Verification
  camera / lidar / gnss / imu

Phase 2: Integrated Sensor Bringup
  camera + lidar + gnss + imu

Phase 3: Fusion Interface Freeze
  topic / TF / timestamp / QoS / calibration

Phase 4: Sensor Fusion Implementation
  fusion node / diagnostics / validation
```

이 구조는 센서 driver 문제와 통합 문제를 분리하기 위한 것이다. 개별 센서 검증이
완료되지 않은 상태에서 통합 launch를 구성하면 네트워크, driver, TF, timestamp
문제가 동시에 발생하여 원인 분석이 어려워진다.

## 4. Recommended Package Structure

```text
src/
├── hl_camera_bringup/
│   ├── launch/
│   └── rviz/
├── hl_lidar_bringup/
│   ├── launch/
│   └── rviz/
├── hl_gnss_bringup/
│   ├── launch/
│   └── rviz/
├── hl_imu_bringup/
│   ├── launch/
│   └── rviz/
├── hl_sensor_bringup/
│   ├── launch/
│   └── rviz/
├── hl_sensor_description/
│   ├── urdf/
│   └── launch/
├── hl_sensor_config/
│   ├── config/
│   │   ├── camera/
│   │   ├── lidar/
│   │   ├── gnss/
│   │   └── imu/
│   └── calibration/
├── hl_sensor_diagnostics/
└── hl_sensor_fusion/
```

## 5. Package Responsibility

| 패키지 | 목적 | 근거 |
| --- | --- | --- |
| `hl_camera_bringup` | OAK-D Pro PoE 개별검증 및 YOLOv6n VPU 추론 | 단일 카메라 검증 완료, 6대 확장 필요 |
| `hl_lidar_bringup` | Livox Mid-360S 개별검증 | point cloud 수신, frame, Hz 확인 필요 |
| `hl_gnss_bringup` | Septentrio Mosaic-go 개별검증 | 위치, 시간, 보정 데이터 확인 필요 |
| `hl_imu_bringup` | Xsens MTi-630 개별검증 | orientation, angular velocity, acceleration 확인 필요 |
| `hl_sensor_bringup` | 4종 센서 통합 bringup | 개별 launch를 include하여 통합 실행 |
| `hl_sensor_description` | URDF 및 static TF 관리 | TF 기준 구조를 코드와 분리하여 관리 |
| `hl_sensor_config` | 센서별 설정 파일 관리 | 장비 교체 및 현장 설정 변경을 추적 가능하게 함 |
| `hl_sensor_diagnostics` | 통합 상태 진단 | Hz, node, topic, TF 상태를 운영 중 확인 |
| `hl_sensor_fusion` | 센서 fusion 구현 | 통합검증 이후 알고리즘을 driver와 분리 |

## 6. Launch Responsibility

| 단계 | Launch 책임 | 포함 범위 |
| --- | --- | --- |
| 개별검증 | 센서별 bringup package | 해당 센서 driver, config, RViz |
| 통합검증 | `hl_sensor_bringup` | 4종 센서 launch include, 공통 TF, RViz |
| Fusion | `hl_sensor_fusion` | fusion node, 입력 remap, diagnostics |

현재 `hl_camera_bringup`에는 launch 파일이 없으며, `ros2 run` 기반 단일 노드 실행만
검증되어 있다. Launch 계층은 카메라 6대 확장 기준이 정리된 뒤 추가한다.

## 7. Open Decisions

| 항목 | 상태 |
| --- | --- |
| ROS Domain ID | 미정 |
| Namespace 규칙 | 미정 |
| 차량 기준 frame | 미정 |
| 센서 frame 이름 | 미정 |
| Driver 선정 | 미정 |
| Launch parameter schema | 미정 |
| Fusion input interface | 미정 |
| Time synchronization 기준 | 미정 |
