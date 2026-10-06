# SYSTEM_ARCHITECTURE.md

## 1. Purpose

이 문서는 현재 구현된 실행 경로와 목표 ROS2 통합 구조를 구분한다.

기준일: 2026-10-06

## 2. Current Architecture

```text
OAK-D Pro PoE 1대
  └─ hl_camera_bringup (ROS2)
       ├─ yolov6n_node
       │    ├─ /oak/rgb/image_raw
       │    └─ /oak/detections
       └─ yolop_node
            ├─ /oak/rgb/image_raw
            ├─ /oak/detections
            ├─ /oak/drivable_area
            └─ /oak/lane_line

OAK-D Pro PoE 6대
  └─ tools/yolop_6cam_recorder (비-ROS2)
       ├─ 6대 MX ID 고정 매핑
       ├─ H.265 + JSONL 녹화
       ├─ 선택적 YOLOP host 후처리
       └─ Web UI preview / record / playback
```

두 ROS2 노드는 동일 카메라와 동일 topic을 사용하므로 대체 실행 관계이다.
6카메라 도구는 ROS graph, TF 또는 ROS message를 제공하지 않는다.

## 3. Repository Structure

```text
src/
└── hl_camera_bringup/
    ├── blobs/
    ├── hl_camera_bringup/
    │   ├── yolov6n_node.py
    │   └── yolop_node.py
    ├── scripts/
    ├── package.xml
    └── setup.py

tools/
└── yolop_6cam_recorder/
    ├── yolop_6cam_recorder.py
    ├── yolop_6cam_web_ui.py
    ├── web_ui/
    └── 실행/관리 shell script
```

## 4. Target Architecture

```text
Sensor Drivers / Bringup
  camera | lidar | gnss | imu
             │
             ▼
Common Configuration + TF/URDF
             │
             ▼
Integrated Bringup + Diagnostics + RViz
             │
             ▼
Frozen Fusion Interface
  topic | timestamp | QoS | calibration
             │
             ▼
Sensor Fusion
```

권장 패키지 책임:

| 패키지 | 책임 | 상태 |
| --- | --- | --- |
| `hl_camera_bringup` | OAK-D 개별검증과 camera ROS2 실행 | 일부 구현 |
| `hl_lidar_bringup` | Livox 개별검증 | 미구현 |
| `hl_gnss_bringup` | Septentrio 개별검증 | 미구현 |
| `hl_imu_bringup` | Xsens 개별검증 | 미구현 |
| `hl_sensor_bringup` | 개별 launch를 포함하는 통합 실행 | 미구현 |
| `hl_sensor_description` | URDF와 static TF | 미구현 |
| `hl_sensor_config` | 공통/차량별 config와 calibration | 미구현 |
| `hl_sensor_diagnostics` | node/topic/Hz/TF 상태 | 미구현 |
| `hl_sensor_fusion` | 확정된 입력 interface 기반 fusion | 미구현 |

## 5. Six-Camera Identity

역할과 장착 방향의 정본은 `docs/camera_use_case.md`이다.

| Tool ID | Logical Role | Proposed Optical Frame |
| --- | --- | --- |
| `cam_01` | 전방 중앙 | `cam01_front_center_optical_frame` |
| `cam_02` | 좌측 전방 장착, 좌측 후면 방향 | `cam02_front_left_rear_view_optical_frame` |
| `cam_03` | 우측 전방 장착, 우측 후면 방향 | `cam03_front_right_rear_view_optical_frame` |
| `cam_04` | 좌측 후방 장착, 좌측 전면 방향 | `cam04_rear_left_front_view_optical_frame` |
| `cam_05` | 우측 후방 장착, 우측 전면 방향 | `cam05_rear_right_front_view_optical_frame` |
| `cam_06` | 후방 중앙 | `cam06_rear_center_optical_frame` |

위 frame은 후보 이름이다. TF publisher와 extrinsic이 없으므로 실제 TF로 확정된
상태가 아니다.

## 6. Launch Responsibility

| 단계 | 책임 |
| --- | --- |
| 개별검증 | 센서별 driver/node, config, 선택적 RViz |
| 카메라 6대 | 카메라별 namespace, parameter, device 식별, lifecycle/오류 처리 |
| 통합검증 | 4종 launch include, 공통 TF, diagnostics, RViz |
| Fusion | 입력 remap, fusion node, output diagnostics |

## 7. Open Decisions

- ROS Domain ID
- `base_link` 또는 `sensor_mount_link` 등 최종 TF 부모 frame
- 6카메라 namespace/topic/frame 규칙
- 카메라별 IP와 MX ID 선택 방식
- timestamp와 시간 동기화 정책
- QoS와 backpressure 정책
- calibration 저장/배포 형식
- LiDAR/GNSS/IMU driver 및 고정 버전
