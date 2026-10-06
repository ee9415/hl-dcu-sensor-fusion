# HL DCU Sensor Fusion

ROS2 Humble 기반 차량 센서 통합 플랫폼 저장소이다. 한라대학교 RISE 사업의
PBV 플랫폼을 대상으로 Jetson Orin NX에서 Camera, LiDAR, GNSS, IMU를 통합하고,
센서별 검증에서 통합 bringup과 sensor fusion으로 확장하는 것을 목표로 한다.

## 1. Current Scope

기준일: 2026-10-06

현재 구현은 두 경로로 나뉜다.

| 경로 | 구현 상태 | 범위 |
| --- | --- | --- |
| ROS2 카메라 bringup | 일부 구현 | OAK-D Pro PoE 1대, YOLOv6n/YOLOP VPU 추론 및 ROS2 topic 발행 |
| 6카메라 운용 도구 | 구현 및 장비 검증 기록 존재 | 비-ROS2 방식의 6대 고정 MX ID 매핑, H.265/JSONL 녹화, Web UI |

6카메라 운용 도구가 존재하더라도 6대 카메라 ROS2 bringup이 완료된 것은 아니다.
ROS2 launch, 카메라별 namespace/topic/frame, TF, calibration, RViz 통합은 아직
구현되지 않았다.

## 2. Target Environment

| 항목 | 기준 |
| --- | --- |
| OS | Ubuntu 22.04 |
| ROS | ROS2 Humble |
| Target | Jetson Orin NX / DCU |
| Language | Python, 향후 C++ 가능 |
| Build | colcon, ament_python |

## 3. Sensor Status

| Sensor | Model | Quantity | Status |
| --- | --- | ---: | --- |
| Camera | OAK-D Pro PoE | 6 | 단일 카메라 ROS2 검증 완료, 비-ROS2 6대 도구 검증 기록 존재, 6대 ROS2 통합 미완 |
| LiDAR | Livox Mid-360S | 1 | 미구현 |
| GNSS | Septentrio Mosaic-go | 1 | 미구현 |
| IMU | Xsens MTi-630 | 1 | 미구현 |

## 4. Implemented Components

### 4.1 `src/hl_camera_bringup`

- `yolov6n_node`: RGB image와 객체 검출 결과 발행
- `yolop_node`: RGB image, 차량 검출, 주행 가능 영역, 차선 mask 발행
- OAK-D VPU용 YOLOv6n/YOLOP blob 설치
- 단일 카메라 기본 주소: `192.168.200.31`

현재 두 노드는 같은 `/oak/*` topic을 사용하므로 동시에 실행하는 구성이 아니다.

### 4.2 `tools/yolop_6cam_recorder`

- `cam_01`~`cam_06` MX ID 고정 매핑
- 6대 동시 H.265 및 JSONL 녹화
- YOLOP tensor 검사와 host 후처리
- 6대 live preview, 녹화 제어, playback Web UI
- ROS2 node/topic/TF를 제공하지 않는 독립 운용 도구

## 5. Development Phases

| Phase | 목표 | 현재 상태 |
| --- | --- | --- |
| Phase 1 | 센서별 개별검증 | 진행 중: Camera 일부 완료, 나머지 센서 미구현 |
| Phase 2 | 4종 센서 통합검증 | 미시작 |
| Phase 3 | Fusion 입력 기준 확정 | 미시작 |
| Phase 4 | Fusion 구현 | 미시작 |

## 6. Documentation Authority

- 현재 구현 및 검증 상태: `PROJECT_STATUS.md`
- 실제 ROS2 topic: `TOPIC_LIST.md`
- TF 구현 여부와 후보 frame: `TF_TREE.md`
- 장비 식별자와 설정 상태: `SENSOR_CONFIGURATION.md`
- 6카메라 역할 및 배치 기준: `docs/camera_use_case.md`
- 실제 검증 결과: `docs/verification/`

`docs/sensor_alignment/`의 치수, 각도, 허용 오차는 설계안이며 실차 확정값이 아니다.
Namespace, 최종 topic, TF 부모 frame, extrinsic은 승인 및 현장 측정 전까지
확정값으로 취급하지 않는다.

## 7. Verification Summary

- 2026-06-29: 단일 OAK-D YOLOv6n ROS2 node, topic, 약 30 Hz 검증 기록
- 2026-08-12: 6카메라 Web UI 배포 및 카메라별 약 10 FPS 확인 기록
- 2026-10-06: `origin/main`의 YOLOP ROS2 node를 현재 작업 브랜치에 통합하고
  Python 정적 구문 검사를 통과함

YOLOP ROS2 node의 현 브랜치 통합 이후 `colcon build`와 실제 장비 런타임 검증은
아직 기록되지 않았다.
