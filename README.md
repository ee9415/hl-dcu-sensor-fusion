# HL DCU Sensor Fusion

ROS2 Humble 기반 차량 센서 통합 플랫폼 저장소입니다.

본 프로젝트는 한라대학교 RISE 사업의 PBV 플랫폼 적용을 목표로 하며,
Jetson Orin NX 환경에서 Camera, LiDAR, GNSS, IMU 센서를 ROS2 시스템으로
통합 운영하기 위한 소프트웨어와 문서를 관리합니다.

## 1. Project Scope

현재 저장소는 프로젝트 초기 문서화 단계입니다.

확인된 구현 상태:

- ROS2 패키지: 없음
- Launch 파일: 없음
- Driver 래퍼 코드: 없음
- RViz 설정: 없음
- TF 정의 파일: 없음
- 센서별 설정 파일: 없음

따라서 본 문서는 현재 저장소 상태를 기준으로 구조, 누락 항목, 향후 유지보수
기준을 명확히 정의하기 위한 기준 문서입니다.

## 2. Target Environment

| 항목 | 기준 |
| --- | --- |
| OS | Ubuntu 22.04 |
| ROS | ROS2 Humble |
| Target | Jetson Orin NX |
| Language | C++, Python |
| Build | colcon, ament |

## 3. Sensor Configuration

| Sensor | Model | Quantity | Status |
| --- | --- | ---: | --- |
| Camera | OAK-D Pro PoE | 6 | 문서화 필요 |
| LiDAR | Livox Mid-360S | 1 | 문서화 필요 |
| GNSS | Septentrio Mosaic-go | 1 | 문서화 필요 |
| IMU | Xsens MTi-630 | 1 | 문서화 필요 |

## 4. Repository Documents

| 문서 | 목적 | 상태 |
| --- | --- | --- |
| [AGENT.md](AGENT.md) | AI 개발자 작업 기준 | 작성됨 |
| [PROJECT_STATUS.md](PROJECT_STATUS.md) | 현재 구현 및 검증 상태 | 작성됨 |
| [CHANGELOG.md](CHANGELOG.md) | 변경 이력 | 작성됨 |
| [SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md) | 시스템 구조 | 작성됨 |
| [NETWORK.md](NETWORK.md) | 네트워크 구성 | 작성됨 |
| [TF_TREE.md](TF_TREE.md) | TF 기준 구조 | 작성됨 |
| [TOPIC_LIST.md](TOPIC_LIST.md) | Topic 목록 | 작성됨 |
| [SENSOR_CONFIGURATION.md](SENSOR_CONFIGURATION.md) | 센서 설정 | 작성됨 |
| [OPERATION_MANUAL.md](OPERATION_MANUAL.md) | 운영 절차 | 작성됨 |

## 5. Development Phases

본 프로젝트는 센서 4종을 즉시 통합하지 않고, 센서별 개별검증을 먼저 완료한 뒤
4종 통합검증과 sensor fusion으로 확장한다.

| Phase | 목표 | 결과물 |
| --- | --- | --- |
| Phase 1 | 센서별 개별검증 | 센서별 bringup, topic, Hz, TF, RViz 검증 |
| Phase 2 | 4종 센서 통합검증 | 통합 launch, namespace 충돌 확인, TF tree 단일화 |
| Phase 3 | Fusion 준비 | 입력 topic, timestamp, extrinsic, QoS 기준 확정 |
| Phase 4 | Fusion 구현 | `hl_sensor_fusion` 패키지 구현 및 검증 |

## 6. Proposed ROS2 Workspace Structure

아래 구조는 현재 구현된 구조가 아니라, 향후 유지보수를 위해 권장하는 목표
구조입니다. 실제 패키지 추가 전에는 Namespace, Topic, TF 기준을 먼저 확정해야
합니다.

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
│   ├── README.md
│   ├── architecture/
│   ├── operation/
│   └── verification/
└── src/
    ├── hl_camera_bringup/
    ├── hl_lidar_bringup/
    ├── hl_gnss_bringup/
    ├── hl_imu_bringup/
    ├── hl_sensor_bringup/
    ├── hl_sensor_description/
    ├── hl_sensor_config/
    ├── hl_sensor_diagnostics/
    └── hl_sensor_fusion/
```

구조 원칙:

- 센서별 bringup 패키지는 개별검증을 위한 독립 launch를 제공한다.
- `hl_sensor_bringup`은 센서별 launch를 include하는 통합 실행 패키지로 제한한다.
- `hl_sensor_description`은 URDF와 static TF 기준을 관리한다.
- `hl_sensor_config`는 센서별 설정, IP, calibration, 차량별 override를 관리한다.
- `hl_sensor_diagnostics`는 Hz, 연결성, node 상태 등 운영 진단을 담당한다.
- `hl_sensor_fusion`은 통합검증 이후 fusion 알고리즘과 입력 interface를 담당한다.

## 7. Verification Policy

구현 변경 후 가능한 경우 다음 항목을 확인합니다.

- `colcon build`
- Launch 실행 여부
- Node 생존 상태
- Topic publish 여부
- Topic Hz
- TF tree 연결 상태
- RViz 표시 상태

현재는 ROS2 패키지가 없으므로 빌드 및 런타임 검증은 수행할 수 없습니다.

## 8. Maintenance Rule

다음 항목은 구현 전에 기준 문서로 먼저 확정해야 합니다.

- Namespace 규칙
- Topic 이름
- TF frame 이름과 부모-자식 관계
- Driver 실행 방식
- 센서 IP 및 포트
- Launch 계층 구조

기존 차량 시스템과 호환성을 유지해야 하므로, 확정 이후에는 변경 전 영향 분석과
문서 업데이트가 필요합니다.
