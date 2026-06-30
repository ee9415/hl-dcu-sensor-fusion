# CHANGELOG.md

이 문서는 저장소의 주요 변경 사항을 기록한다.

형식은 Keep a Changelog의 구조를 참고하되, 본 프로젝트에서는 검증 결과와
운영 영향도를 함께 기록한다.

## [Unreleased]

### Changed (2026-06-30, 문서 상태 동기화)

- `README.md`, `PROJECT_STATUS.md`, `SYSTEM_ARCHITECTURE.md`를 현재 코드 상태에 맞게 갱신
  - `hl_camera_bringup` ament_python 패키지 존재 반영
  - OAK-D Pro PoE YOLOv6n ROS2 노드 구현 및 단일 카메라 검증 상태 반영
  - launch, TF, RViz, 6대 확장은 미완으로 구분
- `SENSOR_CONFIGURATION.md`, `NETWORK.md`에 단일 카메라 검증에서 확인된 장비 정보를 반영
- `OPERATION_MANUAL.md`에 현재 실행 가능한 단일 카메라 노드 기준 명령을 추가
- Topic 기준 문서(`TOPIC_LIST.md`)는 향후 별도 작업 대상으로 두고 변경하지 않음

### Added (2026-06-29, ROS2 토픽 검증)

- `src/hl_camera_bringup/hl_camera_bringup/yolov6n_node.py` — ROS2 Python 노드
  - `/oak/rgb/image_raw` (`sensor_msgs/Image`) 발행
  - `/oak/detections` (`vision_msgs/Detection2DArray`) 발행
  - depthai 파이프라인을 별도 스레드로 실행, ROS2 spin과 분리
  - device_ip, blob_path 등 ROS2 파라미터로 설정 가능
- `src/hl_camera_bringup/package.xml`, `setup.py`, `setup.cfg` — ament_python 패키지 구성
- blob을 `share/hl_camera_bringup/blobs/`에 설치하여 `get_package_share_directory`로 접근

### Verification (2026-06-29, ROS2 토픽)

- `colcon build`: **통과**
- `ros2 run hl_camera_bringup yolov6n_node`: **정상 실행**
- `/oak/rgb/image_raw` 발행률: **29.996 Hz**
- `/oak/detections` 발행률: **29.928 Hz**
- `ros2 topic echo /oak/detections --once`: class_id, score, bbox 정상 수신

### Added (2026-06-29, 작업환경 정리)

- `AGENT.md` 섹션 2 "Working Directory" 추가
  - 모든 작업 파일은 프로젝트 루트 안에 작성하도록 명문화
  - 임시 디렉토리(`~/halla_sensor_ws` 등) 사용 금지 규칙 추가
  - Mandatory Work Order에 8단계 "커밋 및 푸시" 추가
- `~/halla_sensor_ws` 임시 작업 파일 삭제 (프로젝트에 이미 반영 완료)

### Added (2026-06-29)

- `src/` 디렉토리 생성 및 센서별 패키지 뼈대 추가
  - `src/hl_camera_bringup/scripts/` — 카메라 검증 스크립트
  - `src/hl_lidar_bringup/scripts/` — (미작성)
  - `src/hl_gnss_bringup/scripts/` — (미작성)
  - `src/hl_imu_bringup/scripts/` — (미작성)
- `src/hl_camera_bringup/scripts/yolov6n_infer_test.py` — Myriad X VPU 추론 터미널 검증 스크립트
- `src/hl_camera_bringup/scripts/yolov6n_display.py` — 바운딩박스 오버레이 라이브 디스플레이 스크립트
- `src/hl_camera_bringup/blobs/yolov6n_coco_416x416_openvino_2022.1_6shave.blob` — Luxonis 사전변환 YOLOv6n blob (COCO 80클래스)

### Changed (2026-06-29)

- `PROJECT_STATUS.md` — Phase 1 상태 `미시작` → `진행 중`, Camera 상태 업데이트
- `docs/verification/camera/oak_poe_single_test_log.md` — YOLOv6n VPU 추론 및 라이브 디스플레이 검증 결과 추가

### Verification (2026-06-29)

- YOLOv6n 터미널 추론: **통과** — 29.8 fps, Myriad X VPU 동작 확인
- YOLOv6n 라이브 디스플레이: **통과** — 바운딩박스 오버레이 화면 출력 확인
- ROS2 빌드 검증: **통과** — `hl_camera_bringup` ament_python 패키지 기준

### Compatibility

- 기존 Namespace, TF 기준 구현 없음 — 호환성 영향 없음

---

### Added (초기)

- 프로젝트 초기 문서 구조 작성
- `AGENT.md` 작업 기준 문서 추가
- `PROJECT_STATUS.md` 현재 상태 문서 추가
- 시스템, 네트워크, TF, Topic, 센서 설정, 운영 문서 초안 추가
- `docs/` 하위 문서 폴더 구성
- 센서별 개별검증, 4종 통합검증, Fusion 단계 구조 추가
- `docs/verification/` 하위 센서별 검증 기록 폴더 추가

### Changed (초기)

- `README.md`를 ROS2 센서 통합 플랫폼 기준 문서로 확장
- 권장 ROS2 패키지 구조를 개별검증, 통합검증, Fusion 흐름에 맞게 조정

### Verification (초기)

- ROS2 빌드 검증: 수행 불가
- 사유: 현재 저장소에 ROS2 패키지가 없음

### Compatibility (초기)

- 기존 Namespace, Topic, TF, Driver 구현이 없어 호환성 변경 없음
