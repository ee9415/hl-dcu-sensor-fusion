# CHANGELOG.md

이 문서는 저장소의 주요 변경 사항을 기록한다.

형식은 Keep a Changelog의 구조를 참고하되, 본 프로젝트에서는 검증 결과와
운영 영향도를 함께 기록한다.

## [Unreleased]

### Fixed / Verified (2026-10-21, 경계선 장비 배포 0.15.1)

- 차선 근거가 있는 짧은 주행 영역 마스크 틈만 연결, 넓은 틈은 유지.
- 실제 멀티뷰와 네 RTSP 수신 및 중심선 없는 좌우 경계선 표시 확인.
- 전체 69개 테스트 통과. 최종 표본 349 중 343프레임 유효, CSV 누락 0.
- GUI 검사의 선회 버튼 표시 판정 수정, 읽기 전용 RTSP 표본 저장 도구 추가.

### Added (2026-10-21, 한라대 가상 좌우 경계선)

- 설정 가능한 차폭·좌우 여유 폭을 기준으로 독립 경계 쌍을 직접 추정.
- 도로 급확장 시 통로 폭 유지, 곡선 연결·프레임 간 변화 제한·좁은 구간 무효화.
- 활성 중심선/선회 처리를 경계선 전용 결과·BEV로 교체하고 기존 알고리즘은 보존.
- 멀티뷰 경계선 표시 안내와 CSV 경계 배열·상태·설정값 추가. 소스 버전 0.15.0.
- 전체 68개 테스트 통과. 장비 배포와 한라대 현장 검증은 미수행.

### Documentation (2026-10-15, OAK4 출력 및 리소스 스냅샷)

- `full` 앱의 3개 RTSP 영상과 4개 RemoteConnection 토픽 규격 기록
- 실측 9.98 FPS, 후처리 시간, CPU·메모리·전력·온도와 프로세스 RSS 기록
- 58초 RGB·결과 녹화의 프레임 수와 약 25.5 Mbps 합산 비트레이트 기록
- 시스템 파티션 약 87% 사용과 GPU/NPU 개별 사용률 미계측 제한 기록

### Fixed (2026-10-15, OAK4 중심선 영상 분석 후 안정화)

- 중심선 좌우 흔들림을 줄이는 지수 이동 평균과 순간 급변 억제 추가
- 최대 3프레임의 짧은 주행영역 검출 누락 동안 이전 중심선 유지
- 화면 하단 중앙과 연결되지 않은 주행영역 성분을 배제해 모니터 테두리·주변 배경 오검출 완화
- 유효 거리 표시는 모두 유지하면서 큰 객체의 무효 `Z=?` 표시는 프레임당 최대 4개로 제한
- 중심선 시간 필터 단위 테스트 3건 추가, 앱 버전을 `0.5.1`로 갱신
- 앱 0.5.1 재녹화 581프레임에서 중심선 검출률 100%, 연속 프레임 10% 이상
  급변 0회를 확인하고 현재 평활 설정을 유지

### Added (2026-10-15, OAK4 3차식 가상 중심선)

- 주행가능영역의 좌우 경계 추출과 개발용 IPM BEV 변환 추가
- 전방 거리 기준 3차식 가상 중심선 fitting, 유효성 및 residual 계산 추가
- 후처리 영상의 cyan 중심선 재투영과 `CenterlineBEV` 출력 추가
- `rtsp://<device-ip>:8553/bev` 스트림 및 Windows 멀티뷰 BEV 연결 추가
- 직선·곡선·무효 마스크와 BEV 렌더링 단위 테스트 4건 추가
- BEV 렌더링을 벡터화해 실장비 안정 구간 약 9.6 FPS 이상 확인
- 앱 버전을 `0.5.0`으로 갱신

### Changed (2026-10-15, OAK4 실장비 및 멀티뷰 실행)

- 장비 재인증 후 객체 거리 결합 `full` 파이프라인의 실제 실행 확인
- 약 9.79 FPS와 유효 Depth 객체 출력 및 결과 영상의 `Z` 거리 표시 확인
- Windows 멀티뷰 실행용 OpenCV/Pillow 요구사항 파일과 venv 설정 절차 추가
- 실장비 IP로 2×2 멀티뷰 창 실행 확인

### Added (2026-10-15, OAK4 객체 거리 및 CAM_A XYZ)

- YOLOPv2 객체 박스 중앙 ROI와 RGB 정렬 Depth를 결합하는 robust 거리 계산 추가
- 유효 범위, 최소 픽셀·비율 및 percentile trimming 기반 Depth 무효 판정 추가
- 정렬 프레임 intrinsic 기반 CAM_A optical XYZ(mm) 계산과
  `SpatialDetections` 출력 추가
- 후처리 영상에 객체별 `Z=...m` 또는 `Z=?` 상태 표시 추가
- 합성 Depth 입력을 이용한 거리·이상치·희소값·좌표 계산 단위 테스트 4건 추가
- 앱 버전을 `0.4.0`으로 갱신

### Changed (2026-10-15, OAK4 Windows 멀티뷰 UI)

- 녹화 프로그램을 단일 선택 미리보기에서 2×2 멀티뷰 화면으로 변경
- 원본 RGB와 인식 결과를 동시에 표시하도록 기존 RTSP 미리보기 경로 재배치
- RGB 정렬 Depth와 BEV·가상 중심선 패널을 후속 출력 연동 대기 상태로 추가
- 기존 이중 MP4 녹화·저장 및 장비 연결 동작 유지

### Added (2026-10-15, OAK4 RGB 정렬 Depth)

- CAM_B/C Stereo Depth를 CAM_A 좌표계로 변환하는 RVC4 `ImageAlign` 경로 추가
- `PIPELINE_STAGE=depth` 진단 단계와 `DepthAligned` Visualizer 토픽 추가
- `full` 단계에서도 후속 객체 거리 및 BEV 처리를 위한 정렬 Depth 생성
- Stereo Depth에 `ROBOTICS` preset, LR-check 및 subpixel 설정 적용
- 앱 버전을 `0.3.0`으로 갱신
- 장비 로그인 토큰 만료로 실장비 실행은 재인증 후 검증 필요 상태로 기록

### Added (2026-10-15, OAK4 기준 코드 이관)

- 임시 OAK4 검증 프로젝트의 YOLOPv2 Standalone 앱을
  `apps/yolopv2_standalone`으로 선별 이관
- 원본 RGB와 YOLO 결과 RTSP 수신·MP4 녹화 UI를 `tools/oak_recorder.py`로 이관
- 대용량 OAK4 모델과 녹화 산출물을 Git 추적에서 제외
- 이관 단계에서는 기존 동작을 보존하고 Depth 및 가상 중심선 기능은 추가하지 않음

### Added (2026-10-15, CAM01 OAK 4D Pro 개발 방향 기록)

- OAK 4D Pro와 YOLOPv2 기반 CAM01 전방 인식 개발 방향 문서 추가
- 객체 박스와 RGB 정렬 Stereo Depth의 거리·3D 위치 결합 방안 정리
- 주행 가능 영역 기반 가상 중심선의 3차 다항식 피팅 및 영상 재투영 구조 정리
- 원본 RGB, 후처리 영상, 구조화 데이터와 녹화 출력의 역할 구분
- 모든 항목을 구현 전 또는 검증 필요 상태로 기록

### Changed (2026-10-06, 문서 정합성 갱신)

- 현재 구현을 ROS2 단일 카메라 경로와 비-ROS2 6카메라 운용 도구로 구분
- `yolop_node`와 실제 `/oak/*` topic을 루트 상태/구조/topic 문서에 반영
- 6대 MX ID 매핑과 확인된 네트워크 값을 센서 설정 문서에 반영
- `docs/camera_use_case.md`의 CAM01~CAM06 배치를 카메라 역할 정본으로 지정
- 구형 전·후방 근거리/원거리 배치 기반 sensor alignment 문서를 신규 배치에 맞게 수정
- 설계 목표, 코드 확인, 과거 장비 검증, 현장 미확정 값을 명시적으로 분리

### Verification (2026-10-06, 문서 정합성 갱신)

- 저장소 코드, `setup.py`, 검증 로그, 6-camera tool의 MX ID 매핑과 문서 대조
- 구형 camera frame/배치 identifier 잔존 여부 검색
- Markdown link 검사 및 `git diff --check` 수행

### Added (2026-10-06, YOLOP ROS2 branch integration)

- `origin/main`의 `yolop_node`와 YOLOP blob을 6-camera Web UI 작업 브랜치에 병합
- `yolop_node` console entry point를 현재 브랜치에 통합

### Verification (2026-10-06, YOLOP ROS2 branch integration)

- Python source 정적 구문 검사: 통과
- `git diff --check`: 통과
- ROS2 build 및 실제 장비 실행: 현재 Windows 환경에서 미수행

### Changed (2026-08-12, 6-camera Web UI layout)

- 6-camera Web UI layout changed from `1 / 2 / 2 / 1` to `3 / 3`
- Default camera order changed to top `cam_04 / cam_01 / cam_05`, bottom `cam_02 / cam_06 / cam_03`
- Preview and playback panels changed to a cropped 16:9 view with tighter header, grid, and page spacing

### Verification (2026-08-12, 6-camera Web UI layout)

- `git diff --check`: passed
- Local and DCU `web_ui/index.html` SHA-256 matched after deployment
- DCU Web UI restart: passed; all six cameras reported approximately 10 FPS
- Automated browser screenshot: not performed because no browser instance was available; visual confirmation is required on the operator display

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
