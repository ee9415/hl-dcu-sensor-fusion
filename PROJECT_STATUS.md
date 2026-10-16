# PROJECT_STATUS.md

## 프레임별 CSV 로깅 (2026-10-16)

- oakapp 0.14.0: TCP 8557 프레임별 구조화 데이터 출력, 멀티뷰 연결 시 자동 CSV 저장.
- frames.csv: RGB/NN/Depth 시각·번호, 중심선 계수/상태, PC 수신 시각 및 지연 추정.
- objects.csv: 프레임별 객체 박스·신뢰도·거리 XYZ(m), 무효 거리는 빈 값.
- UI 저장 경로·프레임/객체 기록 건수·누락·오류 표시. 전체 테스트 58개 통과.
- 네 영상 동시 수신 및 실제 CSV 증가 확인. 단조 시계 최종 검사 중 415프레임,
  6,008객체, 시퀀스 누락 0 확인. 시계/통신 비대칭 영향으로 지연 무효 175행 포함.
- 지연은 구조화 데이터 수신 추정값이며 화면 표시 지연/정밀 동기화 결과가 아니다.
- 상세: `docs/verification/camera/frame_csv_20261016.md`.

## 멀티뷰 Depth 직접 수신 (2026-10-16)

- oakapp 0.13.0: 정렬 Depth의 표시용 H.264 RTSP 8556/depth 추가(640×360, 설정 10 FPS).
- 멀티뷰 Depth 수신/표시 연결. 원본 1920×1080 mm Depth와 객체 XYZ 계산 유지.
- 웹 Visualizer 기본 비활성화, frontend port 자동 할당 해제.
- 57개 테스트 통과. 실제 GUI 검사에서 RGB/YOLO/Depth 프레임 및 이미지 표시 확인.
- 첫 20초 검사에서 BEV 미연결. 전체 네 화면 동시 검증 및 육안 검증은 미완료.
- 상세: `docs/verification/camera/depth_multiview_20261016.md`.

## 앞쪽 통로 목표 추종 (2026-10-16)

- oakapp 0.12.0: 카메라 중심선 ROI 상단을 높이 50%→38%로 확장.
- 연결된 원거리 구간의 좁고 연속된 8행 밴드 중앙을 목표점(흰색 십자)으로 표시.
- 현재 근거리 위치에서 목표로 향하는 1/2차 후보 선택. 기존 프레임별 변화 제한 유지.
- 테스트 54개 통과. 최종 실영상 30.047초/296프레임, 검출 구간 위치 변화 최대 5px.
  미검출 44프레임이 남아 무소실은 미달성. 목표 추종 지연/잘못된 마스크 문제는 남음.
- 별도 로그 455개 유효 목표 중 454개에서 원거리 목표 밴드 내부 도달 확인.
  실제 차로 정답·차량 조향 안전성 검증 아님. BEV 좌표는 미보정 개발 단위.
- 상세: `docs/verification/camera/far_corridor_goal_20261016.md`.

## 수동 교차로 선회 미리보기 (2026-10-16)

- oakapp 0.11.0: 일반 도로 처리 유지, UI 좌회전/우회전/도로 복귀 추가.
- x(s), y(s) 선회 경로는 이전 선의 위치·접선·곡률을 연결하고 프레임별 변화 제한.
- 보라색 검증용 경로. 자동 교차로 판단/출구 차로 인식/차량 운동 보상/조향 제어는 미구현.
- 테스트 48개 통과, 실제 카메라 명령 수락 및 BEV 표시, Ubuntu 멀티뷰 3개 스트림 확인.
- 엄격한 제한으로 전환에 수십 초 이상 걸릴 수 있음. 실제 교차로 주행 안전성 미검증.
- 상세: `docs/verification/camera/intersection_preview_20261016.md`.

## 보수적 중심선 형상·변화율 제한 (2026-10-16)

- oakapp 0.10.0 배포: 기본 3차항 비활성화, 1차 우선/필요 시 2차 사용.
- 최종 단계에서 곡률·기울기와 매 프레임 횡위치/방향각/곡률 변화 제한.
- 테스트 40개 통과. 최종 도로 영상 30.050초/292프레임, 미검출 0,
  위치 변화 p95 2.5px/최대 6px. 서로 다른 영상 구간이므로 개선율은 산출하지 않음.
- 더 긴 관측에는 경계와 제한 경로의 충돌에 따른 일시 무효가 남음.
  미관측 후 재획득 지연도 확인. 모든 장면의 무소실/실제 조향 안전성은 보장하지 않음.
- 차량 제원 미정. 제한은 미보정 개발 BEV 기준이며 차량 제어용 설정 아님.
- 상세: `docs/verification/camera/conservative_centerline_20261016.md`.

## 1차 우선 차수 선택 (2026-10-16)

- oakapp 0.9.0: 1차부터 시작, 오차 지속 시 2차→3차 순차 허용. 단순해지면 낮은 차수로 복귀.
- c0는 현재 위치에 맞춰 피팅하며 0 고정 아님. 전환 평활화 및 불필요 고차항 0 처리 적용.
- 테스트 36개 통과. 최종 실영상 30.062초/292프레임에서 선 미검출 0,
  위치 변화 95백분위 8px/최대 48px. 더 긴 관측에는 일부 무효 프레임이 있어 추가 조정 필요.
- 2/3차 사용 후 실제 1차(c2=c3=0) 복귀 확인. 차량 제어 안전성은 미검증.
- 상세: `docs/verification/camera/adaptive_order_20261016.md`.

## 흰색 점선 우선 추종 (2026-10-16)

- oakapp 0.8.0: 과거 통로 고정 정책을 제거하고 원시 점선 우선 추종으로 변경했다.
- 한쪽 급확장에만 최대 0.8초 억제 후 점선으로 복귀한다. 일반 곡선/양쪽 이동은 빠르게 추종한다.
- 테스트 29개 통과. 실영상 30.186초/290프레임 녹화 및 시각 확인 완료.
- 겹치는 로그 관측 504/504 유효 프레임, 억제 23프레임. 억제 없는 창의 평균
  점선 대비 차이 0.90~3.56px, 억제 후 일반 추종 복귀 확인. 실제 주행 안전성은 미검증이다.
- 상세: `docs/verification/camera/raw_centerline_follow_20261016.md`.

## 중심선 실영상 연속성 검증 (2026-10-16)

- 부분 경계 추적, 표시 범위 회복, 작은 표식 틈 연결, 강건 피팅을 추가했다.
- 유효 관측의 큰 차이는 재획득 대기 대신 이동 속도를 제한해 연속 추적한다.
- 최종 30.011초/292프레임 실영상 분석에서 중심선 미검출 0프레임,
  검출 위치 변화 95백분위 3픽셀/최대 7픽셀. 합성·회귀 테스트 28개 통과.
- 현재 시험 구간에 한정된 결과이며 차량 제어 안전성/다른 도로는 미검증이다.
- 상세: `docs/verification/camera/centerline_continuity_20261016.md`.

## 통로 연속성 업데이트 (2026-10-16)

- oakapp 0.7.0: 한쪽 주행 영역 확장 시 이전 폭과 반대쪽 경계로 중심선을 추정한다.
- 양쪽 공통 이동은 허용하고 양쪽 불확실/축소 및 장시간 단일 경계 의존은 무효 처리한다.
- 전체 23개 테스트 통과, 카메라 full pipeline 실행 확인. 실제 도로 효과는 추가 검증 필요.
- 배포 후 GUI의 BEV/YOLO 재연결 검사 일부 실패. 영상 수신 안정성 추가 확인 필요.
- 상세: `docs/verification/camera/corridor_tracking_20261016.md`.

## Ubuntu 멀티뷰 업데이트 (2026-10-16)

- 기존 멀티뷰를 WSL2 Ubuntu 22.04/WSLg에서 실행할 수 있도록 호환 처리했다.
- 실제 GUI에서 RGB 1920x1080, YOLOPv2 960x540, BEV 640x480 수신 및
  미리보기 생성 확인 완료. Depth는 아직 출력 연동 대기다.
- Linux 비밀번호 파일 저장은 비활성화했다. Windows DPAPI 저장은 유지한다.
- 실행 방법과 검증 범위: `docs/verification/camera/ubuntu_multiview_20261016.md`.

## 1. Summary

작성일: 2026-10-06

프로젝트는 Phase 1 센서별 개별검증 단계이다. Camera 영역에는 단일 OAK-D Pro
PoE용 ROS2 노드 2종과 별도의 6카메라 녹화/Web UI 도구가 있다. LiDAR, GNSS,
IMU, 통합 bringup, TF/URDF, RViz, diagnostics, sensor fusion은 미구현이다.

## 2. Repository Status

| 항목 | 상태 | 근거 |
| --- | --- | --- |
| ROS2 package | 일부 구현 | `src/hl_camera_bringup` |
| ROS2 executable | 2개 | `yolov6n_node`, `yolop_node` |
| 6-camera tool | 구현 | `tools/yolop_6cam_recorder` |
| Launch | 없음 | `launch/` 미작성 |
| Sensor config | 일부 | 단일 카메라 YAML 2개와 도구 내 MX ID 매핑만 존재 |
| TF/URDF | 없음 | frame_id 문자열만 사용하며 TF publisher 없음 |
| RViz config | 없음 | 설정 파일 미작성 |
| Automated tests | 없음 | 장비 검증 스크립트만 존재 |

## 3. Camera Implementation

| 구성 | 상태 | 출력/기능 |
| --- | --- | --- |
| `yolov6n_node` | 구현, 2026-06-29 장비 검증 기록 | `/oak/rgb/image_raw`, `/oak/detections` |
| `yolop_node` | 코드 통합, 현 브랜치 장비 재검증 필요 | 위 2개 topic + `/oak/drivable_area`, `/oak/lane_line` |
| 6-camera recorder | 구현 | 6대 H.265/JSONL, 선택적 YOLOP 후처리 |
| 6-camera Web UI | 구현, 2026-08-12 장비 검증 기록 | live preview, 녹화 제어, playback |
| 6-camera ROS2 bringup | 미구현 | namespace/topic/frame/launch 확정 필요 |

`yolov6n_node`와 `yolop_node`는 동일한 node 입력 장치와 `/oak/*` topic을 사용하므로
현재 형태로 동시에 실행하지 않는다.

## 4. Phase Status

| Phase | 목표 | 상태 | 완료 기준 |
| --- | --- | --- | --- |
| Phase 1 | 센서별 개별검증 | 진행 중 | 4종 센서별 topic, Hz, TF, RViz 검증 |
| Phase 2 | 4종 센서 통합검증 | 미시작 | 통합 launch, 충돌 없는 namespace/topic, 단일 TF tree |
| Phase 3 | Fusion 준비 | 미시작 | timestamp, QoS, calibration, 입력 interface 확정 |
| Phase 4 | Fusion 구현 | 미시작 | fusion node와 output 검증 |

## 5. Sensor Status

| Sensor | Package | Status | Remaining Work |
| --- | --- | --- | --- |
| Camera | `hl_camera_bringup` | 일부 구현 | 6대 ROS2 launch, camera_info, TF, calibration, RViz, latency 검증 |
| LiDAR | 계획: `hl_lidar_bringup` | 미구현 | driver/version 선정, point cloud/Hz/frame 검증 |
| GNSS | 계획: `hl_gnss_bringup` | 미구현 | 연결 방식, fix/status, timestamp 검증 |
| IMU | 계획: `hl_imu_bringup` | 미구현 | 연결 방식, rate, orientation/frame 검증 |

## 6. Integration and Fusion Status

| 영역 | 계획 패키지 | 상태 |
| --- | --- | --- |
| Integrated bringup | `hl_sensor_bringup` | 미구현 |
| Description/TF | `hl_sensor_description` | 미구현 |
| Common configuration | `hl_sensor_config` | 미구현 |
| Diagnostics | `hl_sensor_diagnostics` | 미구현 |
| Fusion | `hl_sensor_fusion` | 미구현 |

## 7. Verification Status

| 날짜 | 대상 | 결과 | 제한 |
| --- | --- | --- | --- |
| 2026-06-29 | `hl_camera_bringup` YOLOv6n | build/node/topic 및 약 30 Hz 통과 기록 | 단일 카메라 |
| 2026-08-12 | 6-camera Web UI | DCU 배포, 6대 약 10 FPS 기록 | 비-ROS2 도구, 자동 screenshot 없음 |
| 2026-10-06 | YOLOP ROS2 통합 | merge, Python 구문 검사, `git diff --check` 통과 | ROS2 build/장비 실행 미수행 |

## 8. Confirmed and Open Decisions

확인된 기준:

- 6카메라 역할과 논리 배치는 `docs/camera_use_case.md`를 따른다.
- 도구의 장비 식별은 `cam_01`~`cam_06`과 MX ID 매핑을 사용한다.
- 단일 카메라 검증망은 host `192.168.200.40/24`, camera `192.168.200.31`이다.

미확정 항목:

- 최종 namespace와 6카메라 ROS2 topic
- 차량 기준 TF 부모 frame 및 실제 extrinsic
- 6대 카메라 개별 IP와 카메라망 설계
- ROS Domain ID, 시간 동기화, QoS 정책
- LiDAR/GNSS/IMU driver와 버전

## 9. Immediate Next Steps

1. YOLOP ROS2 node `colcon build` 및 단일 장비 런타임 재검증
2. 카메라 ID 표기(`cam_01`)와 ROS frame/topic 표기(`cam01`) 변환 규칙 확정
3. 6대 ROS2 namespace/topic/frame/launch 설계 승인
4. 카메라별 IP, 장착 좌표, intrinsics/extrinsics 현장 기록
5. LiDAR, GNSS, IMU driver 선정과 개별검증
6. 통합 bringup, TF, RViz, diagnostics 구현

## 10. Risks

| 위험 | 영향 | 대응 |
| --- | --- | --- |
| ROS2 단일 카메라와 비-ROS2 6대 도구의 상태 혼동 | 완료 범위 과대평가 | 문서와 검증표에서 실행 경로를 분리 |
| Namespace/topic/TF 미확정 | 6대 ROS2 확장과 fusion 지연 | 구현 전 기준 문서 승인 |
| 장착 문서의 설계값을 실측값으로 오인 | 잘못된 TF/calibration | 설계안과 현장 확정값을 분리 저장 |
| 자동화 테스트 부재 | 회귀 오류 발견 지연 | hardware-independent unit test 추가 |

## 11. CAM01 OAK 4D Pro Development Direction

작성일: 2026-10-15

CAM01 전방 인식의 차기 개발 방향으로 OAK 4D Pro, YOLOPv2, 객체별 Stereo
Depth 거리 결합 및 주행 가능 영역 기반 가상 중심선 생성을 정리했다. 별도 로컬
`OAK_4D_PRO` 프로젝트의 녹화 프로그램과 YOLOPv2 Standalone 앱 파일 존재를
확인했으며, 사용자 확인에 따르면 현재 프로그램은 원본 RGB와 후처리 영상을
출력한다.

이 항목은 설계 검토 상태이며 현재 저장소의 구현 완료 상태를 변경하지 않는다.
Depth 결합, 3차식 중심선, 후처리 영상 표시, ROS2 구조화 데이터 및 통합 10 FPS는
모두 미구현 또는 검증 필요 상태이다. 상세 기준은
`docs/oak4d_cam01_perception_plan.md`를 따른다.

### 2026-10-15 기준 코드 이관

- 임시 `D:\git_hub\OAK_4D_PRO` 프로젝트의 검증된 YOLOPv2 Standalone 앱을
  `apps/yolopv2_standalone`으로 선별 이관했다.
- Windows RGB/YOLO 결과 녹화 UI를 `tools/oak_recorder.py`로 이관했다.
- `third_party`, 캐시, 로그, 중간 산출물과 개별 DLC는 이관하지 않았다.
- OAK App 실행에 필요한 NNArchive는 로컬에 배치하되 Git 추적에서 제외한다.
- 이관 단계에서는 기능을 변경하지 않았다. Depth와 가상 중심선은 다음 단계부터
  독립적으로 추가하고 검증한다.

### 2026-10-15 기준 RGB 정렬 Depth 구현

- OAK4 RVC4용 `CAM_B/C -> StereoDepth -> ImageAlign` 파이프라인을 추가했다.
- CAM_A의 보정 기반 왜곡 제거 영상을 정렬 기준으로 사용하고, `depth` 및 `full`
  단계에서 `DepthAligned` 토픽을 출력한다.
- Stereo Depth는 `640x400`, 10 FPS, `ROBOTICS` preset, LR-check, subpixel로
  설정했다. 정렬 출력 좌표와 크기는 CAM_A `1920x1080` 기준이다.
- `PIPELINE_STAGE=depth`에서는 NN 없이 RGB와 정렬 Depth만 독립 검증할 수 있다.
- Python 문법 및 파이프라인 구성 정적 검사는 통과했다.
- OAK4-D R9 장비 `1142227768`과 Luxonis OS RVC4 1.41.0은 발견했으나,
  로그인 토큰 만료로 장비가 비밀번호 재입력을 요구해 런타임 검증은 중단했다.
- 이 RGB 정렬 Depth 단계 당시에는 객체 거리·XYZ와 가상 중심선을 구현하지
  않았으며, 객체 거리·XYZ는 아래 후속 단계에서 추가했다.

### 2026-10-15 기준 Windows 멀티뷰 UI

- `tools/oak_recorder.py`를 2×2 멀티뷰 화면으로 변경했다.
- 패널은 `원본 RGB`, `인식 결과`, `RGB 정렬 Depth`, `BEV · 가상 중심선`으로
  고정했다.
- 현재 RTSP가 제공되는 원본 RGB와 인식 결과는 동시에 실시간 표시한다.
- Depth와 BEV는 PC 출력이 아직 없으므로 빈 영상을 임의 생성하지 않고 각각
  `출력 연동 대기`, `후속 단계` 상태를 표시한다.
- 기존 RGB/YOLO 선택 녹화, 녹화 중지, 파일 저장, 장비 검색 및 비밀번호 처리
  동작은 유지했다.
- Python 문법 검사와 Tk 4패널 위젯 생성 검사를 통과했다. 실제 두 RTSP 영상이
  표시되는 최종 UI 검증은 장비 재인증 후 수행해야 한다.

### 2026-10-15 기준 객체 거리 및 CAM_A XYZ 구현

- `full` 단계의 YOLOPv2 객체 박스와 `DepthAligned` 프레임을 후처리 노드에서
  결합한다.
- 각 박스 중앙 50% ROI에서 0.3~50 m만 유효값으로 사용하고, 최소 20 pixel 및
  ROI의 10% 이상을 요구한다. 10~90 percentile 범위의 중앙값을 대표 Z로 쓴다.
- 정렬된 Depth 프레임의 변환 후 intrinsic을 사용해 CAM_A optical 좌표계의
  X/Y/Z(mm)를 계산한다.
- 모든 객체 박스는 기존 출력에 유지하고, 유효한 거리만 `SpatialDetections`로
  출력한다. 후처리 영상에는 `Z=...m`, 무효 시 `Z=?`를 표시한다.
- 거리 계산 단위 테스트 4건과 Python 문법 검사는 통과했다.
- 이 구현 시점에는 실장비 거리 정확도·FPS·메시지 출력 검증이 대기 상태였으며,
  기본 런타임 출력은 아래 재인증 단계에서 확인했다.
- 가상 중심선과 BEV 계산은 다음 구현 단계이다.

### 2026-10-15 기준 실장비 재인증 및 멀티뷰 실행

- OAK4-D R9 `1142227768` 재인증 후 `full` 앱을 실행했다.
- `YOLOPv2Result`의 객체 거리 `Z=1.3m`와 `DepthAligned`의 공간 검출 표시를
  실제 화면에서 확인했다.
- 런타임 로그는 안정 구간 약 9.79 FPS, 후처리 약 12.64 ms, 객체 1개 중
  Depth 유효 객체 1개를 기록했다.
- Windows 멀티뷰용 Python 3.12 가상환경과 OpenCV/Pillow 의존성을 구성하고
  IP `192.168.0.4`로 2×2 창을 실행했다.
- 실제 거리 정확도와 장시간 성능은 아직 검증하지 않았다.

### 2026-10-15 기준 BEV 및 3차식 가상 중심선

- 주행가능영역 마스크에서 영상 중심과 연결된 좌우 경계를 추출한다.
- 개발용 고정 IPM으로 전방 40 m, 좌우 12 m BEV에 변환하고
  `y(x)=c0+c1*x+c2*x^2+c3*x^3`을 fitting한다.
- cyan 가상 중심선을 후처리 영상으로 재투영하고, `640x480` BEV를
  `rtsp://<device-ip>:8553/bev`로 출력한다.
- Windows 멀티뷰의 우측 아래 패널을 BEV RTSP에 연결했다.
- 합성 직선·곡선·무효 입력과 BEV 렌더링 테스트 4건을 포함한 전체 8건이
  통과했다.
- 실장비에서 중심선 유효 상태, 계수 및 residual 출력을 확인했고 BEV RTSP
  프레임 `640x480` 수신에 성공했다. 안정 구간 FPS는 약 9.6 이상이었다.
- 현재 IPM은 개발용 고정 사다리꼴이므로 차량 장착 높이·pitch 실측 후 차량
  좌표계 보정과 현장 주행 검증이 필요하다.

### 2026-10-15 기준 녹화 영상 분석 및 중심선 안정화

- 약 505초 녹화본을 1초 간격으로 분석했다. 모니터 전체가 보이는 구간은 고정
  IPM의 가정과 달라 하단 중심선 위치의 95백분위 프레임 점프가 화면 폭의 약
  48.0%였고, 전체화면 재생 구간은 약 5.4%였다.
- 중심선에 지수 이동 평균, 순간 급변 억제, 최대 3프레임 누락 유지를 추가했다.
- 주행영역 추적은 화면 하단 중앙과 연결된 성분만 시작점으로 인정하도록 강화했다.
- 거리 무효 객체의 `Z=?`는 큰 박스에 한해 최대 4개만 표시한다. 모니터 속 차량은
  실제 입체 물체가 아니므로 Stereo Depth 거리를 임의 생성하지 않는다.
- 전체 단위 테스트 11건과 Python 문법 검사를 통과했다.
- OAK4-D R9에 앱 0.5.1을 다시 배포했으며, 워밍업 후 약 9.81 FPS,
  중심선 유효 상태와 BEV 스트림 시작을 확인했다.
- 앱 0.5.1 재녹화본 58.1초·581프레임 검토에서 중심선 검출률 100%, 연속
  프레임 95백분위 이동량 화면 폭 약 0.52%, 10% 이상 급변 0회를 확인했다.
  추가 평활은 급커브 반응 지연 위험 때문에 적용하지 않았다.

### 2026-10-15 기준 OAK4 출력 및 리소스 스냅샷

- 외부 RTSP는 RGB `1920x1080`, YOLOPv2 결과 `960x540`, BEV `640x480`의
  H.264 영상 3개이며 모두 10 FPS로 설정했다.
- RemoteConnection 토픽은 `DepthAligned`, `SpatialDetections`,
  `YOLOPv2Result`, `CenterlineBEV` 4개다.
- 실장비 누적 평균은 약 9.98 FPS, 후처리는 약 32.4~34.0 ms/frame이었다.
- 장비 전체 CPU 약 20.8~25.5%, 메모리 약 2.72/7.03 GiB, 전력 약
  2.7~6.2 W, 최고 관측 CPU 온도 약 61 C였으며 시스템 경고는 없었다.
- 앱 Python 프로세스는 코어 하나 기준 CPU 약 70.3%, RSS 약 950 MiB였다.
- 58초 녹화에서 RGB와 결과 영상 합산 평균 비트레이트는 약 25.5 Mbps였다.
- 시스템 `/` 파티션 사용률은 약 87%로 새 앱 이미지 누적 관리가 필요하다.
- 상세 표본과 해석은 `docs/verification/camera/oak4_runtime_resource_snapshot.md`에
  기록했다.
