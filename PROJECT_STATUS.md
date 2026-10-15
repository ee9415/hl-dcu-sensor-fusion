# PROJECT_STATUS.md

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
