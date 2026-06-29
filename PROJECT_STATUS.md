# PROJECT_STATUS.md

## 1. Summary

작성일: 2026-06-26

현재 저장소는 초기 문서화 단계이다. ROS2 패키지, launch 파일, driver 래퍼,
센서 설정 파일, RViz 설정, TF 설정은 아직 존재하지 않는다.

## 2. Repository Status

| 항목 | 상태 | 근거 |
| --- | --- | --- |
| README.md | 존재 | 저장소 루트에서 확인 |
| AGENT.md | 작성됨 | AI 작업 기준 문서 추가 |
| ROS2 package | 없음 | `src/` 폴더 구조 생성됨, ROS2 패키지 미완 |
| Launch | 없음 | launch 파일 없음 |
| Driver wrapper | 없음 | 검증 스크립트 존재, ROS2 노드 미완 |
| Sensor config | 없음 | config 파일 없음 |
| TF definition | 없음 | TF 문서 외 구현 없음 |
| RViz config | 없음 | rviz 파일 없음 |

## 3. Implementation Status

| 영역 | 상태 | 비고 |
| --- | --- | --- |
| Camera | 미구현 | OAK-D Pro PoE 6대 기준 문서화 필요 |
| LiDAR | 미구현 | Livox Mid-360S driver 연동 방식 결정 필요 |
| GNSS | 미구현 | Septentrio Mosaic-go ROS2 연동 방식 결정 필요 |
| IMU | 미구현 | Xsens MTi-630 ROS2 driver 연동 방식 결정 필요 |
| Namespace | 미정 | 차량 시스템 기준 확정 필요 |
| Topic | 미정 | 센서별 Topic 확정 필요 |
| TF | 미정 | base frame 및 sensor frame 확정 필요 |
| Network | 미정 | 센서 IP, NIC, VLAN 여부 확인 필요 |

## 4. Phase Status

| Phase | 목표 | 상태 | 완료 기준 |
| --- | --- | --- | --- |
| Phase 1 | 센서별 개별검증 | 진행 중 | 센서별 topic, Hz, TF, RViz 검증 기록 |
| Phase 2 | 4종 통합검증 | 미시작 | 통합 launch, namespace, TF tree, RViz 검증 |
| Phase 3 | Fusion 준비 | 미시작 | 입력 topic, timestamp, QoS, calibration 기준 확정 |
| Phase 4 | Fusion 구현 | 미시작 | fusion node build, launch, output topic 검증 |

## 5. Individual Verification Status

| Sensor | Package Plan | Status | Required Verification |
| --- | --- | --- | --- |
| Camera | `hl_camera_bringup` | 진행 중 | ROS2 토픽 발행 검증 완료 / 6대 확장, TF, RViz 미완 |
| LiDAR | `hl_lidar_bringup` | 미구현 | point cloud topic, Hz, frame, RViz |
| GNSS | `hl_gnss_bringup` | 미구현 | fix/status topic, timestamp, frame |
| IMU | `hl_imu_bringup` | 미구현 | imu topic, Hz, orientation convention, frame |

## 6. Integration and Fusion Status

| 영역 | Package Plan | Status | Required Verification |
| --- | --- | --- | --- |
| Integrated bringup | `hl_sensor_bringup` | 미구현 | 4종 동시 실행, namespace 충돌 없음, TF tree 연결 |
| Description/TF | `hl_sensor_description` | 미구현 | URDF/static TF, `view_frames` 검증 |
| Configuration | `hl_sensor_config` | 미구현 | 센서별 config, calibration, 차량별 override |
| Diagnostics | `hl_sensor_diagnostics` | 미구현 | node, topic, Hz, TF 상태 확인 |
| Fusion | `hl_sensor_fusion` | 미구현 | 입력 interface 확정 후 구현 |

## 7. Verification Status

| 검증 항목 | 결과 | 사유 |
| --- | --- | --- |
| Build | 수행 불가 | ROS2 패키지 없음 |
| Launch | 수행 불가 | launch 파일 없음 |
| Node | 수행 불가 | 실행 노드 없음 |
| Topic | 수행 불가 | publish 노드 없음 |
| TF | 수행 불가 | TF publisher 없음 |
| Hz | 수행 불가 | Topic 없음 |
| RViz | 수행 불가 | RViz 설정 및 센서 데이터 없음 |

## 8. Immediate Next Steps

1. Namespace, Topic, TF naming 기준 확정
2. 네트워크 설계 확정
3. 센서별 ROS2 driver 선정 및 버전 고정
4. 센서별 개별검증 패키지 생성
5. 센서별 launch 및 RViz 검증 환경 추가
6. 센서별 Hz, TF, 데이터 수신 검증 기록
7. 4종 통합 bringup 패키지 생성
8. Fusion 입력 interface 확정

## 9. Risk

| 위험 | 영향 | 대응 |
| --- | --- | --- |
| 센서 IP 미확정 | Launch 및 네트워크 구성 지연 | NETWORK.md에 현장 설정 기록 |
| TF 기준 미확정 | 센서 융합 불가 | TF_TREE.md 승인 후 구현 |
| Driver 버전 미고정 | 재현성 저하 | 벤더 driver commit/tag 기록 |
| 실장 위치 미확정 | extrinsic 값 미정 | 차량 좌표계 실측 후 반영 |
