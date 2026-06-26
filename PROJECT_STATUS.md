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
| ROS2 package | 없음 | `src/` 폴더 없음 |
| Launch | 없음 | launch 파일 없음 |
| Driver wrapper | 없음 | 소스 코드 없음 |
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

## 4. Verification Status

| 검증 항목 | 결과 | 사유 |
| --- | --- | --- |
| Build | 수행 불가 | ROS2 패키지 없음 |
| Launch | 수행 불가 | launch 파일 없음 |
| Node | 수행 불가 | 실행 노드 없음 |
| Topic | 수행 불가 | publish 노드 없음 |
| TF | 수행 불가 | TF publisher 없음 |
| Hz | 수행 불가 | Topic 없음 |
| RViz | 수행 불가 | RViz 설정 및 센서 데이터 없음 |

## 5. Immediate Next Steps

1. Namespace, Topic, TF naming 기준 확정
2. 네트워크 설계 확정
3. 센서별 ROS2 driver 선정 및 버전 고정
4. `src/` 하위 ROS2 패키지 생성
5. Bringup launch 계층 설계
6. RViz 검증 환경 추가
7. 실제 장비 기반 Hz, TF, 데이터 수신 검증 기록

## 6. Risk

| 위험 | 영향 | 대응 |
| --- | --- | --- |
| 센서 IP 미확정 | Launch 및 네트워크 구성 지연 | NETWORK.md에 현장 설정 기록 |
| TF 기준 미확정 | 센서 융합 불가 | TF_TREE.md 승인 후 구현 |
| Driver 버전 미고정 | 재현성 저하 | 벤더 driver commit/tag 기록 |
| 실장 위치 미확정 | extrinsic 값 미정 | 차량 좌표계 실측 후 반영 |
