# OPERATION_MANUAL.md

## 1. Purpose

이 문서는 현장 운영자가 ROS2 센서 통합 시스템을 실행, 확인, 종료하는 절차를
정의한다.

현재 저장소에는 실행 가능한 launch 파일이 없으므로 실제 운영 절차는 아직
제공할 수 없다.

## 2. Pre-Operation Checklist

| 항목 | 확인 |
| --- | --- |
| Jetson Orin NX 전원 | 미정 |
| 센서 전원 | 미정 |
| PoE switch 연결 | 미정 |
| Network IP 설정 | 미정 |
| ROS Domain ID | 미정 |
| Driver 설치 | 미정 |
| Workspace build | 미정 |

## 3. Planned Operation Flow

운영 절차는 개별검증을 먼저 수행한 뒤 통합검증으로 진행한다.

### 3.1 Individual Sensor Verification

1. 네트워크 연결 확인
2. ROS2 환경 설정
3. 센서별 단독 launch 실행
4. 센서별 Topic 및 Hz 확인
5. 센서별 TF frame 확인
6. 센서별 RViz 표시 확인
7. 검증 결과를 `docs/verification/<sensor>/` 하위에 기록

### 3.2 Integrated Sensor Verification

1. 개별검증 완료 여부 확인
2. 전체 bringup launch 실행
3. Namespace 및 Topic 충돌 확인
4. TF tree 단일 연결 확인
5. 4종 센서 RViz 동시 표시 확인
6. Fusion 입력 후보 topic과 timestamp 확인
7. 통합 검증 결과를 `docs/verification/integration/` 하위에 기록

## 4. Planned Commands

구현 후 아래 명령을 실제 패키지명에 맞게 확정한다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hl_camera_bringup <camera_launch>.launch.py
ros2 launch hl_lidar_bringup <lidar_launch>.launch.py
ros2 launch hl_gnss_bringup <gnss_launch>.launch.py
ros2 launch hl_imu_bringup <imu_launch>.launch.py
ros2 launch hl_sensor_bringup <integrated_launch>.launch.py
```

## 5. Shutdown

운영 종료 시 다음을 확인한다.

- Launch process 정상 종료
- 센서 driver process 잔류 여부
- 로그 저장 여부
- 센서 전원 차단 순서

## 6. Failure Handling

| 증상 | 확인 항목 |
| --- | --- |
| Topic 없음 | driver 실행, namespace, network |
| Hz 낮음 | network bandwidth, CPU/GPU load, QoS |
| TF 없음 | static transform publisher, URDF launch |
| RViz 표시 안 됨 | Fixed Frame, message type, QoS |
