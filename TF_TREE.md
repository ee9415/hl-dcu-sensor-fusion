# TF_TREE.md

## 1. Purpose

이 문서는 차량 기준 TF 구조와 센서 frame 관계를 관리한다.

현재 저장소에는 TF publisher, URDF, static transform launch가 없으므로 실제 TF
tree는 아직 존재하지 않는다.

## 2. Current Status

| 항목 | 상태 |
| --- | --- |
| base frame | 미정 |
| camera frame | 미정 |
| lidar frame | 미정 |
| gnss frame | 미정 |
| imu frame | 미정 |
| static transform | 미구현 |
| URDF | 미구현 |

## 3. Proposed Decision Items

구현 전에 아래 항목을 확정해야 한다.

- 차량 기준 frame 이름
- 각 센서 mounting 위치
- 각 센서 optical frame 필요 여부
- GNSS antenna frame 정의
- IMU frame과 차량 frame의 축 정렬
- TF publish 주체

## 4. Verification

구현 후 다음 명령으로 검증한다.

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo <parent_frame> <child_frame>
```

RViz에서 Fixed Frame 기준으로 Camera, LiDAR, GNSS, IMU 데이터가 동일 좌표계에
표시되는지 확인한다.
