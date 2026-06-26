# TOPIC_LIST.md

## 1. Purpose

이 문서는 ROS2 Topic 이름, message type, publish rate, source node를 관리한다.

현재 저장소에는 실행 가능한 ROS2 node가 없으므로 실제 topic은 존재하지 않는다.

## 2. Current Topic Status

| Sensor | Topic | Type | Hz | Status |
| --- | --- | --- | ---: | --- |
| Camera | 미정 | 미정 | 미정 | 미구현 |
| LiDAR | 미정 | 미정 | 미정 | 미구현 |
| GNSS | 미정 | 미정 | 미정 | 미구현 |
| IMU | 미정 | 미정 | 미정 | 미구현 |

## 3. Topic Definition Rule

Topic 이름은 기존 차량 시스템과 호환되어야 한다.

초기 구현 전 다음 정보를 확정한다.

- Namespace
- Sensor instance name
- Raw data topic
- Rectified/converted data topic
- Diagnostics topic
- QoS profile
- Expected Hz

## 4. Verification

구현 후 다음 명령으로 확인한다.

```bash
ros2 topic list
ros2 topic info <topic_name>
ros2 topic hz <topic_name>
ros2 topic echo <topic_name> --once
```
