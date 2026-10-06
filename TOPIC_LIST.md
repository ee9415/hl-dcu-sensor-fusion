# TOPIC_LIST.md

## 1. Purpose

실제 구현된 ROS2 topic과 향후 후보 interface를 구분하여 관리한다.

기준일: 2026-10-06

## 2. Implemented Topics

### `yolov6n_node`

| Topic | Type | QoS/Rate | Verification |
| --- | --- | --- | --- |
| `/oak/rgb/image_raw` | `sensor_msgs/msg/Image` | Best Effort, depth 10 / 약 30 Hz 기록 | 2026-06-29 검증 |
| `/oak/detections` | `vision_msgs/msg/Detection2DArray` | depth 10 / 약 30 Hz 기록 | 2026-06-29 검증 |

### `yolop_node`

| Topic | Type | QoS | Verification |
| --- | --- | --- | --- |
| `/oak/rgb/image_raw` | `sensor_msgs/msg/Image` | Best Effort, depth 10 | 코드 확인, 현 브랜치 장비 재검증 필요 |
| `/oak/detections` | `vision_msgs/msg/Detection2DArray` | depth 10 | 코드 확인, 현 브랜치 장비 재검증 필요 |
| `/oak/drivable_area` | `sensor_msgs/msg/Image` (`mono8`) | Best Effort, depth 10 | 코드 확인, 현 브랜치 장비 재검증 필요 |
| `/oak/lane_line` | `sensor_msgs/msg/Image` (`mono8`) | Best Effort, depth 10 | 코드 확인, 현 브랜치 장비 재검증 필요 |

두 노드는 같은 topic 이름을 사용하므로 현재 구성에서는 동시에 실행하지 않는다.
두 노드 모두 `camera_info` topic을 발행하지 않는다.

## 3. Six-Camera Proposed Interface

다음은 `docs/camera_use_case.md`의 기준안이며 아직 구현된 topic이 아니다.

| Data | Proposed Pattern | Example |
| --- | --- | --- |
| Raw image | `/camera/{cam_id}/image_raw` | `/camera/cam01/image_raw` |
| Camera info | `/camera/{cam_id}/camera_info` | `/camera/cam01/camera_info` |
| Detection | `/perception/{cam_id}/detections` | `/perception/cam01/detections` |
| Drivable area | `/perception/{cam_id}/drivable_area` | `/perception/cam01/drivable_area` |
| Lane line | `/perception/{cam_id}/lane_line` | `/perception/cam01/lane_line` |
| Diagnostics | `/diagnostics/camera/{cam_id}` | `/diagnostics/camera/cam01` |

최종 namespace, remap, QoS, expected Hz는 launch 설계와 함께 승인해야 한다.

## 4. Other Sensors

| Sensor | Topic | Type | Status |
| --- | --- | --- | --- |
| LiDAR | 미정 | 미정 | 미구현 |
| GNSS | 미정 | 미정 | 미구현 |
| IMU | 미정 | 미정 | 미구현 |
| Fusion output | 미정 | 미정 | 미구현 |

## 5. Verification

```bash
ros2 topic list
ros2 topic info --verbose <topic_name>
ros2 topic hz <topic_name>
ros2 topic echo <topic_name> --once
```
