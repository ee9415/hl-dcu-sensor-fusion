# gps_waypoint_tracker_ros2

`gps_waypoint_tracker.py`의 계산 로직을 유지하고 ROS1 `rospy` 인터페이스를
ROS2 Humble `rclpy` 인터페이스로 변환한 패키지입니다.

## 빌드

```bash
cd /home/jigu/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select gps_waypoint_tracker_ros2
source /home/jigu/ros2_ws/install/setup.bash
```

## 헤드리스 실행

```bash
ros2 launch gps_waypoint_tracker_ros2 gps_waypoint_tracker.launch.py \
  csv_path:="/home/jigu/ros2_ws/src/0730_gps_test/waypoint_csv/0730_waypoint_straight.csv" \
  fix_topic:=/gps/fix \
  enable_plot:=false \
  enable_rviz:=true
```

## 그래프 포함 실행

```bash
ros2 launch gps_waypoint_tracker_ros2 gps_waypoint_tracker.launch.py \
  csv_path:="/home/jigu/ros2_ws/src/0730_gps_test/waypoint_csv/0730_waypoint_straight.csv" \
  fix_topic:=/gps/fix \
  enable_plot:=true \
  enable_rviz:=true
```

## 주행 CSV 로그

`gps_waypoint_tracker.launch.py`는 기본적으로 별도
`drive_csv_logger` 프로세스를 함께 실행합니다. 따라서 위 GPS 실행 명령만
사용해도 첫 GPS 메시지를 받은 뒤 다음 폴더에 20 Hz CSV가 자동 생성됩니다.

```text
/home/jigu/ros2_ws/gps_logs/
```

파일 이름은 실행 시각과 프로세스 ID를 포함합니다.

```text
gps_drive_log_YYYYMMDD_HHMMSS_pidNNNN.csv
```

주요 기록 항목:

- 기록 시각과 실행 후 경과 시간
- GPS 메시지 시각, 위도, 경도, 고도, status/service, covariance
- GPS와 동일한 Web Mercator `x/y` 좌표
- 현재 웨이포인트 인덱스
- GPS tracker가 출력한 조향각
- CAN DRIVE/STOP, 사유, 목표 속도, AURIX 조향각과 서보 펄스
- CAN Sequence, Signature, 8바이트 payload
- AURIX 피드백 속도, 조향각, ADC, Sequence와 상태 비트
- 각 입력의 마지막 수신 후 경과 시간

로그 폴더와 주기는 launch 인수로 바꿀 수 있습니다.

```bash
ros2 launch gps_waypoint_tracker_ros2 gps_waypoint_tracker.launch.py \
  csv_path:="/home/jigu/ros2_ws/src/0730_gps_test/waypoint_csv/0730_waypoint_straight.csv" \
  fix_topic:=/gps/fix \
  enable_plot:=false \
  enable_rviz:=true \
  enable_csv_log:=true \
  log_directory:=/home/jigu/ros2_ws/gps_logs \
  log_period_sec:=0.05
```

로그가 필요하지 않은 시험에서는 다음 인수를 추가합니다.

```text
enable_csv_log:=false
```

로거만 별도로 실행할 수도 있습니다.

```bash
ros2 launch gps_waypoint_tracker_ros2 drive_csv_logger.launch.py \
  fix_topic:=/gps/fix \
  log_directory:=/home/jigu/ros2_ws/gps_logs \
  log_period_sec:=0.05
```

정상 종료 시 CSV를 flush하고 닫습니다. 갑작스러운 전원 차단에서도 손실을
줄이기 위해 실행 중에는 1초 간격으로 파일 버퍼를 flush합니다.

## RViz2

`gps_waypoint_tracker.launch.py`는 기본적으로 RViz2를 함께 실행합니다.
파란색은 waypoint 경로, 자홍색은 현재 목표점, 빨간색은 차량 위치,
초록색은 진행 헤딩, 노란색은 조향 방향, 주황색은 실제 GPS 궤적입니다.

- MarkerArray: `/jigu/rviz_markers`
- Path: `/jigu/gps_path`
- Fixed Frame: `map`

## 토픽

- 입력: `/gps/fix` 또는 `fix_topic`으로 지정한 `sensor_msgs/msg/NavSatFix`
- 출력: `/filtered_steering_angle1` (`std_msgs/msg/Float32`)
- 출력: `/current_waypoint_index1` (`std_msgs/msg/Int32`)

원본 코드의 기본 CSV 경로는 현재 존재하지 않으므로 실행할 때 실제 CSV
절대 경로를 반드시 지정해야 합니다.

## AURIX TC275 CAN 계약

`AURIX_ACKERMANN_TC275 (12).Zip`의 최신 `Can_SpeedTx.c/.h` 기준입니다.

- CAN ID: `0x300`
- Standard CAN 11-bit, DLC 8, little-endian
- Bitrate: `500000`
- 송신 주기: `0.05 s` (20 Hz)
- AURIX 명령 watchdog: `250 ms`
- `+` 조향각은 왼쪽, `-` 조향각은 오른쪽

| 바이트 | 값 |
|---|---|
| 0–1 | 속도 `uint16`, km/h × 100 |
| 2–3 | 조향 `int16`, degree × 10 |
| 4–5 | 조향 펄스 `uint16`, µs |
| 6 | 매 프레임 증가하는 Sequence |
| 7 | `0xA0` STOP, `0xA1` DRIVE |

GPS tracker는 기존 코드의 부호 반전을 유지하므로 CAN bridge의
`invert_steering` 기본값은 `true`입니다. 이 설정으로 AURIX 규약의
`+LEFT/-RIGHT`로 변환됩니다.

## CAN dry-run

실제 CAN 장치를 열지 않고 20 Hz 프레임을 확인합니다.

```bash
ros2 launch gps_waypoint_tracker_ros2 aurix_can_bridge.launch.py \
  dry_run:=true \
  enable_drive:=false
```

원시 8바이트 페이로드 확인:

```bash
ros2 topic echo /aurix/can_tx_payload
ros2 topic echo /aurix/can_tx_status
```

기본 상태는 STOP이며 다음 페이로드 형태입니다.

```text
00 00 00 00 7c 06 SS a0
```

`SS`는 20 Hz마다 증가하는 Sequence입니다.

## SocketCAN 준비

현재 시스템에는 `can0`가 없으므로 실제 연결 전에 PCAN/SocketCAN 장치를
연결해야 합니다.

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0
```

실제 CAN을 열되 계속 STOP만 보내는 단계:

```bash
ros2 launch gps_waypoint_tracker_ros2 aurix_can_bridge.launch.py \
  can_interface:=can0 \
  dry_run:=false \
  enable_drive:=false \
  target_speed_kph:=0.0
```

AURIX의 `0x130` 피드백은 다음 ROS2 토픽으로 변환됩니다.

- `/aurix/feedback/speed_kph`
- `/aurix/feedback/steering_angle_deg`
- `/aurix/feedback/steering_adc_raw`
- `/aurix/feedback/status`

실제 DRIVE는 차량을 띄우거나 구동륜을 지면에서 분리한 상태에서 STOP과
조향 부호를 검증한 다음에만 `enable_drive:=true`로 변경해야 합니다.
