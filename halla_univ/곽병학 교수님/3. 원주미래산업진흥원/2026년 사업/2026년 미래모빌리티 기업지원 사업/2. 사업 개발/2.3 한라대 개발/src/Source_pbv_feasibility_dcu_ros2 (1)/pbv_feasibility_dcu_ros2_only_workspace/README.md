# PBV Feasibility DCU — ROS2-only workspace

이 워크스페이스는 업로드된 PBV feasibility 패키지를 **ROS 2 topic 기반 내부 통신**으로 정리한 버전입니다.

핵심 변경점은 세 가지입니다.

- `perception_camera_py`는 그대로 유지하되 **UDP bridge를 제거**했습니다.
- Fault / State / Safety / Decision / CAN 로직은 기존 **C 디렉토리 구조를 유지한 정적 라이브러리**로 묶었습니다.
- ROS 2에서 바로 쓰기 쉽도록 얇은 **pipeline adapter node**를 추가했습니다. 이 노드는 `/pbv/perception/snapshot`을 받아 C 로직을 호출하고 결과를 다시 ROS 2 topic으로 publish합니다.

## 디렉토리

```text
pbv_feasibility_dcu_ros2_only_workspace/
├─ src/
│  ├─ pbv_dcu_interfaces/
│  ├─ perception_camera_py/
│  └─ pbv_dcu_core_ros2/
├─ sample_data/
├─ scripts/
└─ tools/
```

## 내부 토픽 흐름

```text
camera_perception_node (Python)
  └─ /pbv/perception/snapshot
        ↓
pbv_dcu_pipeline_node (ROS 2 adapter + C core)
  ├─ /pbv/fault/status
  ├─ /pbv/state/status
  ├─ /pbv/safety/status
  ├─ /pbv/control/command
  └─ /pbv/can/tx
```

## 빌드

```bash
cd pbv_feasibility_dcu_ros2_only_workspace
colcon build --symlink-install
source install/setup.bash
```

## 실행

실카메라가 있으면:

```bash
ros2 launch pbv_dcu_core_ros2 pbv_dcu_ros2_only.launch.py
```

카메라 없이 synthetic frame으로 바로 보고 싶으면:

```bash
ros2 launch pbv_dcu_core_ros2 pbv_dcu_ros2_only.launch.py simulate_no_camera:=true
```

가상 CAN이 필요하면:

```bash
./scripts/setup_vcan.sh
ros2 launch pbv_dcu_core_ros2 pbv_dcu_ros2_only.launch.py simulate_no_camera:=true use_socketcan:=true can_if:=vcan0
```

CSV replay로 perception만 흘려보고 싶으면:

```bash
python3 tools/replay_perception_ros2.py --csv sample_data/perception_replay.csv --fps 5
ros2 run pbv_dcu_core_ros2 pbv_dcu_pipeline_node --ros-args -p config_path:=$(pwd)/src/pbv_dcu_core_ros2/config/dcu_runtime.conf
```

## 참고

- `udp_rx.c`, `perception_packet.c`, `packet_bridge.py`는 제거했습니다.
- 원본 C 알고리즘 파일들은 `pbv_dcu_core_ros2/src/*` 아래에 그대로 유지했습니다.
- 이 환경에서는 ROS 2가 설치되어 있지 않아 `colcon build`까지 실제 검증하지는 못했습니다. 대신 디렉토리와 빌드 스크립트는 일반적인 `ament_cmake + rclcpp`, `ament_python` 워크스페이스 형태로 맞췄습니다.
