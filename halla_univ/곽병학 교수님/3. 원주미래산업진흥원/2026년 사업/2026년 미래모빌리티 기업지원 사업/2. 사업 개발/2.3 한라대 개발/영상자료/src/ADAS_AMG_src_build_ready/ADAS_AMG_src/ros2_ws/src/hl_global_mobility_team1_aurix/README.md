# HL Global Mobility Team 1 AURIX ROS2 Bridge

This ROS2 package sends the AURIX command/alive CAN frame for the Ackermann vehicle.

- Team display name: `HL Global Mobility Team 1`
- ROS2 package: `hl_global_mobility_team1_aurix`
- CAN interface: `can0`
- CAN bitrate: `500000`
- Alive CAN ID: `0x300`
- Alive period: `50 ms`
- Reconnect retry period: `200 ms`

## Install Dependencies

Ubuntu 22.04 / ROS2 Humble example:

```bash
sudo apt update
sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions can-utils
```

## One-Shot Run

Copy this `ros2_ws` folder to the industrial PC, then run:

```bash
cd ~/ros2_ws
chmod +x src/hl_global_mobility_team1_aurix/scripts/*.sh
./src/hl_global_mobility_team1_aurix/scripts/run_hl_team1_aurix.sh can0
```

The node publishes status here:

```bash
ros2 topic echo /aurix/can_alive_status
```

Raw CAN verification:

```bash
candump can0
```

Expected transmit frame:

```text
can0  300   [8]  00 00 00 00 00 00 00 00
```

## Manual Run

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
./src/hl_global_mobility_team1_aurix/scripts/setup_can0.sh can0 500000 200
colcon build --symlink-install
source install/setup.bash
ros2 launch hl_global_mobility_team1_aurix aurix_can_alive.launch.py interface:=can0
```

## Start Automatically On Boot

```bash
cd ~/ros2_ws
chmod +x src/hl_global_mobility_team1_aurix/scripts/*.sh
./src/hl_global_mobility_team1_aurix/scripts/install_systemd_service.sh can0
sudo systemctl start hl-team1-aurix-can.service
```

Check logs:

```bash
journalctl -u hl-team1-aurix-can.service -f
```

Stop auto-start:

```bash
sudo systemctl disable --now hl-team1-aurix-can.service
```

## Parameters

Edit `config/aurix_can_alive.yaml` if needed.

```yaml
team_name: "HL Global Mobility Team 1"
interface: "can0"
bitrate: 500000
alive_can_id: 768
alive_data: [0, 0, 0, 0, 0, 0, 0, 0]
alive_period_ms: 50.0
reconnect_period_ms: 200.0
```
