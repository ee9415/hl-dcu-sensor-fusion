# OAK-D Pro PoE Single Camera Test Log

Date: 2026-06-29

## Test Target

- Device: OAK-D Pro PoE
- Quantity: 1
- Host interface: `eno2`
- Host IP: `192.168.200.40/24`
- Camera IP: `192.168.200.31`
- Camera MXID: `1944301001E1761300`
- Detected device type: `OAK-D-PRO-POE`

## Summary

The PoE camera was discovered, connected through `depthai_ros_driver`,
and verified through ROS2 image topics. RGB video was displayed using
`image_view` on `/oak/rgb/image_raw`.

Two configuration files were added:

- `oak_poe_single_test.yaml`: baseline RGBD test configuration
- `oak_poe_rgb_low_latency.yaml`: RGB-only low-latency display configuration

## Verification Results

### Network and Device Discovery

- Ethernet interface `eno2` was up.
- The OAK device was detected at `192.168.200.31`.
- The driver connected successfully using `camera.i_ip`.
- Driver log confirmed:
  - `Camera with MXID: 1944301001E1761300 and Name: 192.168.200.31 connected`
  - `Device type: OAK-D-PRO-POE`
  - `Camera ready!`

### Baseline RGBD Test

Configuration:

- File: `oak_poe_single_test.yaml`
- Pipeline: `RGBD`
- IMU: enabled
- Low bandwidth: enabled for RGB, left, right, stereo

Observed topics:

- `/oak/rgb/image_raw`
- `/oak/rgb/camera_info`
- `/oak/stereo/image_raw`
- `/oak/stereo/camera_info`
- `/oak/imu/data`
- `/tf`
- `/tf_static`

Measured publish rates:

- `/oak/rgb/image_raw`: about `19.5 Hz`
- `/oak/stereo/image_raw`: about `24.7 Hz`

Camera info:

- Resolution: `1280 x 720`
- Frame ID: `oak_rgb_camera_optical_frame`

### Video Display Test

Display command:

```bash
env ROS_LOG_DIR=/tmp/ros2-log ros2 run image_view image_view --ros-args -r image:=/oak/rgb/image_raw
```

Result:

- RGB video was visible.
- `image_view` was preferred over `rqt_image_view` because it subscribes to the
  raw image topic directly and avoids compressed/depth transport errors.

### Low-Latency Tuning

Configuration:

- File: `oak_poe_rgb_low_latency.yaml`
- Pipeline: `RGB`
- IMU: disabled
- ISP scale: `1920x1080` sensor input scaled to `1280x720`
- Low bandwidth: enabled
- RGB queue size: `1`
- Raw transport only

Purpose:

- Remove unused depth/stereo/IMU processing for video display.
- Reduce queued stale frames by setting `rgb.i_max_q_size: 1`.
- Keep PoE bandwidth manageable with `rgb.i_low_bandwidth: true`.

Measured publish rate:

- `/oak/rgb/image_raw`: about `21 Hz`

Result:

- Video remained visible.
- Latency was reduced compared with the heavier RGBD display path.

## Notes

The camera reported a bootloader warning:

```text
Flashed bootloader version 0.0.21, less than 0.0.28 is susceptible to bootup/restart failure.
Upgrading is advised, flashing main/factory (not user) bootloader. Available: 0.0.28
```

The camera can take extra time to reconnect after restarting the driver.
Bootloader update should be considered before field operation.

## Recommended Next Steps

- Keep `oak_poe_single_test.yaml` as the baseline functional test.
- Use `oak_poe_rgb_low_latency.yaml` when visual display latency matters.
- Add a formal `hl_camera_bringup` launch file after the package structure is created.
- Record future camera mounting position, frame name, and calibration data.
