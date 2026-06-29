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

## YOLOv6n VPU Inference Test

Date: 2026-06-29

### Purpose

Verify that on-device Myriad X VPU inference runs correctly on OAK-D Pro PoE
using a Luxonis pre-converted YOLOv6n (COCO) blob.

### Environment

- depthai: `2.30.0.0`
- blobconverter: installed (Luxonis depthai zoo accessible)
- Model: `yolov6n_coco_416x416_openvino_2022.1_6shave.blob`
- Source: Luxonis depthai model zoo (`zoo_type="depthai"`)
- Shaves: 6
- Input: 416×416 BGR
- Classes: 80 (COCO), anchor-free

### Model Parameters

```
num_classes:         80
coordinates:         4
anchors:             []
anchor_masks:        {}
iou_threshold:       0.5
confidence_threshold: 0.5
```

### Terminal Inference Test

Script: `src/hl_camera_bringup/scripts/yolov6n_infer_test.py`

- Frames tested: 100
- Total elapsed: 3.4 s
- Average FPS: **29.8**
- Sample detection: `chair` 0.91–0.92 confidence

### Live Display Test

Script: `src/hl_camera_bringup/scripts/yolov6n_display.py`

- Bounding boxes rendered on RGB frame via OpenCV (`cv2.imshow`)
- FPS overlay displayed on frame
- Display confirmed on `DISPLAY=:0`
- Window size: 832×832 (2× input resolution)

### Result

- VPU inference: **confirmed**
- On-device YOLOv6n detection: **working**
- Live display with bounding boxes: **confirmed**

## ROS2 Topic Publishing Test

Date: 2026-06-29

### Purpose

Verify that YOLOv6n inference results are published as ROS2 topics and
correctly received by a subscriber.

### Node

Package: `hl_camera_bringup`
Node: `yolov6n_node`
Entry point: `ros2 run hl_camera_bringup yolov6n_node`

### Published Topics

| Topic | Type | Rate |
| --- | --- | --- |
| `/oak/rgb/image_raw` | `sensor_msgs/Image` | ~30 Hz |
| `/oak/detections` | `vision_msgs/Detection2DArray` | ~30 Hz |

### Verification Commands

```bash
ros2 topic list
ros2 topic hz /oak/rgb/image_raw --window 30
ros2 topic hz /oak/detections --window 30
ros2 topic echo /oak/detections --once
```

### Results

Topic list:

```
/oak/detections
/oak/rgb/image_raw
/parameter_events
/rosout
```

Publish rates:

- `/oak/rgb/image_raw`: average **29.996 Hz**  (min 0.030 s, max 0.036 s)
- `/oak/detections`:   average **29.928 Hz**  (min 0.031 s, max 0.036 s)

Sample detection output (`/oak/detections`):

```
detections:
- results: [{class_id: tvmonitor, score: 0.900}]
  bbox: {center: {x: 116.4, y: 274.0}, size_x: 60.1, size_y: 58.3}
- results: [{class_id: person, score: 0.899}]
  bbox: {center: {x: 171.7, y: 306.7}, size_x: 107.4, size_y: 91.1}
- results: [{class_id: tvmonitor, score: 0.883}]
  bbox: {center: {x: 89.3, y: 344.3}, size_x: 57.0, size_y: 52.4}
- results: [{class_id: laptop, score: 0.794}]
  bbox: {center: {x: 273.9, y: 371.0}, size_x: 196.4, size_y: 88.5}
frame_id: oak_rgb_camera_optical_frame
```

### Result

- ROS2 node 실행: **확인**
- 토픽 발행: **확인** (`/oak/rgb/image_raw`, `/oak/detections`)
- 발행률: **~30 Hz** (안정적)
- 수신 데이터: **정상** (class_id, score, bbox 포함)

## Recommended Next Steps

- Keep `oak_poe_single_test.yaml` as the baseline functional test.
- Use `oak_poe_rgb_low_latency.yaml` when visual display latency matters.
- Add a formal `hl_camera_bringup` launch file after the package structure is created.
- ~~Wrap `yolov6n_display.py` as a ROS2 node publishing detection results.~~ **완료**
- Record future camera mounting position, frame name, and calibration data.
