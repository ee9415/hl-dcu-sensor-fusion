# TF_TREE.md

## 1. Purpose

구현된 frame 사용과 목표 TF 구조를 구분한다.

기준일: 2026-10-06

## 2. Current Status

- TF publisher, URDF, static transform launch는 없다.
- `yolov6n_node`와 `yolop_node`는 message header에 기본
  `oak_rgb_camera_optical_frame` 문자열을 넣는다.
- frame_id가 존재하는 것과 해당 frame이 TF tree에 연결된 것은 다르다.

| 항목 | 상태 |
| --- | --- |
| Vehicle base frame | 미정 |
| Common sensor mount frame | 후보: `sensor_mount_link`, 미확정 |
| Single-camera message frame | `oak_rgb_camera_optical_frame`, TF 미연결 |
| Six-camera optical frames | 이름 후보 정의, TF 미구현 |
| LiDAR/GNSS/IMU frames | 미정 |

## 3. Six-Camera Frame Candidates

| Camera | Proposed Optical Frame | Status |
| --- | --- | --- |
| CAM01 | `cam01_front_center_optical_frame` | 후보 |
| CAM02 | `cam02_front_left_rear_view_optical_frame` | 후보 |
| CAM03 | `cam03_front_right_rear_view_optical_frame` | 후보 |
| CAM04 | `cam04_rear_left_front_view_optical_frame` | 후보 |
| CAM05 | `cam05_rear_right_front_view_optical_frame` | 후보 |
| CAM06 | `cam06_rear_center_optical_frame` | 후보 |

장착 역할은 `docs/camera_use_case.md`를 따른다. 부모 frame과 모든 translation/
rotation 값은 현장 측정 및 calibration 전까지 확정하지 않는다.

## 4. Required Decisions

- 차량 기준 frame (`base_link` 등)
- 공통 sensor mount frame 사용 여부
- 각 센서 body frame과 optical frame의 관계
- GNSS antenna/IMU/LiDAR frame
- static TF publish 주체와 calibration YAML schema

## 5. Verification

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo <parent_frame> <child_frame>
```

RViz에서 모든 센서 데이터가 동일한 Fixed Frame에 연결되는지 검증해야 한다.
