# OPERATION_MANUAL.md

## 1. Purpose

현재 실행 가능한 단일 카메라 ROS2 경로와 6카메라 비-ROS2 경로를 구분한다.

기준일: 2026-10-06

## 2. Pre-Operation Checklist

- 대상 실행 경로가 단일 ROS2인지 6-camera tool인지 확인
- 카메라/PoE switch/DCU 전원과 링크 확인
- 장비 IP 또는 MX ID 확인
- 저장 공간 확인
- 동일 OAK 장치를 점유하는 기존 프로세스 종료

## 3. Single-Camera ROS2

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# YOLOv6n 객체 검출
ros2 run hl_camera_bringup yolov6n_node

# 또는 YOLOP 차량/주행영역/차선 추론
ros2 run hl_camera_bringup yolop_node
```

두 노드는 동일 카메라와 `/oak/*` topic을 사용하므로 하나만 실행한다.

확인:

```bash
ros2 node list
ros2 topic list
ros2 topic hz /oak/rgb/image_raw
ros2 topic echo /oak/detections --once
```

## 4. Six-Camera Recorder/Web UI

이 경로는 ROS2가 아니다. 상세 설치와 옵션은
`tools/yolop_6cam_recorder/README.md`를 따른다.

```bash
cd tools/yolop_6cam_recorder

# 영상만 녹화
./run_yolop_6cam_recorder.sh --record-only --output recordings

# Web UI
./run_yolop_6cam_web_ui.sh --host 127.0.0.1 --port 8080 --output recordings
```

recorder와 Web UI는 같은 장비를 동시에 점유할 수 없으므로 병행 실행하지 않는다.

## 5. Not Yet Available

- 6대 카메라 ROS2 launch
- LiDAR/GNSS/IMU 개별 launch
- 4종 통합 launch
- TF/URDF와 RViz 통합 실행

문서에 있는 미래형 `ros2 launch` 명령은 구현 완료 전 운영 명령으로 사용하지 않는다.

## 6. Shutdown and Failure Handling

- Ctrl+C 또는 관리 script의 `stop`으로 정상 종료한다.
- 녹화 중 `kill -9`와 전원 차단은 파일 flush를 보장하지 않는다.
- topic 없음: node, 장비 점유, network, namespace를 확인한다.
- Hz 저하: network, 저장장치, CPU/VPU 부하와 queue를 확인한다.
- TF 없음: 현재는 정상적인 미구현 상태이며 TF publisher 구현이 필요하다.
