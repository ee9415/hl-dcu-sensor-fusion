# Camera Placement and Use Cases

## 1. Purpose

이 문서는 한라대학교 저속 자율주행 차량 프로젝트의 6대 카메라 배치, 카메라별 역할, 주요 Use Case를 정의한다.

대상 차량은 교내 캠퍼스 환경에서 운행하는 특수목적 저속 자율주행 차량이며, 문서의 목적은 ROS2 기반 센서 bringup, AI 인식 노드, 판단 로직 개발자가 같은 기준으로 카메라 데이터를 설계하고 검증할 수 있게 하는 것이다.

기존 전방/후방 근거리·원거리 분리 배치는 사용하지 않는다. 본 문서의 신규 6-camera 배치를 기준으로 개발한다.

## 2. Design Scope

본 프로젝트의 카메라 시스템은 고속도로 수준의 장거리 인식이나 복잡한 도시 주행 전체 자동화를 목표로 하지 않는다. 현실적인 1차 개발 범위는 다음과 같다.

- 캠퍼스 내부 저속 주행 구간에서 차선, 정지선, 신호등, 보행자, 자전거, 차량, 오토바이, 근거리 장애물을 인식한다.
- CAM01을 자율주행 판단의 핵심 입력으로 사용한다.
- CAM02~CAM05는 측면 안전 감시와 사각지대 보완에 사용한다.
- CAM06은 후방 안전 확인, 후진, 정차 전후 위험 확인에 사용한다.
- 각 카메라는 공통 객체 인식 기능을 공유할 수 있지만, 모든 카메라가 동일한 판단 책임을 갖지는 않는다.

## 3. Camera Placement

| Camera ID | Installation Position | View Direction | Primary Role |
| --- | --- | --- | --- |
| CAM01 | 전방 중앙 | 전방 | 자율주행 판단용 핵심 카메라. 차선, 정지선, 신호등, 전방 객체 감지 |
| CAM02 | 좌측 전방 | 좌측 후면 방향 | 좌측 측면 및 좌후방 사각지대 감시. 접근 객체, 보행자, 자전거, 차량, 오토바이, 측면 장애물 감지 |
| CAM03 | 우측 전방 | 우측 후면 방향 | 우측 측면 및 우후방 사각지대 감시. 접근 객체, 보행자, 자전거, 차량, 오토바이, 측면 장애물 감지 |
| CAM04 | 좌측 후방 | 좌측 전방 방향 | 좌측 측면 및 좌전방 보완 감시. 연석, 벽, 기둥, 측면 장애물, 근접 보행자 감지 |
| CAM05 | 우측 후방 | 우측 전방 방향 | 우측 측면 및 우전방 보완 감시. 연석, 벽, 기둥, 측면 장애물, 근접 보행자 감지 |
| CAM06 | 후방 중앙 | 후방 | 후방 안전 감시. 후방 객체 감지, 후진 및 정차 시 근거리 장애물 확인 |

## 4. Role Model

### 4.1 Common Role

6대 카메라는 공통적으로 다음 기능을 수행할 수 있다.

| Function | Description | Notes |
| --- | --- | --- |
| 객체 감지 | 보행자, 자전거, 차량, 오토바이, 콘, 박스, 쓰레기통 등 감지 | YOLO 계열 모델 적용 가능 |
| 위험 영역 판단 | 차량 주변의 접근 객체 또는 근거리 장애물 확인 | 카메라별 ROI 설정 필요 |
| timestamp 동기화 | 이미지, detection 결과, TF 기준 시간 정렬 | sensor fusion 단계에서 필수 |
| 상태 진단 | image topic publish, FPS, latency, dropped frame 확인 | 운영 진단 노드와 연계 |

공통 역할은 카메라별 입력 포맷과 detection message 형식을 통일하기 위한 기준이다. 그러나 최종 판단 로직에서 각 카메라의 중요도와 책임은 위치별로 다르게 설정한다.

### 4.2 Position-Specific Role

| Camera Group | Cameras | Specialized Role | Development Focus |
| --- | --- | --- | --- |
| 전방 판단 | CAM01 | 차선 추종, 정지선, 신호등, 전방 객체 감지 | 자율주행 판단 로직의 핵심 입력 |
| 측면 안전 | CAM02, CAM03, CAM04, CAM05 | 좌우 측면 객체, 보행자, 자전거, 차량, 오토바이, 연석, 벽, 기둥, 측면 장애물 감지 | 사각지대 보완, 차체 측면 접근 위험 판단 |
| 후방 안전 | CAM06 | 후방 객체, 후진 중 장애물, 정차 전후 위험 확인 | 후진 및 저속 정차 안전 확인 |

## 5. Recommended ROS2 Interface

실제 패키지 구현 시 아래와 같은 네이밍을 기준안으로 사용할 수 있다. 최종 이름은 `TOPIC_LIST.md`, `TF_TREE.md`, launch 파일과 함께 확정한다.

### 5.1 Frame Naming

| Camera ID | Recommended Optical Frame | Parent Frame Candidate |
| --- | --- | --- |
| CAM01 | `cam01_front_center_optical_frame` | `base_link` |
| CAM02 | `cam02_front_left_rear_view_optical_frame` | `base_link` |
| CAM03 | `cam03_front_right_rear_view_optical_frame` | `base_link` |
| CAM04 | `cam04_rear_left_front_view_optical_frame` | `base_link` |
| CAM05 | `cam05_rear_right_front_view_optical_frame` | `base_link` |
| CAM06 | `cam06_rear_center_optical_frame` | `base_link` |

### 5.2 Topic Naming

| Data | Topic Pattern | Example |
| --- | --- | --- |
| Raw image | `/camera/{cam_id}/image_raw` | `/camera/cam01/image_raw` |
| Camera info | `/camera/{cam_id}/camera_info` | `/camera/cam01/camera_info` |
| AI detection | `/perception/{cam_id}/detections` | `/perception/cam01/detections` |
| Debug image | `/perception/{cam_id}/debug_image` | `/perception/cam01/debug_image` |
| Camera health | `/diagnostics/camera/{cam_id}` | `/diagnostics/camera/cam01` |

## 6. Use Cases

### UC-01. 차선 추종 자율주행

| Item | Description |
| --- | --- |
| 목적 | 캠퍼스 내부 주행로에서 차량이 차선 중앙을 유지하며 저속으로 주행한다. |
| 사용 카메라 | CAM01 중심. 필요 시 CAM02, CAM03으로 측면 접근 위험 보조 확인 |
| 필요한 인식 항목 | 차선, 주행 가능 영역, 전방 차량, 보행자, 자전거, 콘, 박스 등 전방 장애물 |
| 주요 기능 | 차선 중앙 유지, 곡선 구간 주행, 속도 유지, 장애물 감속 |

개발 기준:

- CAM01 image를 기준으로 lane detection 또는 drivable area detection을 수행한다.
- 차선이 불명확한 구간에서는 전방 객체와 주행 가능 영역을 우선 사용한다.
- 저속 차량이므로 급격한 회피보다 감속, 정지, operator 개입 가능 상태를 우선한다.
- 전방 장애물이 일정 거리 이내로 들어오면 planning/control 쪽에 감속 또는 정지 후보를 제공한다.

### UC-02. 교차로 및 횡단보도 통과

| Item | Description |
| --- | --- |
| 목적 | 캠퍼스 교차로, 횡단보도, 보행자 통행 구간에서 정지와 재출발을 안전하게 수행한다. |
| 사용 카메라 | CAM01 중심. CAM02~CAM05는 좌우 접근 객체 확인에 사용 |
| 필요한 인식 항목 | 정지선, 횡단보도, 신호등, 보행자, 자전거, 좌우 접근 차량 또는 오토바이 |
| 주요 기능 | 정지선 감지, 보행자 확인, 신호등 확인, 정지 후 재출발 |

개발 기준:

- CAM01에서 정지선, 횡단보도, 신호등을 우선 인식한다.
- 신호등 인식은 캠퍼스 내 실제 신호기 설치 여부와 카메라 화각을 기준으로 적용 범위를 제한한다.
- CAM02~CAM05는 교차로 진입 전 좌우 측면 접근 객체를 확인하는 보조 입력으로 사용한다.
- 재출발 조건은 전방 진행 가능, 보행자 없음, 좌우 접근 위험 낮음 상태를 모두 만족할 때로 정의한다.

### UC-03. 정적 장애물 회피

| Item | Description |
| --- | --- |
| 목적 | 주행 경로 상의 박스, 콘, 쓰레기통, 공사 구조물 등 정적 장애물을 감지하고 저속으로 회피한다. |
| 사용 카메라 | CAM01 중심. CAM02~CAM05는 회피 중 측면 여유 공간 확인에 사용 |
| 필요한 인식 항목 | 박스, 콘, 쓰레기통, 공사 구조물, 벽, 기둥, 연석, 주차 차량 |
| 주요 기능 | 장애물 감지, 감속, 회피, 원래 주행 경로 복귀 |

개발 기준:

- CAM01에서 전방 장애물 위치와 크기를 추정하고, LiDAR가 사용 가능한 경우 거리 판단은 LiDAR와 함께 검증한다.
- CAM02~CAM05는 회피 경로가 차량 측면 장애물과 충돌하지 않는지 확인하는 안전 보조 역할을 한다.
- 초기 개발에서는 복잡한 동적 회피보다 정지 또는 단순 저속 우회 시나리오를 우선 구현한다.
- 장애물 class가 불명확하더라도 주행 가능 영역을 침범하면 위험 객체로 처리한다.

### UC-04. 측면 차량 및 보행자 감시

| Item | Description |
| --- | --- |
| 목적 | 차량 좌우 측면과 사각지대에서 접근하는 보행자, 자전거, 차량, 오토바이를 감시한다. |
| 사용 카메라 | CAM02, CAM03, CAM04, CAM05 중심 |
| 필요한 인식 항목 | 좌측/우측 접근 보행자, 자전거, 차량, 오토바이, 연석, 벽, 기둥, 측면 장애물 |
| 주요 기능 | 좌우 접근 객체 감지, 측면 사각지대 보완, 위험 객체 경고, 회피 판단 지원 |

개발 기준:

- CAM02와 CAM04는 좌측 안전 영역을, CAM03과 CAM05는 우측 안전 영역을 담당한다.
- 각 측면 카메라는 차량 진행 방향 기준의 위험 ROI를 별도로 가진다.
- 측면 객체는 단독으로 조향 판단을 내리기보다 감속, 정지, 회피 가능 여부 판단을 지원한다.
- 좁은 캠퍼스 도로, 보행자 혼재 구간, 주차 차량 옆 통과 상황을 주요 검증 시나리오로 둔다.

### UC-05. 후진 및 저속 정차

| Item | Description |
| --- | --- |
| 목적 | 후진 또는 정차 전후에 차량 후방의 사람, 차량, 장애물을 확인해 저속 안전을 확보한다. |
| 사용 카메라 | CAM06 중심. 필요 시 CAM04, CAM05로 후측방 보조 확인 |
| 필요한 인식 항목 | 후방 보행자, 차량, 자전거, 오토바이, 콘, 박스, 벽, 기둥, 근거리 장애물 |
| 주요 기능 | 후방 객체 감지, 후진 시 안전 확인, 정차 전후 위험 확인 |

개발 기준:

- CAM06은 후진 모드 또는 정차 모드에서 우선순위를 높인다.
- 후방 객체가 근거리 ROI에 들어오면 후진 제한 또는 정지 명령 후보를 제공한다.
- CAM04, CAM05는 후측방에서 접근하는 객체를 보조적으로 확인한다.
- 후진 자동화는 초기 단계에서 제한적으로 적용하고, 안전 경고 및 정지 판단부터 구현한다.

## 7. Per-Camera AI Task Priority

| Camera ID | Lane | Stop Line | Traffic Light | Object Detection | Side Obstacle | Rear Obstacle | Priority |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| CAM01 | High | High | High | High | Low | None | 자율주행 판단 핵심 |
| CAM02 | None | None | None | High | High | Low | 좌측 사각지대 보조 |
| CAM03 | None | None | None | High | High | Low | 우측 사각지대 보조 |
| CAM04 | None | None | None | High | High | Low | 좌측 측면 보완 |
| CAM05 | None | None | None | High | High | Low | 우측 측면 보완 |
| CAM06 | None | None | None | High | Low | High | 후방 안전 핵심 |

## 8. Implementation Notes

- 카메라별 calibration 결과는 intrinsics와 extrinsics를 분리해 관리한다.
- 모든 detection 결과는 header timestamp와 frame_id를 정확히 포함해야 한다.
- 카메라별 AI 모델은 동일 모델을 공유하되, post-processing ROI와 class priority는 다르게 설정할 수 있다.
- CAM01의 lane, stop line, traffic light 인식은 객체 감지와 별도 노드로 분리하는 것이 유지보수에 유리하다.
- CAM02~CAM05는 측면 안전용 ROI, CAM06은 후방 근거리 ROI를 명확히 정의해야 한다.
- 판단 노드는 각 카메라 detection을 동일 가중치로 합치지 말고, 차량 상태와 Use Case에 따라 우선순위를 다르게 적용한다.
- 초기 검증은 rosbag 기반 반복 재생, RViz 시각화, debug image 저장을 함께 사용한다.

## 9. Minimum Verification Checklist

| Category | Check Item |
| --- | --- |
| Bringup | 6대 카메라 image topic이 동시에 publish되는지 확인 |
| Frame | 각 camera optical frame이 `base_link` 기준으로 연결되는지 확인 |
| Calibration | 카메라별 intrinsics, extrinsics 파일 존재 및 로딩 확인 |
| AI Inference | CAM01, CAM02~CAM05, CAM06 detection topic publish 확인 |
| Latency | image input부터 detection output까지 지연 시간 측정 |
| Use Case | UC-01~UC-05별 rosbag 또는 현장 주행 데이터로 결과 확인 |
| Safety | 위험 객체 감지 시 감속/정지 후보가 판단 노드로 전달되는지 확인 |

