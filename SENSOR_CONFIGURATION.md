# SENSOR_CONFIGURATION.md

## 1. Purpose

센서 모델, 수량, 장비 식별자와 설정 확정 상태를 관리한다.

기준일: 2026-10-06

## 2. Sensor Inventory

| Sensor | Model | Quantity | Interface | Configuration Status |
| --- | --- | ---: | --- | --- |
| Camera | OAK-D Pro PoE | 6 | Ethernet/PoE | 6대 MX ID 매핑 확인, 전체 IP/calibration 미확정 |
| LiDAR | Livox Mid-360S | 1 | Ethernet | 미작성 |
| GNSS | Septentrio Mosaic-go | 1 | 현장 확인 필요 | 미작성 |
| IMU | Xsens MTi-630 | 1 | 현장 확인 필요 | 미작성 |

## 3. Camera Identity

`tools/yolop_6cam_recorder`에서 사용하는 고정 매핑이다.

| Tool ID | MX ID | Logical Role |
| --- | --- | --- |
| `cam_01` | `1944301001E1761300` | 전방 중앙 |
| `cam_02` | `19443010E131771300` | 좌측 전방 장착, 좌측 후면 방향 |
| `cam_03` | `194430105130731300` | 우측 전방 장착, 우측 후면 방향 |
| `cam_04` | `19443010517C731300` | 좌측 후방 장착, 좌측 전면 방향 |
| `cam_05` | `194430105111771300` | 우측 후방 장착, 우측 전면 방향 |
| `cam_06` | `1944301071DA761300` | 후방 중앙 |

역할은 `docs/camera_use_case.md`를 따른다. 실제 장착 상태는 현장에서 MX ID와
물리 위치를 다시 대조해야 한다.

## 4. Confirmed Single-Camera Values

| 항목 | 값 | 근거 |
| --- | --- | --- |
| Host interface | `eno2` | 2026-06-29 검증 로그 |
| Host IP | `192.168.200.40/24` | 2026-06-29 검증 로그 |
| Camera IP | `192.168.200.31` | 2026-06-29 검증 로그 |
| Camera MX ID | `1944301001E1761300` | 2026-06-29 검증 로그 |
| Device type | `OAK-D-PRO-POE` | 2026-06-29 검증 로그 |
| YOLOv6n blob | `yolov6n_coco_416x416_openvino_2022.1_6shave.blob` | repository |
| YOLOP blob | `yolop_bdd100k_320x320_openvino_2022.1_6shave.blob` | repository |

단일 카메라 IP를 나머지 5대에 적용해서는 안 된다.

## 5. Missing Configuration

- 6대 개별 IP/subnet
- resolution/FPS/exposure 운영 기준
- intrinsics와 distortion
- 실측 extrinsics
- namespace/topic/frame mapping
- 시간 동기화 및 QoS
- LiDAR/GNSS/IMU 연결과 driver 설정

향후 공통 default와 차량별 override는 `src/hl_sensor_config/` 계층으로 분리한다.
