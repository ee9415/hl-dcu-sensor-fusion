# SENSOR_CONFIGURATION.md

## 1. Purpose

이 문서는 차량 탑재 센서의 모델, 수량, 연결 방식, 설정 파일 위치를 관리한다.

현재 저장소에는 공통 센서 설정 파일은 없다. 다만 OAK-D Pro PoE 단일 카메라
검증에서 사용한 IP, MX ID, 모델 blob 정보는 검증 로그에 기록되어 있다.

## 2. Sensor Inventory

| Sensor | Model | Quantity | Interface | Config Status |
| --- | --- | ---: | --- | --- |
| Camera | OAK-D Pro PoE | 6 | Ethernet/PoE | 단일 카메라 검증값 기록, 6대 설정 미작성 |
| LiDAR | Livox Mid-360S | 1 | Ethernet | 미작성 |
| GNSS | Septentrio Mosaic-go | 1 | 확인 필요 | 미작성 |
| IMU | Xsens MTi-630 | 1 | 확인 필요 | 미작성 |

## 3. Required Configuration

### Camera

- Device IP
- MX ID 또는 device identifier
- Resolution
- FPS
- Exposure/auto exposure
- Intrinsics
- Extrinsics

현재 확인된 단일 카메라 검증값:

| 항목 | 값 | 근거 |
| --- | --- | --- |
| Host interface | `eno2` | `docs/verification/camera/oak_poe_single_test_log.md` |
| Host IP | `192.168.200.40/24` | `docs/verification/camera/oak_poe_single_test_log.md` |
| Camera IP | `192.168.200.31` | `docs/verification/camera/oak_poe_single_test_log.md` |
| Camera MXID | `1944301001E1761300` | `docs/verification/camera/oak_poe_single_test_log.md` |
| Detected device type | `OAK-D-PRO-POE` | `docs/verification/camera/oak_poe_single_test_log.md` |
| YOLOv6n blob | `yolov6n_coco_416x416_openvino_2022.1_6shave.blob` | `src/hl_camera_bringup/blobs/` |

위 값은 단일 장비 검증값이며, 6대 카메라 운영 설정으로 확정된 값은 아니다.

### LiDAR

- Device IP
- Host IP
- Data port
- Command port
- Coordinate mode
- Point cloud topic

### GNSS

- Connection method
- Correction data source
- Output message type
- Time synchronization method
- Antenna position

### IMU

- Connection method
- Output rate
- Orientation convention
- Covariance
- Vehicle mounting orientation

## 4. Configuration Storage Policy

센서별 설정은 향후 다음 위치에 분리하여 관리한다.

```text
src/hl_sensor_config/config/
├── camera/
├── lidar/
├── gnss/
└── imu/
```

실차별 값은 공통 default와 차량별 override를 분리한다.
