# SENSOR_CONFIGURATION.md

## 1. Purpose

이 문서는 차량 탑재 센서의 모델, 수량, 연결 방식, 설정 파일 위치를 관리한다.

현재 저장소에는 센서별 설정 파일이 없다.

## 2. Sensor Inventory

| Sensor | Model | Quantity | Interface | Config Status |
| --- | --- | ---: | --- | --- |
| Camera | OAK-D Pro PoE | 6 | Ethernet/PoE | 미작성 |
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
