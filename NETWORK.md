# NETWORK.md

## 1. Purpose

Jetson/DCU 관리망과 센서망에서 확인된 값과 미확정 값을 구분한다.

기준일: 2026-10-06

## 2. Confirmed Records

| Context | Host/DCU | Camera | Status |
| --- | --- | --- | --- |
| 2026-06-29 단일 ROS2 검증 | `eno2`, `192.168.200.40/24` | `192.168.200.31`, MX ID `1944301001E1761300` | 검증 기록 존재 |
| 6-camera Web UI 관리 접속 | DCU `192.168.201.4` | 6대 MX ID 검색 방식 | 운영 문서에 기록 |

`192.168.201.4`는 DCU 관리 주소이다. 이 값만으로 카메라 6대의 센서망 주소를
추론하지 않는다.

## 3. Open Network Items

| 항목 | 상태 |
| --- | --- |
| 6대 카메라 개별 IP/subnet | 미정 |
| Camera NIC와 관리 NIC의 분리 구성 | 현장 확인 필요 |
| Livox IP/port | 미정 |
| GNSS/IMU 연결 방식 | 미정 |
| PoE switch 모델과 uplink 용량 | 미정 |
| PTP/NTP/ROS time 기준 | 미정 |
| VLAN, route, firewall | 미정 |

## 4. Verification Commands

```bash
ip addr
ip route
ping <sensor_ip>
ethtool <nic_name>
```

6대 장비는 실행 전 `dai.Device.getAllAvailableDevices()`의 MX ID를
`SENSOR_CONFIGURATION.md`와 대조한다.
