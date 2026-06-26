# NETWORK.md

## 1. Purpose

이 문서는 Jetson Orin NX와 센서 간 네트워크 구성을 기록한다.

현재 저장소에는 센서 IP, NIC, switch, VLAN, PTP 설정 정보가 없다. 따라서 실제
네트워크 값은 현장 확인 후 기록해야 한다.

## 2. Target Devices

| 장비 | 수량 | 네트워크 필요 여부 | 상태 |
| --- | ---: | --- | --- |
| OAK-D Pro PoE | 6 | 필요 | IP 미정 |
| Livox Mid-360S | 1 | 필요 | IP 미정 |
| Septentrio Mosaic-go | 1 | 필요 가능 | 연결 방식 미정 |
| Xsens MTi-630 | 1 | 연결 방식 확인 필요 | USB/Ethernet 미정 |

## 3. Required Information

| 항목 | 상태 |
| --- | --- |
| Jetson NIC 이름 | 미정 |
| 센서별 IP 주소 | 미정 |
| Subnet mask | 미정 |
| Gateway | 미정 |
| PoE switch 모델 | 미정 |
| PTP/NTP 사용 여부 | 미정 |
| 방화벽 정책 | 미정 |

## 4. Verification Commands

현장 환경에서 다음 명령으로 네트워크 상태를 검증한다.

```bash
ip addr
ip route
ping <sensor_ip>
ethtool <nic_name>
```

센서 driver 실행 전 네트워크 연결성과 대역폭을 먼저 확인해야 한다.
