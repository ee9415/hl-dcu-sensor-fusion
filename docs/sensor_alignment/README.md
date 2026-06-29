# Sensor Alignment Documentation

자율주행 차량 센서 정렬 및 카메라 브라켓 설계 문서 모음.

## 목적

연구용 자율주행 차량(한라대 DCU 플랫폼)에서 센서를 반복 조립 후에도 동일한 위치를 재현하고,
Calibration 오차를 최소화하며, Sensor Fusion 정확도를 향상시키기 위한 기계적 정렬 시스템을 정의한다.

## 문서 구성

| 파일 | 내용 |
|------|------|
| [01_alignment_overview.md](01_alignment_overview.md) | Sensor Alignment 개요 및 설계 철학 |
| [02_mechanical_design.md](02_mechanical_design.md) | Mechanical Alignment 상세 설계 |
| [03_camera_bracket.md](03_camera_bracket.md) | Camera Bracket 설계안 (OAK-D Pro PoE × 6) |
| [04_calibration_procedure.md](04_calibration_procedure.md) | Calibration 절차 (기계 정렬 → Extrinsic 계산 → TF 반영) |
| [05_software_calibration.md](05_software_calibration.md) | Software Calibration 적용 방법 및 허용 오차 |
| [06_research_vehicle_structure.md](06_research_vehicle_structure.md) | 연구용 차량 권장 구조 |
| [07_vs_production.md](07_vs_production.md) | 양산차 대비 연구용 차량의 설계 차이점 |
| [08_future_work.md](08_future_work.md) | 향후 개선 방향 |

## 대상 센서

- OAK-D Pro PoE Camera × 6
- Livox Mid-360 LiDAR × 1
- GNSS × 1
- IMU × 1

## 시스템 환경

- ROS2 Humble
- TF 기준 링크: `sensor_mount_link`
