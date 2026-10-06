# Sensor Alignment Documentation

자율주행 차량 센서 정렬과 calibration을 위한 설계 자료이다.

기준일: 2026-10-06

## Status

- 6카메라 역할과 논리 배치는 `../camera_use_case.md`를 정본으로 사용한다.
- 본 폴더의 기계 치수, 각도, 허용 오차는 설계 목표이며 실차 확정값이 아니다.
- TF 부모 frame, 장착 좌표, intrinsics/extrinsics는 현장 측정과 calibration 후
  별도 결과 파일 및 검증 로그로 확정해야 한다.
- 과거 전·후방 근거리/원거리 2대씩 배치안은 사용하지 않는다.

## Documents

| 파일 | 내용 |
| --- | --- |
| `01_alignment_overview.md` | 정렬 원칙과 현행 6카메라 배치 |
| `02_mechanical_design.md` | 조정형 브라켓 설계 목표 |
| `03_camera_bracket.md` | 현행 배치 기준 브라켓 요구사항 |
| `04_calibration_procedure.md` | calibration 절차 예시 |
| `05_software_calibration.md` | software 적용과 품질 평가안 |
| `06_research_vehicle_structure.md` | 연구 차량 권장 장착 구조 |
| `07_vs_production.md` | 양산 구조와의 차이 |
| `08_future_work.md` | 향후 작업 |

## Target Sensors

- OAK-D Pro PoE Camera × 6
- Livox Mid-360S LiDAR × 1
- Septentrio Mosaic-go GNSS × 1
- Xsens MTi-630 IMU × 1
