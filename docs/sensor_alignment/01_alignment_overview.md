# 1. Sensor Alignment 개요

## 1.1 목적

기계적 정렬과 extrinsic calibration을 통해 센서 재장착 재현성과 sensor fusion
입력의 좌표 일관성을 확보한다. 기계 정렬값은 calibration 초기값일 뿐이며,
실제 TF에는 검증된 calibration 결과를 사용한다.

## 1.2 원칙

1. 기계 오차를 먼저 줄이고 software calibration으로 잔여 오차를 보정한다.
2. Datum과 위치결정 구조를 사용해 반복 장착성을 확보한다.
3. 설계값, 실측값, calibration 결과를 서로 다른 파일과 이력으로 관리한다.
4. frame 이름과 부모 frame은 `TF_TREE.md` 승인 후 구현한다.

## 1.3 Current Six-Camera Layout

`docs/camera_use_case.md`의 신규 배치를 따른다.

| Camera | Installation | View Direction | Primary Role | Proposed Optical Frame |
| --- | --- | --- | --- | --- |
| CAM01 | 전방 중앙 | 전방 | 차선·정지선·신호등·전방 객체 | `cam01_front_center_optical_frame` |
| CAM02 | 좌측 전방 | 좌측 후면 | 좌측 사각지대와 접근 객체 | `cam02_front_left_rear_view_optical_frame` |
| CAM03 | 우측 전방 | 우측 후면 | 우측 사각지대와 접근 객체 | `cam03_front_right_rear_view_optical_frame` |
| CAM04 | 좌측 후방 | 좌측 전면 | 좌측 측면 보완 | `cam04_rear_left_front_view_optical_frame` |
| CAM05 | 우측 후방 | 우측 전면 | 우측 측면 보완 | `cam05_rear_right_front_view_optical_frame` |
| CAM06 | 후방 중앙 | 후방 | 후진과 후방 안전 | `cam06_rear_center_optical_frame` |

위 frame 이름은 후보이며 TF는 아직 구현되지 않았다. 장착 높이, pitch/yaw/roll,
translation은 모두 현장 측정 필요 항목이다.

## 1.4 Other Sensors

| Sensor | Intended Position | Status |
| --- | --- | --- |
| Livox Mid-360S | 360° 시야가 확보되는 상부 위치 후보 | 실장 위치 미정 |
| GNSS antenna | 상공 시야가 확보되는 루프 위치 후보 | 실장 위치 미정 |
| Xsens MTi-630 | 차체 기준축과 정렬 가능한 위치 후보 | 실장 위치 미정 |

공통 부모 frame은 `base_link`, `sensor_mount_link` 등이 후보지만 확정되지 않았다.

## 1.5 Alignment Flow

```text
배치/브라켓 설계
  → 실차 장착 및 장비 ID 대조
  → 위치·각도 실측
  → intrinsic/extrinsic calibration
  → 후보 TF 작성
  → reprojection/TF/RViz 검증
  → 승인된 값 배포 및 이력 저장
```

## 1.6 Required Records

- 장비 ID, 물리 위치, 촬영 방향
- 실측 translation과 roll/pitch/yaw
- intrinsic 파일과 획득 조건
- extrinsic 결과와 품질 지표
- 적용한 TF 파일의 버전
- 검증 날짜, 방법, 결과
