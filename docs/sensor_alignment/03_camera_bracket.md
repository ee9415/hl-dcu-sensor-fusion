# 3. Camera Bracket 설계 요구사항

## 3.1 Status

이 문서는 신규 6카메라 배치에 필요한 브라켓 요구사항을 정의한다. CAD 도면,
실제 치수, 장착 각도와 체결 방식은 아직 확정되지 않았다. 과거 전·후방 카메라를
근거리/원거리 쌍으로 수직 배치하는 안은 현행 기준에서 제외한다.

## 3.2 Camera-Specific Requirements

| Camera | Mount Region | Required View | Bracket Requirement |
| --- | --- | --- | --- |
| CAM01 | 전방 중앙 | 전방 | 차량 중심선 기준 yaw 조정, 차선/신호 시야 확보 |
| CAM02 | 좌측 전방 | 좌측 후면 | 차체 좌측 사각지대 방향 조정, 케이블 간섭 방지 |
| CAM03 | 우측 전방 | 우측 후면 | CAM02 대칭 요구조건, 실제 차체 비대칭 확인 |
| CAM04 | 좌측 후방 | 좌측 전면 | CAM02와 시야 보완 영역 형성, 돌출 최소화 |
| CAM05 | 우측 후방 | 우측 전면 | CAM03과 시야 보완 영역 형성, 돌출 최소화 |
| CAM06 | 후방 중앙 | 후방 | 차량 중심선 기준 yaw 조정, 후진 근거리 시야 확보 |

## 3.3 Common Mechanical Requirements

- OAK-D Pro PoE 체결부와 커넥터 간섭이 없어야 한다.
- 각 카메라는 roll/pitch/yaw 미세조정 후 잠글 수 있어야 한다.
- 기준면 또는 위치결정 구조로 탈착 후 재현성을 확보한다.
- PoE 케이블의 굽힘 반경과 strain relief를 확보한다.
- 차체 외곽 돌출과 보행자 접촉 위험을 최소화한다.
- 방수, 부식, 진동, 열팽창 요구조건은 실차 운용 환경으로 검증한다.

## 3.4 Symmetry and Part Reuse

- CAM02/CAM03과 CAM04/CAM05는 좌우 대칭 부품 후보이다.
- 실제 차체 장착면, 배선, 문/패널 가동 범위가 다르면 완전 공용화를 강제하지 않는다.
- CAM01과 CAM06은 중앙 장착 구조를 공용화할 수 있는지 CAD 단계에서 검토한다.

## 3.5 Values Requiring Field Confirmation

| Item | Status |
| --- | --- |
| Mount coordinates | 미정 |
| Initial roll/pitch/yaw | 미정 |
| Camera overlap/FOV | 실차 검증 필요 |
| Plate thickness/material | 구조 검토 필요 |
| Fastener size/torque | 설계 승인 필요 |
| Natural frequency/vibration | 시험 필요 |
| Weather protection | 시험 필요 |

## 3.6 Acceptance Evidence

- CAD와 제작 도면 revision
- 장비별 MX ID/위치 사진
- 장착 좌표 실측 기록
- 체결 토크 기록
- 정차/주행 진동 전후 각도 비교
- 6개 영상의 사각지대와 중첩 영역 확인
- calibration 및 reprojection 결과
