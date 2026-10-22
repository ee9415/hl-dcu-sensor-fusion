# 클로드 이전 버전 복원 — 2026-10-23

## 복원 기준

사용자가 클로드 검토 이전으로 복귀를 요청했다. 대상은 `43493f4` / 앱0.25.0이다.
그 다음 알고리즘 변경 `eef1144`가 움직임/관성 분리 및 외삽 제한을 도입한0.26.0이다.
새로운 보완을 섞지 않고 앱 전체, CSV 로거, FPS 측정 도구를 대상 커밋 그대로 복원했다.
파란 영역 전용 비교 스크립트도 제거했다. 수집 도구와 과거 검증 문서는 이력으로 보존했다.

## 백업과 검증

- 미커밋 실험 파일: `artifacts/rollback_preclaude_20261023/before_rollback.zip`.
- 변경 patch와 추가 파일도 같은 디렉터리에 보존. 기존 사용자 파일은 변경하지 않았다.
- `git diff 43493f4 -- apps/yolopv2_standalone tools/frame_csv_logger.py src/hl_camera_bringup/scripts/measure_boundary_fps.py`: 차이 없음.
- Linux unittest 기존44개 통과(5.937초). ROS 인터페이스 변경 없음.
- 카메라192.168.201.2의 기존 앱을 중지하고 복원한 앱을 실행했다.
- 실제 실행 환경변수0.25.0 확인. 장비 image_boundaries.py SHA256:
  `873c6be6f0d8b5c7621992208cb7421b766bd3e87560f94bcd44bcc5dca68ffc`, 기존0.25 원본과 일치.
- 재측정 `artifacts/rollback_0250_verified/report.json`: 313프레임/31.201초,
  9.9996 FPS, 순번 누락0, 경계쌍 무효0, 중심선0.
- 멀티뷰 연결/좌우 선 표시 확인. 관측이 없는 커브에서 흰색선과 어긋난 예측도 보였으므로
  정확도 성공으로 판정하지 않았다. 요청대로 원본 복원 후 추가 수정하지 않는다.

## 한계

이번 작업은 이전 동작 복원이다. 실제 차선 정확도 개선이나 모든 순간의 경계 표시를
새롭게 보장하지 않는다. 0.25.0 원래 동작과 한계는 `inertial_boundary_hold_20261022.md` 참조.
