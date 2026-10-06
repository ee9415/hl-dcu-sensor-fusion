# YOLOP 6-Camera Web UI Test Log

Date: 2026-08-12

## Scope

- Target: DCU에서 실행되는 비-ROS2 `tools/yolop_6cam_recorder` Web UI
- Cameras: OAK-D Pro PoE 6대
- UI layout: 상단 `cam_04 / cam_01 / cam_05`, 하단 `cam_02 / cam_06 / cam_03`

## Recorded Results

- Local과 DCU의 `web_ui/index.html` SHA-256 일치
- DCU Web UI 재시작 성공
- 6대 카메라 모두 약 10 FPS 상태 보고
- preview/playback 패널 3열 2단 layout 적용

## Limitations

- 자동화된 browser screenshot은 당시 browser instance가 없어 수행하지 못함
- operator display에서의 최종 육안 확인 필요
- 본 결과는 비-ROS2 Web UI 검증이며 6카메라 ROS2 topic, TF, RViz 검증이 아님

## Source

동일 날짜의 `CHANGELOG.md` 기록을 검증 로그 형식으로 정리하였다. 추가 장비 시험을
수행한 기록은 아니다.
