# Depth 멀티뷰 연동 검증 — 2026-10-16

## 변경 근거와 구현

기존 Depth 칸은 수신 소스 없이 대기 문구만 표시했다. 장비의 RGB 정렬
Depth를 3:1 샘플링하여 640×360 컬러 H.264로 보내고 기존 수신/표시 루프에
추가했다. 포트 8556, 경로 /depth, 설정 10 FPS이다. 원본 mm 값과 객체
거리 계산 입력은 변경하지 않았다. 빨강 0.3m~파랑 10m 고정 로그 색상이며
범위 밖은 포화, 무효값은 검정이다. 압축 영상은 정량 Depth 저장용이 아니다.

RemoteConnection은 OAK_WEB_VISUALIZER=1일 때만 활성화한다.
assign_frontend_port=false로 변경했다. 기본 컨테이너가 출력하는 Viewer URL과
nginx 서비스는 남아 있지만 멀티뷰 Depth는 웹에 의존하지 않는다.

## 검증 방법과 결과

- 장비 192.168.0.4에 0.13.0 배포. 처음 발견한 HostNode.out 읽기 전용
  속성 충돌을 preview_out으로 수정한 뒤 full pipeline 실행 확인.
- 전체 단위 테스트 57개 통과, 수정 Python 파일 구문 검사 통과.
- 실제 Ubuntu Tk GUI의 20초 검사: 창 mapped=true.
  RGB 1920×1080, YOLO 960×540, Depth 640×360 모두 connected=true,
  각 Label의 ImageTk 이미지 존재 확인. BEV는 당시 connected=false.
- 재검사에서는 20초 안에 BEV/Depth 미연결이 관측되어 연결 검사 대기 시간을
  60초로 조정하고 수신 프레임 계수기를 추가했다.
- 60초 재검사 창들은 결과 출력 전 종료되어 네 화면 동시 성공을 확인하지 못했다.
- 장비 후속 HEALTH 로그 frames=27899, average_fps=10.0 확인.
  이는 후처리 파이프라인 평균이며 각 RTSP 수신 FPS 측정값은 아니다.
- computer-use 실제 창 캡처는 앱 접근 승인 시간 초과로 완료하지 못했다.

## 미확정 및 제한

네 화면의 안정적인 동시 수신, 재연결 시간, 육안 화면 검증은 추가 확인 필요.
Depth 단독 수신과 Tk 이미지 생성은 확인했으나 최종 육안 검증 완료로 간주하지 않는다.
카메라가 모니터를 촬영하면 Depth는 모니터까지의 거리이며 영상 속 도로 거리가 아니다.
이번 변경은 standalone 앱과 PC 수신기 범위로 ROS 드라이버/토픽/TF는 변경하지 않았다.
ROS launch/colcon/실차 조향 안전성 검증은 이번 범위에서 수행하지 않았다.
