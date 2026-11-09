# 전송자 없는 PC 입력의 영상 재연결 복구 (2026-11-09)

## 원인

사용자 로그에서 영상 연결 준비 완료 후 RGB/결과/TOP/Depth가 모두 15초 timeout을 반복했다. 장치 조회 결과 앱은 running/code1이지만 입력은 pc, publisher_connected=false, 수신39311/소비39310으로 남아 있었다. PC 전송 종료 후 입력을 임의로 카메라로 바꾸지 않는 기존 정책에 따라 프레임 없이 대기했으며, 새 UI는 포트 개방만 검사하고 영상 연결을 시작했다.

## 변경

명시적인 영상 연결 버튼 처리에서 새 prepare_preview 명령을 먼저 확인한다. 장치 lock 안에서 PC 입력·전송자 없음일 때만 camera로 전환하고 source epoch를 증가시킨다. 전송 중인 PC 소스 및 owner는 유지한다. 연결 종료 직후의 자동 전환은 하지 않는다. 준비 응답 실패 시 ready를 표시하지 않는다. 카메라 미리보기 복구 후 PC 영상은 전송 시작으로 재개할 수 있다.

## 검증

- WSL 전체391개 통과(62.416초). 추가 PC UI 연결 준비 테스트2개 통과: 복구 응답 후 ready 순서, 실패 시 오류 처리.
- 카메라 실제 소켓에서 연결된 PC owner 보존을 확인하고 연결 종료 후 명시적 복구를 시험했다. RGB44/결과10/TOP10/Depth25프레임, 모두 connected·최근 프레임0.208초 이내·재접속0.
- 실행 환경0.33.10 UUID `61fbe0c1-ad26-42dc-93c4-a8d2b5191fce` 유지. 코드 `0.33.10-code2` ZIP82,305bytes만 전송. release `299a76ffe5551c30a90ce398d45684e358f895a1116f53d174d6a1280b4604b1`, runtime fingerprint 유지, 이전 code1 보관. 전체 컨테이너 빌드/재설치 없음.
- 근거: `artifacts/idle_pc_reconnect_20261109`의 tests.log, deployed.json, live_report.json, multiview.log.
- 새 UI를 실행하기 전 PC 소스 선택 후 소켓 종료로 문제 상태를 다시 만들었다. 실제 영상 연결 버튼 클릭15:33:55에 복구 로그를 확인했고 RGB15:33:56, Depth15:33:57, TOP/결과15:33:58에 connected. UI 네 패널 영상 표시 확인. 최종 카메라 미리보기 상태로 유지했다.

ROS 드라이버·토픽·TF 변경이 없는 앱 입력 제어/UI 수정이다. 실제 PC 영상이 연결된 채로 일시정지된 상황은 명시적으로 유지하며 이 복구의 대상이 아니다.
