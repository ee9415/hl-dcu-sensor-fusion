# Windows CAN 브릿지 메시지 모니터 (2026-11-25)

## 목적

AURIX가 없는 상태에서 WSL ROS CAN 브릿지가 생성한 메시지를 Windows 중계
지점에서 캡처한다. 이 프로그램은 Kvaser SDK를 import하거나 CAN 채널을 열지 않는다.
CAN 트랜시버로 보내지 않으며, 물리 송신용 중계와 캡처용 중계는 handshake에서 구분한다.

```text
기존 카메라 ROS 결과 → 경로 발행자 → CAN 브릿지 /can/tx
  → WSL 모니터 전용 전달 → Windows CAN 모니터
                                 └ 화면 + JSONL 저장
```

## 실행

Windows에서 `tools/start_windows_can_monitor.cmd`를 더블클릭한다.
또는 저장소 루트에서:

```powershell
.\.venv\Scripts\pythonw.exe tools\windows_can_monitor.py
```

기본적으로 Ubuntu-22.04의 기본 gateway를 조회하고 Windows가 바인딩할 수 있는
주소인지 확인한다. 확인하지 못하면127.0.0.1로 시작하며 화면에서 수정할 수 있다.
명시적 주소는 `--host 172.18.192.1`처럼 지정한다. 실제 주소는 환경별로 확인한다.
모니터는53200포트와 기존 `artifacts/camera_can/windows.key`를 사용한다.
키가 없으면 생성하며, 값을 화면·로그에 출력하지 않는다.

기존 camera_can 런치를 종료한 뒤 WSL 저장소 루트에서:

```bash
bash tools/run_camera_can_monitor.sh
# 주소를 명시할 경우
bash tools/run_camera_can_monitor.sh 172.18.192.1
```

이 실행기는 `artifacts/camera_can/monitor_config.json`을 생성한다.
windows_monitor_only=true, Windows주소·공유키 경로를 설정하고 ROS 전달을 켠다.
enable_driver=true는 이 구성에서는 TCP 캡처 전달만 뜻한다. 실제 드라이버를
켜는 `windows_kvaser_can.py --enable-can`은 실행하지 않는다.
기존 카메라 앱·후처리와 기본 경로 품질 기준은 바꾸지 않는다.

현재 보정 미완료 기본 설정에서는 Header0x81/INACTIVE,계수·속도·거리·신뢰도0의
패킷이20Hz로 보이는 것이 정상이다. Sequence와 CRC는 계속 변한다.

### 임시 보정 시험 프로필 (2026-11-25)

사용자 요청으로 실측이 아닌 초기 실험값을 분리했다.
`src/hl_camera_can/config/camera_can.monitor_bench.json`은 기본 설정에 합치는
override 파일이며, 단독 config_path로 쓰지 않는다.

```bash
bash tools/run_camera_can_monitor.sh 172.18.192.1 53200 bench
```

초기값: 높이0.65m,전륜축 중앙과 같은전방/좌우위치0m,pitch하향25°,yaw/roll0°,
320×180의fx/fy190px,cx159.5/cy89.5. 촬영시각 불확실성50ms도 실측값이 아닌
시험용 가정이다. calibrated/timing_verified는false,bench_only는true를 유지한다.
시험 품질값은 최소신뢰도15,경계신뢰도0.3,관측점6개,관측구간0.5m,
근거리외삽최대1.2m,피팅오차0.2m,원점오차지표0.3m,거리최대8m다.
실제 보정값으로 간주하거나 물리 송신 설정에 사용하지 않는다.

NaN·범위·만료·연속3프레임 검사는 유지한다. 2026-11-25 사용자 요청에 따라
추정/held 경계도 후보에 포함하고 신뢰도에 감점한다. 최초 적용 당시40개 영상 샘플은
양쪽동시관측 조건으로 모두 비활성이었으나, 변경 후33개 후보 통과/7개 저신뢰 비활성이다.
짧은 유효구간·피팅실패·신뢰도 기준미달에서는INACTIVE/계수0을 유지한다.
프로필을 적용한 실제 노드도bench_only=true,물리송신0이며,계수를 임의로 만들어
정상 경로처럼 송신하지 않는다. 기본 실행(bench 인자 없음)은 기존 미보정 차단으로 돌아간다.

## 화면

- ID0x320,Standard/FD/BRS,32bytes와DLC13 표시.
- ACTIVE/INACTIVE,Header/Request,Sequence,CRC 수신값/계산값.
- C0~C3,유효거리,신뢰도,최소·최대속도.
- 최근250개 원시32바이트·수신간격·Sequence중복/누락/역행 검사.
- 목록에서 프레임을 선택하면 해당 값이 고정된다. ‘최신 프레임 따라가기’로 복귀.
- ‘수신 중지’는 네트워크 수신을 중지한다. 화면 고정만 하려면 따라가기 체크를 해제한다.
- 모든 캡처는 `artifacts/camera_can/monitor/capture_*.jsonl`에 자동 저장한다.
  화면에 표시되지 않는 과거 데이터도 로그에 남는다. ‘로그 폴더 열기’로 확인한다.

## 모드 분리 및 진단

모니터 응답은 captured=true/sent=false다. 물리 송신 클라이언트는 모니터를
거부하고, 모니터 전용 클라이언트는 물리 송신 서버를 거부한다.
serve_connection에 캡처 콜백과 하드웨어 드라이버를 동시에 넘길 수도 없다.
`/can/driver_status`의 captured는캡처완료,sent는물리/실드라이버전달횟수다.
모니터 모드에서는 monitor_only=true, sent=0이어야 한다.

Windows Python의 시각 해상도와 WSL clock slew 영향을 줄이기 위해 Windows는
GetSystemTimePreciseAsFileTime을 사용하고 송신 프레임마다 시계offset을 갱신한다.
표시용 수신 간격은15.625ms 해상도의GetTickCount64 대신QPC(perf_counter)로 측정한다.
기존10ms 왕복·시계급변 한계와 packet deadline 검사는 유지한다.
오류 패킷은 화면·로그에서 오류로 표시하고 정상 전달로 처리하지 않는다.

2026-11-25 검증:36개 테스트 통과,물리/가상Kvaser서비스가 필요한 기존1개는skip.
실제 WSL→Windows GUI30초605개,패킷/Sequence오류0,약19.99Hz,물리송신0.
프로세스의 Kvaser/CANlib DLL 로드0개 확인. 처음 시계검사에서 관찰한 오류는
고정밀 시각·프레임별offset 갱신 적용 후 위 연속구간에서 재현되지 않았다.
자료: `artifacts/camera_can/monitor/verification.json`, `state.json`,캡처JSONL.
