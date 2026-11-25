# 카메라 경로 ROS/CAN FD 구현 검증 (2026-11-25)

## 범위와 기준

카메라가 추론하고 WSL 제어컴이 후처리하는 기존 구조를 유지했다.
새 hl_camera_can 패키지는 경로 발행자/구독 CAN 브릿지/드라이버를 분리한다.
Windows Kvaser Leaf v3 환경에 맞춰 Windows CANlib 중계를 추가했다.
기본값은 미보정 경로·물리 드라이버 비활성이다. AURIX 제어 소스는 수정하지 않았다.

사용자 지정 기준 소스 `1113_fix/1024/0_Src/AppSw/Tricore`를 확인했다.

| 확인 항목 | 근거 |
|---|---|
| 500k/2M | Can_Driver.c ACKERMANN_CAN_BAUDRATE/FAST_BAUDRATE 및 FD 설정 |
| ID0x320/FD32bytes/BRS/150ms | Can_AdasRx.h 상수와 CanAdasRx_InitChannel/GetLatestChannel |
| CRC ID 상위→하위→31payload | CanAdasRx_CalculateCrc |
| float32 LE/속도·거리 형식 | CanAdasRx_GetF32Le, GetS16Le, GetU16Le, PollChannel |
| Header 비트/Atomic0/범위 | Can_AdasRx.h, CanAdasRx_ValidatePayload, ExpectedAtomic |
| Function1/ActiveGood2 | Adas_Manager.h |
| 重複 거부/half-range128/타임아웃 후baseline | CanAdasRx_PollChannel, CanAdasValidate_AcceptNormalized |

상위 Sequence 검사에는 fresh 상태의 8비트 모듈로 전진 조건이 있다. 따라서
재시작 시200ms 무송신 후200ms 비활성 구간을 둔다. 하위 수신기의 동일sequence
거부로 첫 baseline 프레임이 중복일 수 있어도 후속 비활성 sequence로 진행한다.

## 검증 결과

- 별도 artifacts/camera_can 빌드로 ROS 인터페이스·패키지 성공. 기존 ros_offload
  설치와 실행 중인 카메라 앱·후처리를 덮어쓰지 않았다.
- 최초 통합28개 테스트 통과: 프로토콜21개, 실제 ROS2개와 Windows 전체경로1개,
  중계 인증/기한4개. 이후 roll/yaw와 드라이버 대기열 만료2개를 추가한 최종 설치본
  검증은30개 중29개 통과/1개 조건부 skip. skip은 별도 서비스가 필요한 Windows
  가상 전체경로이며 앞선 실행에서는 실제로 통과했다.
- 원본 AURIX의 두 CRC 함수를 추출해 C로 컴파일하고 활성/비활성8패킷과 비교했다.
  J1850 표준 체크 문자열123456789→0x4B도 확인했다.
- ROS 합성 관측→경로→CAN: 약20.19Hz,46프레임, 물리송신0. 촬영시각 만료,
  duplicate freshness 미갱신,3개 관측 재활성화,인식실패 즉시비활성,ROS시간 정지/
  역행,소스변경·순서역행,NaN/범위초과,관측유효거리/근거리외삽 검증.
- Windows CANlib 읽기 전용 조회: 채널0 Kvaser Leaf v3, FD지원, 물리채널;
  채널1·2는 FD지원 가상채널. 물리채널0은 열지 않았다.
- 실제 CANlib 가상1→2: 활성/비활성 각각1개,ID0x320,길이32,flags0x30002
  (Standard/FD/BRS),수신payload 일치. canWriteWait에DLC코드13이 아닌32를 전달했다.
- WSL→Windows TCP→Kvaser 가상채널1: 40개/2.0006초,최대RPC4.07ms.
  Windows·WSL 시계 차이를 연결 시 측정하며,관찰된 offset은 고정값으로 사용하지 않는다.
- 실제 ROS 발행·구독→Windows 가상 드라이버: ROS27프레임,Windows25회 전달,
  오류0. 구독 시작/종료 시점이 달라 개수를 동일프레임 대응으로 해석하지 않는다.
- 인증 실패,서비스disabled,기한 만료는 드라이버까지 전달되지 않음을 확인했다.
  Windows 가상 중계 서비스는 검증 후 종료했다.
- 실제 현재 카메라 입력에 기본 설정 연결:6초 CAN121개·약20.14Hz,목표61개,
  전부 비활성,사유calibration_or_capture_timing_unverified,물리송신0,드라이버오류0.
  기본 비활성 ROS 세 노드는 실행 상태로 유지했다.

자료는 `artifacts/camera_can/tests/`의 ros_report.json, windows_ros_report.json,
windows_wsl_report.json, kvaser_virtual.json 및 live_default_report.json이다.
카메라 앱·모델의 기존 회귀검증을 이번 변경으로 다시 수행할 필요는 없으며,
새 ROS interface에 대한 빌드·직렬화·실제 pub/sub로 변경 범위를 검증했다.

## 미검증 사항

실측 camera intrinsics/extrinsics,차로 폭/경사·차체 자세,촬영시각 매핑과 오차,
품질 임계값,차량 전륜축 원점,실제 Leaf v3→AURIX 수신·배선·종단·버스오류,
제어 authority/조향 반응은 현장 검증이 필요하다. 현재 가상 시험은 실차 주행 검증이 아니다.
개발용 외삽 오차와confidence는 확률적 안전성 증명이 아니다.

Windows SDK 상수·함수는 [Kvaser CANlib](https://kvaser.com/canlib-webhelp/group___c_a_n.htm),
[canlib.h](https://kvaser.com/canlib-webhelp/canlib_8h_source.htm),
[canstat.h](https://kvaser.com/canlib-webhelp/canstat_8h_source.htm)를 기준으로 했다.
실행·구체 설정은 [운영 안내](../../operation/camera_can_fd.md)를 따른다.
