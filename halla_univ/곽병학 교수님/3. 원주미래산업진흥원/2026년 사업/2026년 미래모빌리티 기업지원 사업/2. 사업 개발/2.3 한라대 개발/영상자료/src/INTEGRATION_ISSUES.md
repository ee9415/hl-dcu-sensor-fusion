# GPS ROS 2 - AURIX 통합 문제점 정리

- 작성 시각: 2026-08-19 08:21:33 KST (UTC+09:00)
- 최근 수정 시각: 2026-08-19 08:28:58 KST (UTC+09:00)
- 대상 폴더:
  - `gps_waypoint_tracker_ros2_current/gps_waypoint_tracker_ros2`
  - `ADAS_AMG_src_build_ready/ADAS_AMG_src`
- 기준: 저장된 문서보다 현재 소스 코드와 빌드 산출물을 우선하여 검토

## 1. 요약

두 폴더는 서로 다른 플랫폼에서 실행되며 하나의 차량 제어 시스템으로 연동될 목적이다.

- GPS ROS 2 폴더: Ubuntu 22.04/ROS 2 Humble 기반 PC에서 GPS waypoint 추종, 조향·속도 계산, 로깅 및 CAN 통신을 담당한다.
- ADAS/AURIX 폴더: Infineon AURIX TC275에서 CAN 입력 검증, ADAS 중재, 차량 안전 제한, 조향 및 구동 제어를 담당한다.

현재 두 폴더의 GPS 명령 인터페이스는 호환되지 않는다. ROS 브리지는 이전 `0x314` Classic CAN 직접 조향 규격을 구현하지만, 최신 AURIX 소스와 빌드 산출물은 `0x314` 수신 경로를 제거하고 `0x321` CAN FD 궤적 규격을 사용한다.

따라서 현재 파일 그대로 조합하면 AURIX 센서 피드백 등 일부 CAN 통신은 가능할 수 있으나, **ROS GPS 주행 명령은 최신 AURIX GPS 입력으로 전달되지 않는다.**

## 2. 확인된 플랫폼 구성

### 2.1 GPS ROS 2 PC

주요 파일:

- `gps_waypoint_tracker_ros2/gps_waypoint_tracker.py`: GPS waypoint 추종 및 조향 명령 계산
- `gps_waypoint_tracker_ros2/aurix_can_bridge.py`: SocketCAN 송수신
- `gps_waypoint_tracker_ros2/drive_csv_logger.py`: 주행 데이터 기록
- `gps_waypoint_tracker_ros2/waypoint_rviz_visualizer.py`: RViz 시각화

### 2.2 AURIX TC275

주요 파일:

- `0_Src/AppSw/Tricore/Can_SpeedTx.c`: CAN 초기화, Classic CAN 송수신 및 진단
- `0_Src/AppSw/Tricore/Can_TrajectoryFd.c`: Camera/GPS CAN FD 궤적 수신 및 조향 계산
- `0_Src/AppSw/Tricore/Adas_Manager.c`: ADAS 입력 검증 및 중재
- `0_Src/AppSw/Tricore/VehicleManager.c`: 안전 상태와 최종 속도·조향 제한
- `0_Src/AppSw/Tricore/ControlLoop.c`: 5 ms 제어 주기에서 각 모듈 연결

`ADAS_AMG_src` 안의 `ros2_ws`는 AURIX에서 실행되는 코드가 아니라 PC용 구형 ROS 2 브리지다.

## 3. 핵심 인터페이스 불일치

| 항목 | 현재 ROS 브리지 | 최신 AURIX GPS 입력 | 영향 |
|---|---|---|---|
| CAN ID | `0x314` | `0x321` | AURIX GPS 수신 객체와 ID 불일치 |
| CAN 종류 | Classic CAN | CAN FD+BRS | 프레임 형식 불일치 |
| 페이로드 | 8바이트 | 32바이트 | DLC 불일치 |
| 제어 데이터 | 최종 조향각과 속도 | 3차 경로 다항식과 속도 | 제어 책임과 데이터 의미 불일치 |
| Function ID | Nominal `1` | 외부 FD 정의상 GPS `6` | 프로토콜 정의는 다르지만 현재 외부 FD 수신 코드에서는 저장 후 검증하지 않음 |
| 활성 Header | `0x59` | 내부 ADAS 슬롯 생성 결과 `0x5E` | 고정값 비교 없음; 외부에서는 version, enable, lateral-valid 비트만 실제 판단에 사용 |
| Sequence | B6 하위 5비트 | B1 | 위치와 크기 불일치 |
| RequestStatus | B6 상위 3비트 | B2 | 위치 불일치 |
| Segment/PassIndex | B5 | B3 | 위치 불일치 |
| 목표 속도 | B1~B2 | B20~B21 | 위치 불일치 |
| CRC | B7, CAN ID+B0~B6 | B31, CAN ID+B0~B30 | CRC 검증 실패 |
| 송신/감시 주기 | launch 50 ms, ROS 명령 timeout 200 ms | 권장 50 ms, FD watchdog 500 ms | 안전 정책 차이 |

현재 ROS가 `0x314`로 송신한 프레임은 AURIX GPS 수신 객체의 ID와 일치하지 않는다. CAN ID만 `0x321`로 바꾸고 기존 8바이트 페이로드를 그대로 보내더라도 `CanTrajectoryFd_PollChannel()`의 DLC 검사에서 `8 != 32`로 즉시 거부되며, Header와 Function ID 파싱 단계까지 도달하지 않는다. 따라서 DLC, SocketCAN 프레임 형식, 페이로드 내용, CRC 및 경로 생성 방식까지 함께 변경해야 한다.

## 4. 현재 ROS `0x314` 직접 조향 규격

현재 `aurix_can_bridge.py`와 launch가 구현한 외부 송신 규격은 다음과 같다.

- CAN ID: `0x314` (`788`)
- Classic CAN, 11-bit ID, DLC 8
- launch 송신 주기: 50 ms
- B0: 비활성 `0x19`, 활성 `0x59`
- B1~B2: 목표 속도 `int16` little-endian, km/h x100
- B3~B4: 목표 조향각 `int16` little-endian, degree x10
- B5: PassIndex
- B6: RequestStatus 상위 3비트 + Sequence 하위 5비트
- B7: CRC-8/SAE-J1850

SocketCAN 코드도 `=IB3x8s` 형식의 8바이트 Classic CAN 프레임을 사용하며 CAN FD 소켓 옵션을 설정하지 않는다.

## 5. 최신 AURIX `0x321` 궤적 규격

현재 `Can_TrajectoryFd.c/.h`가 구현한 외부 GPS 입력 규격은 다음과 같다.

- CAN ID: `0x321`
- CAN FD, 11-bit ID, DLC 32, BRS 사용
- nominal bitrate: 500 kbit/s
- data bitrate: 2 Mbit/s
- 권장 주기: 50 ms (20 Hz)
- watchdog: 500 ms
- CRC: CRC-8/SAE-J1850, CAN ID 상위/하위 바이트와 B0~B30을 계산하여 B31에 저장

| 바이트 | 형식 | 의미 |
|---|---|---|
| B0 | `uint8` | Header: version/enable/atomic/lateral-valid/long-valid/function ID |
| B1 | `uint8` | Sequence |
| B2 | `uint8` | RequestStatus |
| B3 | `uint8` | Segment `0..6` |
| B4~B7 | `float32 LE` | `c0` |
| B8~B11 | `float32 LE` | `c1` |
| B12~B15 | `float32 LE` | `c2` |
| B16~B19 | `float32 LE` | `c3` |
| B20~B21 | `int16 LE` | 목표 속도, km/h x100 |
| B22~B23 | `uint16 LE` | 경로 유효 길이, cm |
| B24~B30 | 예약 | 현재 수신 코드에서 직접 사용하지 않음 |
| B31 | `uint8` | CRC-8 |

경로 모델:

```text
y(x) = c0 + c1*x + c2*x^2 + c3*x^3
```

GPS 활성 Header의 프로토콜상 기본 비트 조합은 version=0, enable=1, atomic=0, lateral-valid=1, long-valid=1, function ID=6이며 값은 `0x5E`다. 다만 현재 외부 CAN FD 수신 코드는 Header 전체를 `0x5E`와 비교하지 않는다. bit7(version)은 수신 시 검사하고, enable은 활성 여부에, lateral-valid는 패킷 유효성 판단에 사용한다. long-valid와 function ID는 구조체에 저장하지만 이후 읽지 않으며 atomic은 외부 FD 단계에서 사용하지 않는다.

외부 B2 RequestStatus도 구조체에 저장되지만 이후 제어 판단에는 사용되지 않는다. AURIX가 내부 8바이트 ADAS 슬롯 프레임을 만들 때 Function ID, Header와 RequestStatus를 펌웨어 상수 및 현재 활성 상태로 다시 생성한다.

### 5.1 외부 CAN FD 검증 공백

현재 외부 CAN FD 수신 단계는 DLC, CRC, version bit7 및 Sequence 중복을 검사하고 lateral-valid를 패킷 사용 조건으로 확인한다. 그러나 다음 값은 저장만 하거나 별도로 검증하지 않는다.

- 외부 Function ID
- Atomic
- LongValid
- RequestStatus
- `c0~c3`의 NaN/Infinity 여부
- Segment 범위
- 목표 속도 원시값 범위
- 경로 유효 길이의 최소·최대 범위

목표 속도와 최종 조향각 일부는 내부 슬롯 프레임 생성 과정에서 제한되지만, 외부 패킷 자체의 의미 검증은 프로토콜 설명보다 느슨하다. ROS 송신 측 수정과 함께 수신 검증 강화 여부를 결정해야 한다.

## 6. AURIX 실제 실행 경로 확인

현재 소스에서 다음 흐름이 확인되었다.

```text
CanSpeedTx_Init()
  -> CAN 노드를 FD 모드로 초기화
  -> CanTrajectoryFd_InitAll()
  -> Camera 0x320 / GPS 0x321 수신 객체 초기화

ControlLoop() (5 ms)
  -> CanTrajectoryFd_PollChannel(g_canTrajFdGps)
  -> 32바이트/DLC/CRC/Sequence 검사
  -> 궤적을 사용해 AURIX에서 최종 조향각 계산
  -> 내부 8바이트 GPS 슬롯 프레임 생성
  -> AdasManager_OnCanFrame(0x321, ...)
  -> VehicleManager를 거쳐 최종 차량 제어
```

`AdasManager_OnCanFrame(..., 8)` 호출은 외부 GPS 규격이 8바이트라는 뜻이 아니다. AURIX가 외부 32바이트 궤적을 받은 뒤 내부 중재용 8바이트 프레임으로 변환해 주입하는 구조다.

Classic CAN ADAS 수신 객체는 `0x310`~`0x313`만 초기화되며, 이전 `0x314` 수신 객체는 최신 코드에서 제거되어 있다.

## 7. 레거시 `0x300` 경로

AURIX에는 별도의 `0x300` Classic CAN 직접 명령 경로가 남아 있다. 하지만 이것은 현재 ROS의 `0x314` 페이로드와 다른 규격이다.

- CAN ID: `0x300`
- B0~B1: 속도 x100
- B2~B3: 조향각 x10
- B4~B5: 조향 pulse us
- B6: Sequence
- B7: `0xA0` 계열 signature와 enable bit
- watchdog: 250 ms

따라서 ROS의 CAN ID만 `0x300`으로 변경해도 정상 동작하지 않는다.

## 8. 문서와 코드의 불일치

다음 문서 또는 주석은 이전 `0x314` 구조를 설명하고 있어 최신 AURIX 코드와 맞지 않는다.

- `ADAS_MANAGER_CAN_SPEC.md`
- `aurix_can_bridge.py`의 “Uploaded firmware commissions 0x314...” 주석
- `ControlLoop.c`의 “0x310..0x314 command commissions ADAS...” 주석

GPS 패키지 `README.md`와 `AURIX_CAN_ALIGNMENT.md`는 `0x314`가 아니라 현재 AURIX에도 별도 경로로 남아 있는 레거시 `0x300` 계약을 설명한다. 따라서 v7의 `0x314` 제거 문제와 직접 연결되는 문서 목록에서는 제외한다. 다만 GPS 패키지의 현재 브리지가 `0x314`를 사용한다는 사실을 README가 명확히 반영하지 않는 별도의 문서 구성 문제는 남는다.

Markdown 파일 자체의 한글 인코딩은 정상이다. 모든 원본 Markdown 파일이 BOM 없는 UTF-8 strict decoding에 성공했으며 Unicode replacement character도 발견되지 않았다. 최초 검토 당시의 깨진 표시는 Windows PowerShell 5.1에서 BOM 없는 UTF-8 파일을 `Get-Content`로 읽으면서 `-Encoding UTF8`을 지정하지 않아 발생한 도구 측 디코딩 문제였다.

최신 인터페이스 확정 후 실제 `0x314` 관련 문서와 주석을 코드, 테스트, DBC 및 launch와 함께 갱신해야 한다.

## 9. 불일치 발생 경위에 대한 강한 정황

파일 타임스탬프는 다음 순서를 가리킨다.

| 파일 | 최종 수정 시각 |
|---|---|
| `ADAS_MANAGER_CAN_SPEC.md` | 2026-08-16 22:43경 |
| ROS `aurix_can_bridge.py` | 2026-08-17 04:16경 |
| AURIX `Can_TrajectoryFd.c`, `ControlLoop.c`, `Can_SpeedTx.c` | 2026-08-18 00:43~00:46경 |
| AURIX ELF/HEX/map | 2026-08-18 07:33경 |

코드의 `v7` 주석, `0x314` 제거 설명, ROS의 이전 펌웨어 전제 주석 및 빌드 시각을 종합하면 다음 과정으로 추정된다.

```text
Legacy 0x314 직접 조향 문서/ROS 구현
  -> AURIX가 0x320/0x321 CAN FD 궤적 구조로 리팩터링
  -> 새 AURIX 소스가 ELF/HEX로 빌드됨
  -> ROS 브리지, 테스트, DBC 및 문서에는 변경이 전파되지 않음
```

타임스탬프는 복사나 압축 해제 과정에서 달라질 수 있으므로 이것만으로 개발 이력을 확정할 수는 없다. 그러나 실제 코드와 빌드 map도 동일한 변경 방향을 보여 주므로 가능성이 매우 높다.

## 10. 빌드 산출물 확인

제공된 AURIX 빌드 산출물:

- `AURIX_ACKERMANN_TC275_CANFD.elf`
- `AURIX_ACKERMANN_TC275_CANFD.hex`
- `AURIX_ACKERMANN_TC275_CANFD.map`

map 파일에는 다음 함수가 실제 링크되어 있다.

- `CanTrajectoryFd_InitAll`
- `CanTrajectoryFd_PollChannel`
- `TrajFd_BuildAdasManagerSlotFrame`

따라서 폴더에 포함된 최신 HEX는 CAN FD 궤적 구현을 포함한 빌드로 판단된다. 단, 실제 차량에 이 HEX가 플래시되었는지는 파일만으로 확인할 수 없다. 차량에 과거 펌웨어가 설치되어 있다면 실제 동작은 달라질 수 있다.

## 11. 결정된 통합 방향

사용자는 **AURIX 궤적 방식을 유지**하기로 결정했다.

목표 구조:

```text
GPS waypoint
  -> ROS에서 차량 기준 전방 경로 생성
  -> 3차 다항식 c0~c3 피팅
  -> 0x321 CAN FD 32바이트 전송
  -> AURIX에서 Stanley/Curvature 기반 최종 조향 계산
  -> VehicleManager 안전 제한 후 차량 적용
```

현재 미확정인 현장 값은 우선 ROS parameter로 만들고 안전한 기본값을 사용한다.

권장 기본값:

- 좌표계: 현재 차량 위치 원점, 차량 전방 `+x`, 차량 왼쪽 `+y`, 단위 m
- heading: 기존 GPS tracker의 진행 방향 계산값 재사용
- 경로 피팅 범위: 전방 0.5~5.0 m
- 최소 피팅 점: 4개
- 경로/heading/GPS 무효 시: 비활성 프레임 및 목표 속도 0
- 초기 실행: `dry_run=true`, `enable_drive=false`
- 실제 주행 전 저속 단계 검증 필수

## 12. 추가 확인이 필요한 항목

다음 항목은 현재 폴더만으로 확정할 수 없다.

1. 실제 차량에 플래시된 AURIX HEX 버전
2. PC CAN 어댑터의 CAN FD 및 BRS 지원 여부
3. 실제 CAN 인터페이스 이름 (`can0` 예상)
4. 차량 heading에 사용할 최종 ROS 토픽 또는 기존 tracker 값
5. 차량 기준 경로 좌표계가 AURIX의 다항식 계산 가정과 정확히 일치하는지
6. Camera `0x320`과 GPS `0x321`을 동시에 사용할 때 전환 정책
7. 실제 코스에 맞는 피팅 거리, 목표 속도 및 안전 제한값

특히 현재 AURIX `ControlLoop.c`는 궤적 조향 계산에 `vehicleHeadingRad=0.0`을 전달한다. ROS가 차량 좌표계로 변환된 궤적을 보내는 전제라면 의도된 구성일 수 있지만, 전역 좌표계 궤적을 보낼 경우 잘못된 계산이 된다. ROS 송신 좌표 규약과 함께 검증해야 한다.

## 13. 예정 수정 범위

1. ROS tracker에서 차량 위치, heading 및 전방 경로 데이터 제공
2. waypoint를 차량 좌표계로 변환
3. 3차 다항식 `c0~c3` 피팅 구현
4. `aurix_can_bridge.py`를 `0x321` CAN FD 32바이트 송신으로 변경
5. Header, Sequence, RequestStatus, Segment 및 CRC 구현
6. SocketCAN FD 옵션과 CAN 인터페이스 설정 스크립트 추가/수정
7. 경로 무효, timeout, 시작 및 종료 시 안전 프레임 처리
8. 단위 테스트와 golden vector 추가
9. launch/config 파라미터 정리
10. CAN 사양서, DBC, README 및 오래된 주석 갱신
11. dry-run에서 프레임 바이트 검증 후 실제 CAN 수신 검증

## 14. 현재 결론

- 불일치는 단순 CAN ID 오타가 아니다.
- 직접 조향 명령에서 궤적 전달로 제어 아키텍처가 변경되며 파생된 인터페이스 전체의 불일치다.
- 최신 AURIX 구조를 유지하려면 ROS 송신 측을 `0x321` CAN FD 32바이트 궤적 프로토콜로 변경해야 한다.
- CAN ID만 변경하거나 8바이트 페이로드를 그대로 재사용하는 방식으로는 해결할 수 없다.
- 실제 차량 적용 전에는 CAN FD 장비 확인, dry-run, CRC/DLC 검증, 비활성 프레임 검증 및 저속 시험이 필요하다.
