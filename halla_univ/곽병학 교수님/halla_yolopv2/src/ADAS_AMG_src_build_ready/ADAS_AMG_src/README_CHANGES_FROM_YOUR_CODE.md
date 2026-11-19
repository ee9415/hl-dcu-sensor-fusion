# 이 zip에서 바뀐 부분 (원본 대비 최소한만 수정)

너가 준 원본 프로젝트를 그대로 베이스로 쓰고, **CAN으로 상위(ROS2) 명령을 받아서
실제 조향/속도에 반영하는 부분만** 추가했어. 조종기(PS2) 관련 코드(`Ps2_Controller.c`)는
**한 글자도 안 건드렸음** — 그대로 원래처럼 동작함.

## 펌웨어 (4개 파일 + dbc)

- `0_Src/AppSw/Tricore/Can_SpeedTx.c` / `.h`
  - 원래 있던 CAN 수신 뼈대(0x300 메시지 오브젝트, alive 타임아웃)에 실제 디코드 로직 추가
  - `CanSpeedTx_DecodePcCommand()`: 0x300 페이로드 → 조향각/속도/EnableFlag 디코드
  - `CanSpeedTx_PollReceive()` 끝에서 그 값을 `ControlLoop_SetTargetSteeringDeg()` /
    `ControlLoop_SetTargetVehicleKph()` 에 반영 (페일세이프 포함, 아래 참고)
  - **★ 핵심 버그 수정 (총 2개의 우회 게이트)**:
    1. `ControlLoop_Init()`이 부팅 시 `g_manualSteeringPulseValid`를 `TRUE`로 고정시켜놔서
       조향이 항상 "수동 펄스 모드"로 걸려있었음 (조종기가 이 방식으로 도는 이유).
    2. `STEERING_POT_PID_ENABLE=1`이라 부팅 시 `g_steeringPotTargetValid`도 `TRUE`로
       고정되어 있어서, 설령 1번을 꺼도 이번엔 "포텐셔미터 PID 모드"가 대신 우회해서
       고정된 raw ADC 중앙값(`STEERING_ADC_RAW_CENTER`)만 타겟으로 씀.
    이 두 게이트가 **둘 다** 꺼져야만 `ControlLoop()`가 실제로 우리가 CAN으로 세팅한
    `g_targetSteeringDeg`(각도 기반)를 사용함. CAN 명령이 활성화될 때
    `ControlLoop_SetManualSteeringPulse(..., FALSE)` +
    `ControlLoop_SetSteeringPotTargetRaw(..., FALSE)` 둘 다 호출하도록 추가함.
  - **새 진단 CAN 메시지 `0x130` (`CanDebugStatus`) 추가**: `candump can0,130:7FF`로
    CAN에서 디코드한 값과 실제 반영된 값을 디버거 없이 바로 비교 확인 가능
- `0_Src/AppSw/Tricore/ControlLoop.c` / `.h`
  - `ControlLoop_SetTargetSteeringDeg()` 함수 신규 추가 (기존 속도용 setter와 대칭)
  - 그 외 기존 로직 전부 원본 그대로
- `AURIX_ACKERMANN_TC275.dbc`
  - `BO_ 768 PcCommand` (상위→AURIX 명령), `BO_ 304 CanDebugStatus` (진단용) 메시지 정의 추가

### 페일세이프 정책
- CAN 명령이 한 번도 활성화(EnableFlag=1)된 적 없으면 → 기존 standalone 기본 주행
  그대로 유지 (조종기 단독 사용 시 영향 없음)
- 한 번이라도 CAN 제어가 활성화된 뒤 통신이 끊기거나 EnableFlag=0이 오면 → 즉시
  조향 중앙/속도 0으로 페일세이프

## ROS2 (`ros2_ws/src/hl_global_mobility_team1_aurix/`)

- **신규 파일만 추가**, 기존 `aurix_can_alive_node.py` 등은 그대로:
  - `hl_global_mobility_team1_aurix/aurix_can_bridge_node.py`
  - `config/aurix_can_bridge.yaml`
  - `launch/aurix_can_bridge.launch.py`
- `setup.py`: entry_point 한 줄 추가 (`aurix_can_bridge_node`)
- `package.xml`: `ackermann_msgs` 의존성 한 줄 추가

## CAN 페이로드 포맷 (0x300 PcCommand, 상위→AURIX, 8바이트 리틀엔디안)

| byte | 필드 | 스케일 |
|---|---|---|
| 0-1 | TargetSteeringDeg | int16, ×10 (0.1 deg/LSB) |
| 2-3 | TargetSpeedKph | int16, ×100 (0.01 km/h/LSB) |
| 4 | EnableFlag | 0=페일세이프, 1=적용 |
| 5 | AliveCounter | 0~255 롤오버 |
| 6-7 | Reserved | 0 |

## 빌드 & 실행

### 1. 펌웨어
AURIX Development Studio에서 이 프로젝트 그대로 열어서 Build → 디버거로 플래시.

### 2. ROS2
```bash
cd ~/ros2_ws/src
rm -rf hl_global_mobility_team1_aurix
cp -r <이 zip 압축 푼 위치>/ros2_ws/src/hl_global_mobility_team1_aurix .

sudo apt install ros-$ROS_DISTRO-ackermann-msgs -y
cd ~/ros2_ws
colcon build --symlink-install --packages-select hl_global_mobility_team1_aurix
source install/setup.bash

bash src/hl_global_mobility_team1_aurix/scripts/setup_can0.sh can0 500000
ros2 launch hl_global_mobility_team1_aurix aurix_can_bridge.launch.py
```

`aurix_lidar_avoid` (라이다 회피 패키지)는 이 zip에 안 들어있음 — 이미 따로 전달한
`aurix_lidar_avoid.zip` 그대로 쓰면 됨. 최종적으로는:
```bash
ros2 launch aurix_lidar_avoid full_stack.launch.py use_can_bridge:=true
```
이거 하나로 회피노드 + 이 CAN브릿지 노드가 같이 켜짐.

## 조향이 여전히 안 먹히면

`candump can0,130:7FF` 로 진단해봐:

| byte | 의미 |
|---|---|
| 0-1 (÷10) | CAN에서 디코드한 조향값 |
| 2-3 (÷10) | 실제 `g_targetSteeringDeg`에 반영된 값 (0-1이랑 같아야 정상) |
| 4 | `g_canRxAlive` (1이어야 함) |
| 5 | `g_canCmdEnableFlag` (1이어야 함) |
| 6 | `s_canControlEverActive` (1이어야 함) |

byte0-1, byte2-3이 둘 다 우리가 보낸 값과 같은데 실제 바퀴만 안 돌면 → 서보 전원/배선
쪽 하드웨어 문제로 좁혀지는 거야.
