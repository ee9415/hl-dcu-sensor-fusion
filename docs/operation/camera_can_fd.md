# 카메라 목표 경로 → AURIX CAN FD (2026-11-25)

## 실행 구조

```text
OAK 추론 → 기존 WSL ROS bridge/postprocess
  /oak/postprocess/result
    → camera_path_publisher: 관측 경계의 차량 좌표 변환·목표 경로 생성
  /camera/target_path (hl_oak_interfaces/TargetPath)
    → camera_can_bridge: 입력 구독·촬영 시각 만료·3개 정상 영상·독립20Hz 타이머
  /can/tx (hl_oak_interfaces/CanFdFrame)
    → camera_can_driver: 대기열 만료 검사·Windows TCP 중계
    → Windows tools/windows_kvaser_can.py → Kvaser CANlib → Leaf v3 → AURIX
```

ROS 노드는 WSL, Kvaser 장치 드라이버는 Windows에 있다. Windows 드라이버를
WSL SocketCAN(can0)으로 직접 사용할 수 있다고 가정하지 않는다. Linux CAN FD
환경으로 옮길 때는 설정의 driver_backend=socketcan을 선택할 수 있다.
조향각·차량 제어·ADAS authority 선택은 기존 AURIX가 담당한다.

## 최초 빌드와 기본 실행

기존 카메라 후처리는 [OAK ROS 운영 안내](oak_ros_offload.md)를 따른다.
그와 별도로 WSL 저장소 루트에서:

```bash
source /opt/ros/humble/setup.bash
colcon --log-base artifacts/camera_can/log build \
  --base-paths src/hl_oak_interfaces src/hl_camera_can \
  --build-base artifacts/camera_can/build \
  --install-base artifacts/camera_can/install
bash tools/run_camera_can.sh
```

기본값은 **보정 미완료·실제 드라이버 비활성**이다. ROS `/can/tx`에는 비활성
패킷만 발행하며 Windows 채널은 열지 않는다. 실행기는 flock으로 중복 실행을 막는다.
직접 ros2 launch를 중복 실행하지 않는다. 기존 ROS 노드·topic·TF·카메라 앱은 변경하지 않는다.
빌드는 별도 artifacts/camera_can 경로를 사용해 실행 중인 ros_offload 설치를 보존한다.

```bash
source artifacts/camera_can/install/setup.bash
ros2 topic echo /camera/target_path --once
ros2 topic echo /camera/can_status --once
ros2 topic echo /can/driver_status --once
```

## 설정과 경로의 의미

`src/hl_camera_can/config/camera_can.json`을 별도 JSON으로 복사해서 수정한 뒤
`config_path:=/절대/경로.json`으로 지정한다. 세 노드는 같은 파일을 사용하며
설정 변경 후 재시작한다. 설치된 기본 설정은 재빌드로 갱신한다.

- 원점은 전륜축 중앙, +X 전방, +Y 왼쪽, 단위m, frame_id=vehicle_front_axle.
- 좌우 **현재 observed 경계의 supported 영상점**만 사용한다. held/가상 경계,
  표시용 TOP·픽셀 다항식은 CAN 계수로 사용하지 않는다.
- 보정한 영상 내부 파라미터(fx/fy/cx/cy), 카메라 위치(forward/left/height),
  roll·하향 pitch·좌향 yaw(Rz·Ry·Rx 순서)로 광선을 지면 Z=0에 투영한다. 도로 평면 가정이다.
- 좌우 경계의 관측 X 범위가 겹치는 구간에서 차량 좌표의 중점을 계산하고
  `Y=C0+C1*X+C2*X²+C3*X³`을 피팅한다. 유효 거리는 관측된 먼 끝 X와12m 중
  작은 값이며, cm 송신값은 내림하여 관측 범위를 늘리지 않는다.
- 원점부터 가까운 관측 시작점까지는 외삽이다. 기본0.8m 이내, 충분한 관측 구간,
  피팅 오차·원점 외삽 오차 지표·차로 폭·신뢰도 조건을 모두 통과해야 한다.
- confidence는 초기 개발용 품질 점수이며 통계적으로 보정된 성공 확률이 아니다.
- geometry의 위치·각도·내부 파라미터는 **실측값이 아닌 임시값**이다. 기본
  calibrated=false, timing_verified=false, capture_uncertainty_ms=1000이다.
  차량 장착 보정과 촬영 시간 매핑을 검증한 뒤 각각의 플래그와 오차 한계를 설정한다.
- 현재 ROS stamp는 카메라 capture clock을 최초 호스트 수신 시각에 대응한 추정값이다.
  capture_uncertainty_ms까지 더한 데이터 나이를 검사한다. 단순히 수신 시각으로
  stamp를 덮어써서 오래된 영상이 새 영상처럼 보이게 하지 않는다.
- PC 영상은 기본 거부한다. 구동계 분리 시험에서만 geometry.bench_test_active=true와
  can.allow_bench_active=true를 명시할 수 있다. bench_only가 표시되며 실제 보정으로
  간주하지 않는다. 임시 내부값으로 품질 검증을 통과하지 못할 수도 있다.

기본 경로 만료는 촬영 시각 기준300ms(불확실성 포함), 재활성화는 서로 다른
연속 정상 프레임3개다. 같은 프레임의 중복 수신은 나이와 정상 횟수를 갱신하지 않는다.
소스 변경·역행·잘못된 경로·신뢰도 부족은 즉시 비활성화하고 새3개를 요구한다.
ROS 시간이 멈춰도 steady 송신 타이머와 monotonic 경과시간으로 만료된다.

## Windows Kvaser 연결

2026-11-25 읽기 전용 조회에서 채널0=Kvaser Leaf v3, CAN FD 지원을 확인했다.
채널1·2는 가상 채널이었다. 채널 번호는 재연결 후 다시 조회한다.

Windows PowerShell, 저장소 루트에서:

```powershell
.\.venv\Scripts\python.exe tools\windows_kvaser_can.py --list
```

기본 서비스 실행은 실제 CAN을 켜지 않는다. 명시적으로 `--enable-can`을 줘야
채널을 exclusive ISO CAN FD로 열고500kbit/s·2Mbit/s·BRS로 bus-on한다.
`--virtual`을 주면 반드시 가상 채널이어야 하며 물리 채널 선택을 거부한다.

```powershell
# 실제 장치 사용 시: 차량 조건·배선·보정 확인 후 명시적으로 실행
.\.venv\Scripts\python.exe tools\windows_kvaser_can.py `
  --enable-can --channel 0 --host <Windows의-WSL-가상네트워크-IP>
```

WSL NAT 모드의 Windows 주소는 `ip -json route show default`의 gateway로 확인한다.
이번 확인값은172.18.192.1이며 재시작 후 바뀔 수 있다. mirrored 모드의 localhost
지원 여부를 확인하지 않고127.0.0.1을 Windows라고 간주하지 않는다. 방화벽 설정은
자동 변경하지 않는다. 서비스는 지정한 주소에만 바인딩하며 기본127.0.0.1이다.

서비스가 생성하는 `artifacts/camera_can/windows.key`는 인증용 공유 키다.
값을 출력하거나 Git에 추가하지 않는다. WSL 설정 복사본에 다음 항목을 맞춘다.

```json
{
  "driver_backend": "windows_tcp",
  "windows_host": "172.18.192.1",
  "windows_port": 53200,
  "windows_key_file": "/mnt/d/git_hub/hl-dcu-sensor-fusion/artifacts/camera_can/windows.key",
  "windows_allow_virtual": false
}
```

이는 전체 설정 파일의 `can` 섹션 일부다. 나머지 필드는 보존한다.

```bash
bash tools/run_camera_can.sh config_path:=/절대/설정.json enable_driver:=true
```

Windows 중계는 인증된 단일 연결에서 한 요청씩만 처리하고 재생 큐를 만들지 않는다.
매초 왕복으로 Windows·WSL 시각 offset의 하한을 추정하고10ms 여유를 차감한다.
왕복10ms 초과·시계 급변·인증 실패·기한 초과는 송신하지 않는다. ROS 프레임에도
생성 monotonic 시각과 경로 만료 deadline을 넣어 대기열에서 늦어진 ACTIVE를 거부한다.
이 메시지는 같은 제어컴의 WSL 노드 사이 전용이다(서로 다른 호스트의 monotonic
시각을 비교하는 인터페이스가 아니다). Windows도 실제 canWriteWait 직전에 기한을
다시 확인한다. 하드웨어 송신 확인에20ms를 배정하고, 실패 시 TX flush·bus-off·close하여
이전 ACTIVE를 무기한 재전송하지 않는다. 재시작은 명시적으로 한다.

## AURIX 규격 확인

기준: 사용자가 지정한 `1113_fix/1024/0_Src/AppSw/Tricore`의
Can_Driver.c, Can_AdasRx.c/.h, Can_AdasValidate.c, Adas_Manager.h.

- Standard ID0x320, FD+BRS,32bytes, arbitration500k/data2M,50ms.
- SocketCAN len과 Kvaser canWriteWait의 dlc 인자는 **바이트 수32**다.
  실제 버스 DLC 코드는13이다. CAN 드라이버에13바이트를 넘기지 않는다.
- ACTIVE는 Header0xD9/Status2, Function1/Atomic0, 네 float32 LE,
  min0/max300(km/h×100), 유효 거리cm, confidence1~100. Context/Reserved0.
- INACTIVE는 Header0x81/Status0, Sequence·CRC 외 payload 전부0.
- CRC-8/SAE-J1850은 `03 20 + payload[0:31]`, poly1D/initFF/xoroutFF.
- 하위 수신기는 바로 전 sequence와 같은 값을 거부한다. 상위 validator는
  fresh 중 delta=(new-old)&FF가1~127이면 수락,0은중복,128~255는거부한다.
  150ms 초과·무효화 후에는 다음 검증 프레임을 새 baseline으로 삼는다.
- 새 bridge는 기본200ms 무송신 후200ms 비활성부터 시작한다. 기존 송신기가
  동시에 계속 보내는 상태에서는 이 재동기화가 성립하지 않는다. 송신 주체는1개다.

실차 보정, 경사·차체 운동, 실제 촬영 나이, Leaf↔AURIX 배선·종단·수신 상태,
제어 authority 및 조향 반응은 현장 검증 대상이다. 유효 경로 발행 자체는 주행 허가가 아니다.
