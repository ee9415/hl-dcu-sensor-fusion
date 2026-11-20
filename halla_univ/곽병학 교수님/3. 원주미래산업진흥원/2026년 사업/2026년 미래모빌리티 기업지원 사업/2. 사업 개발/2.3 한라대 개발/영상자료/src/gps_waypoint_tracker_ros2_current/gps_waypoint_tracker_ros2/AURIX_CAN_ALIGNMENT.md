# TC275 CAN 정합 확인

검토 원본:

```text
/home/jigu/Downloads/AURIX_ACKERMANN_TC275 (12).Zip
```

검토한 최신 파일:

```text
0_Src/AppSw/Tricore/Can_SpeedTx.h
0_Src/AppSw/Tricore/Can_SpeedTx.c
0_Src/AppSw/Tricore/ControlLoop.c
0_Src/AppSw/Tricore/CCU6_PWM_Generation.h
0_Src/AppSw/Tricore/Steering_ADC.h
0_Src/AppSw/Tricore/Board_Pins.h
```

## PC에서 AURIX로 보내는 명령

- CAN ID: `0x300`
- Standard CAN 11-bit
- Bitrate: `500000 bit/s`
- DLC: `8`
- Byte order: little-endian
- 권장 송신 주기: `50 ms` (20 Hz)

| Byte | 형식 | 의미 |
|---|---|---|
| 0–1 | `uint16` | 목표 속도, km/h × 100, `0..400` |
| 2–3 | `int16` | 목표 조향각, degree × 10, `-270..270` |
| 4–5 | `uint16` | 기대 조향 펄스 µs |
| 6 | `uint8` | 매 프레임 증가하는 Sequence |
| 7 | `uint8` | `0xA0` STOP, `0xA1` DRIVE |

조향 부호는 `+LEFT/-RIGHT`입니다. 펄스는 다음과 같습니다.

| 조향 | 펄스 |
|---|---:|
| `+27°` 왼쪽 | `1110 µs` |
| `0°` 중앙 | `1660 µs` |
| `-27°` 오른쪽 | `1900 µs` |

AURIX는 조향각에서 다시 계산한 펄스와 Byte 4–5의 차이가 `3 µs`를
초과하면 프레임을 거부합니다.

STOP 프레임은 Signature만 바꾸면 안 됩니다. 속도 `0`, 조향각 `0`,
펄스 `1660`을 함께 보내야 유효합니다.

## AURIX 안전 동작

- 유효 명령 watchdog은 `250 ms`입니다.
- 같은 Sequence를 반복하면 watchdog이 갱신되지 않습니다.
- 잘못된 `0x300` 프레임은 PC 모드 진입 후 즉시 STOP으로 처리됩니다.
- GPS 조향 입력이 끊기면 ROS2 bridge는 `200 ms`에 먼저 STOP을 보냅니다.
- ROS2 bridge는 STOP을 포함하여 Sequence를 `255 → 0`으로 계속 증가시킵니다.
- 종료 시 STOP 프레임을 3회 추가 송신합니다.

## AURIX에서 PC로 오는 피드백

CAN ID `0x130`, Standard CAN, DLC 8, 50 Hz입니다.

| Byte | 의미 |
|---|---|
| 0–1 | 측정 속도 `int16`, km/h × 100 |
| 2–3 | 측정 조향각 `int16`, degree × 10 |
| 4–5 | 조향 포텐셔미터 ADC raw |
| 6 | Sequence |
| 7 | `0xD0` + 상태 비트 |

상태 비트:

- bit 0: encoder speed valid
- bit 1: steering potentiometer valid
- bit 2: PC control active
- bit 3: E-STOP active

ROS2 bridge는 이를 `/aurix/feedback/*` 토픽으로 변환합니다.

## 실제 주행 전 확인할 AURIX 설정

1. `ControlLoop.c`에서는 연결된 PS2 조이스틱이 PC CAN보다 우선합니다.
   PS2가 연결되어 우선권이 활성화되면 정상 CAN 프레임도 액추에이터에
   적용되지 않습니다.
2. ZIP의 `Board_Pins.h`에는 현재 `ESTOP_ENABLE (0u)`가 설정되어 있습니다.
   즉, 실제 주행 전 하드웨어 E-STOP 정책과 펌웨어 설정을 별도로 확인해야
   합니다.
3. AURIX가 실제로 이 ZIP의 최신 빌드로 플래시되었는지 확인해야 합니다.
   소스 ZIP의 정합만으로 보드에 올라간 바이너리까지 보장되지는 않습니다.

## 검증된 바이트 벡터

```text
STOP, seq=7
00 00 00 00 7c 06 07 a0

DRIVE, 0.50 km/h, 0°, seq=1
32 00 00 00 7c 06 01 a1

DRIVE, 4.00 km/h, +27°, seq=255
90 01 0e 01 56 04 ff a1

DRIVE, 1.25 km/h, -27°, seq=0
7d 00 f2 fe 6c 07 00 a1
```

## 사용하면 안 되는 기존 송신 코드

현재 워크스페이스의 다음 파일은 `0x300` 바이트 배열이 이 TC275 계약과
다릅니다.

```text
src/gps_stanley_aurix_can_straight/scripts/
gps_stanley_aurix_can_straight_node.py
```

이 파일은 조향을 Byte 0–1에 넣고, 속도를 Byte 2–3에 넣으며, 펄스와
`0xA0/0xA1` Signature를 보내지 않습니다. 이번 TC275 펌웨어와 함께
사용하지 말고 `gps_waypoint_tracker_ros2/aurix_can_bridge.py`를 사용해야
합니다.
