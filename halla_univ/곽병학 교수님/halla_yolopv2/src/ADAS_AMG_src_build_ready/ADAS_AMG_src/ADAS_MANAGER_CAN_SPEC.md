# ADAS Manager CAN interface

This project receives five independent Classical CAN frames at 500 kbit/s,
11-bit identifier, DLC 8.

> Integration revision: the original PPT placed RequestStatus in B5 and a
> full uint8 Sequence in B6.  The current manager needs PassIndex, parking,
> traffic-light and selected-lane context, so the implemented wire contract
> uses **B5=AppContext** and **B6=Status[7:5]+Sequence[4:0]**.  Senders must use
> this document and `ADAS_MANAGER_5APP.dbc`; mixing the old PPT byte layout
> with this receiver is invalid.

| CAN ID | Application | FunctionId | v0 active B0 | v1 active B0 |
|---|---|---:|---:|---:|
| `0x310` | Minimum Risk Maneuver | 5 | `0x7D` | `0xFD` |
| `0x311` | Collision Avoidance | 3 | `0x7B` | `0xFB` |
| `0x312` | Auto Parking | 4 | `0x7C` | `0xFC` |
| `0x313` | Traffic Compliance, longitudinal only | 2 | `0x4A` | `0xCA` |
| `0x314` | Nominal Driving | 1 | `0x59` | `0xD9` |

## Common 8-byte payload

| Byte | Definition |
|---|---|
| B0 | bit7 FormatVersion, bit6 Enable, bit5 Atomic, bit4 LateralValid, bit3 LongitudinalValid, bit2:0 FunctionId |
| B1..2 | signed target speed, little-endian, `0.01 km/h` |
| B3..4 | signed target steering, little-endian, `0.1 deg`; positive=LEFT, negative=RIGHT |
| B5 | application context described below |
| B6 | bit7:5 RequestStatus, bit4:0 rolling Sequence (0..31) |
| B7 | CRC-8 |

Packet-safety limits follow the vehicle-verified `CAN_FIXED` command path:
speed is `-0.70..13.00 km/h` (reverse is limited to `-0.70 km/h`) and
steering is `-27.0..+27.0 deg`. A disabled/non-active packet must carry zero
speed and zero steering. Exact standard ID, DLC 8, CRC, function/header,
range and forward rolling-sequence checks must all pass.

RequestStatus is 0 Inactive, 1 Ready, 2 Active/Valid, 3
Active/Degraded, 4 Complete, 5 Abort, 6 Fault, 7 Invalid.

The receiver accepts the PPT v0 active headers and the corrected v1 headers.
CRC profile is CRC-8/SAE-J1850, poly `0x1D`, init `0xFF`, xorout `0xFF`,
non-reflected. CRC input order is CAN-ID high byte, CAN-ID low byte, then B0
through B6. During the current Nominal-only integration test, only ID `0x314`
with the v0 header and B7=`0x00` may use the sender's CRC placeholder. All
other applications, non-zero invalid CRCs, and every v1 packet still require
a valid CRC. Remove this narrow exception when the ROS2 sender implements the
CRC profile.

Golden vectors:

- Nominal v0: ID `0x314`, B0..B6 `59 90 01 00 00 00 40` -> B7 `0E`
- Nominal v1: ID `0x314`, B0..B6 `D9 90 01 00 00 00 40` -> B7 `AF`
- Collision v0: ID `0x311`, B0..B6 `7B C8 00 9C FF 36 41` -> B7 `64`
- Collision v1: ID `0x311`, B0..B6 `FB C8 00 9C FF 36 41` -> B7 `C5`

## B5 context

- Nominal: PassIndex `0..6`; `0xFF` is unresolved/invalid and must not be
  sent with Active status.
- Collision: bit1:0 ManeuverMode, bit4:2 ThreatLevel, bit6:5
  AvoidancePhase, bit7 ImmediateStop. Collision is accepted only as one
  atomic speed+steering pair.
- Parking: bit1:0 ParkingType, bit5:2 ParkingPhase, bit6 SlotValid, bit7
  StopHoldRequired.
- Traffic: bit2:0 TrafficSignal, bit4:3 SelectedLane, bit6:5
  IntersectionPhase, bit7 StopRequired.

PassIndex is 0 straight, 1 static obstacle, 2 hill, 3 dynamic
obstacle/acceleration, 4 parking, 5 intersection, 6 selected lane.

## Development-stage arbitration

- Every application starts as `INVALID`; `ReceivedMask=0` distinguishes a
  source that has never transmitted from one that transmitted bad data.
- Missing MRM, Collision, Parking, or Traffic does not block Nominal and does
  not create a fault.
- If Nominal is the only fresh Active/Valid source, its speed and steering
  are passed together to the existing VehicleManager.
- MRM and Collision can preempt in every segment. Their longitudinal and
  lateral values are never mixed with another application.
- Parking has segment priority in Pass 4. Traffic has longitudinal priority
  in Pass 5 and uses fresh Nominal lateral data when Traffic is long-only.
- Only a fresh frame with valid header, range and sequence participates; CRC
  must also be valid except for the temporary `0x314` v0/B7=`0x00` exception.
  A duplicate, replay, malformed frame, or an application that never sent a
  frame cannot win arbitration.
- `messageLost`, wrong DLC, bad CRC/header/range or replay immediately makes
  that application's current command INVALID; its previous command is not
  held until timeout. Once commissioned, the source must send a newer valid
  disabled heartbeat and then a newer Active frame to recover. A duplicate
  sequence never refreshes the watchdog.
- Every application has an independent validity state, sequence history and
  timeout. A bad or missing application never invalidates another valid app.
- After a source timeout, a valid disabled heartbeat must re-establish the
  sequence baseline before a newer Active frame can participate again.
- Once a valid ADAS frame has commissioned the ADAS input during a boot, a
  timeout stops through the existing CAN fail-safe; it never silently falls
  back to an old legacy `0x300` command.

Timeouts are 30 ms for Collision, 60 ms for MRM, and 150 ms for Nominal,
Traffic, and Parking. The five frames use independent RX mailboxes (MO20..24).
