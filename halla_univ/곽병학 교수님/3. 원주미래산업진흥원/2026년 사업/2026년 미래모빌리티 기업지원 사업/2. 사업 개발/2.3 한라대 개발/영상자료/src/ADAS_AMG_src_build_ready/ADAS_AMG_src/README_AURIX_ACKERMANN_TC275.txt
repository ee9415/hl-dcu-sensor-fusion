AURIX_ACKERMANN_TC275
=====================

Target board
------------
- Infineon AURIX TC275 Lite Kit
- Project skeleton converted from the provided AURIX example for an Ackermann car

Wiring
------
- Encoder A PhA -> P02.6
- Encoder A PhB -> P02.7
- Encoder B PhA -> P33.7 / GPT120_T2INB / X2 pin 35
- Encoder B PhB -> P33.6 / GPT120_T2EUDB / X2 pin 34
- Encoder VCC/GND -> 3.3 V logic recommended, common GND required

- Steering feedback ADC -> AN1
- Steering feedback VCC -> 3.3 V
- Steering feedback GND -> common GND

- Steering servo PWM signal -> mikroBus PWM / P02.8
- Steering servo VCC -> external 5 V to 6 V BEC recommended
- Steering servo GND -> common GND with TC275

- Drive motor driver top row
  - VCC  -> TC275 3.3 V logic supply, if the driver supports 3.3 V logic
  - PWMA -> P02.0 / TOUT0
  - INA1 -> P33.8
  - INA2 -> P33.9
  - GND  -> common GND with TC275

- Drive motor driver bottom row
  - VCC  -> same logic VCC as top row
  - PWMB -> P02.1 / TOUT1
  - INB1 -> P33.10
  - INB2 -> P33.11
  - GND  -> common GND with TC275

- UART to ROS PC
  - RX -> P14.0
  - TX -> P14.1

- E-STOP -> P20.6, LOW = stop, internal pull-up enabled
- Battery voltage ADC -> P40.1 after external divider, max 5 V at the pin

Important
---------
- P00.6 is not ADC on the TC275 Lite Kit. Move the steering feedback wire to AN1.
- Do not feed 5 V encoder A/B signals directly into TC275 GPIO.
- Left encoder uses GPT12 hardware counter inputs T3IN/T3EUD on P02.6/P02.7.
- Right encoder uses GPT12 hardware counter inputs T2IN/T2EUD on P33.7/P33.6.
- UART pins are reserved as requested, but the local iLLD ASCLIN0 hardware RX map is P14.1, not P14.0.
- Servo power should not come directly from the TC275 board.
- The car runs closed-loop speed control. The power-on target is 0 km/h.
- Speed feedforward is calibrated from 100% duty = 336 rpm.
- Keep the wheels off the ground for the first upload.

Runtime
-------
- GPT12 T6 interrupt runs every 100 us.
- Left and right encoders are counted by GPT12 hardware in EncoderService_100us().
- Main control loop runs every 5 ms.
- Speed is calculated from the encoder count difference over each fixed 5 ms control period.
- Steering ADC is read from AN1 through VADC group 0 channel 1.
- Steering servo PWM is 50 Hz on GTM ATOM0 channel 0 / P02.8.
- Drive PWM is 20 kHz on GTM ATOM1 channel 0 / P02.0 and channel 1 / P02.1.
- Drive output is controlled by a feedforward + PID speed loop.
- Steering is enabled when DRIVE_ONLY_TEST = 0.
- Default control target is center steering: g_targetSteeringDeg = 0.0.
- Steering feedback calibration is -27 deg (right) through +27 deg (left).

Calibration
-----------
- Steering_ADC.h
  - STEERING_ADC_RAW_MIN
  - STEERING_ADC_RAW_CENTER
  - STEERING_ADC_RAW_MAX
  - STEERING_MAX_ANGLE_DEG

- CCU6_PWM_Generation.h
  - STEERING_SERVO_LEFT_US
  - STEERING_SERVO_CENTER_US
  - STEERING_SERVO_RIGHT_US

- ControlLoop.c
  - SPEED_CONTROL_MAX_RPM_AT_100_DUTY = 336.0 rpm
  - STEERING_TARGET_LIMIT_DEG is the legacy angle-loop clamp.
  - PS2 and PC CAN commands use the steering-pot target and are limited to +/-27 deg.
  - SPEED_CALIBRATION_FIXED_DUTY_ENABLE, set to 1 only for max RPM measurement
  - SPEED_CALIBRATION_FIXED_DUTY_PERCENT = 100.0
  - SPEED_TARGET_KMH
  - SPEED_CONTROL_MAX_DUTY
  - SPEED_PID_KP_DUTY_PER_RPM
  - SPEED_PID_KI_DUTY_PER_RPM_SEC
  - SPEED_PID_KD_DUTY_PER_RPM_PER_SEC
  - Normal nonzero duty command ramp = 1 % per 5 ms control loop.
  - A zero/fault command bypasses the ramp and removes PWM immediately (coast).

- Speed_Calculate.h
  - ENCODER_PPR
  - MOTOR_GEAR_RATIO
  - SPEED_BUFFER_FIXED_COUNT = 20 (100 ms at the 5 ms sample period)
  - SPEED_SAMPLE_TICK100US = 50, fixed 5 ms speed sample period
  - COUNTS_PER_REV = 39600 confirmed (550 PPR x4 x 18 gear ratio)
  - WHEEL_DIAMETER_M = 0.265 m

CAN
---
- CANoe setup
  - Baudrate: 500 kbit/s
  - Standard IDs
  - TC275 CAN TX: P20.8, CAN RX: P20.7
  - CANoe test mode uses P20.6 as CAN transceiver NEN, so E-STOP is disabled in Board_Pins.h.

- 0x120 WheelSpeed
  - LeftRpmX10
  - RightRpmX10
  - VehicleMpsX1000
  - SpeedValid

- 0x121 SteeringStatus
  - TargetSteeringDegX10
  - MeasuredSteeringDegX10
  - SteeringAdcRaw
  - ServoPulseOffset5us
  - AckermannState

- 0x122 DriveStatus
  - DriveDutyPercent
  - DriveDirection
  - EStopActive
  - SpeedBufferCount
  - LeftDeltaCount
  - RightDeltaCount

- 0x123 EncoderCount
  - LeftEncoderCount
  - RightEncoderCount

- 0x124 EncoderDebug
  - LeftHwCount
  - RightHwCount
  - LeftHwDelta
  - RightHwDelta

- 0x125 SpeedControlStatus
  - TargetVehicleRpmX10
  - CurrentVehicleRpmX10
  - DutyCommandX10
  - FeedforwardDutyX10

- 0x126 RpmDebug
  - LeftRawRpmX10
  - LeftAvgRpmX10
  - RightRawRpmX10
  - RightAvgRpmX10

- 0x130 SensorFeedback (TC275 -> PC, standard CAN, DLC 8, little-endian, 50 Hz)
  - byte 0..1: encoder-derived measured speed x100 [km/h], signed
  - byte 2..3: potentiometer-derived measured steering x10 [deg], signed
  - byte 4..5: steering potentiometer ADC raw
  - byte 6: rolling feedback sequence
  - byte 7: 0xD0 signature plus encoder-valid bit0, steering-valid bit1,
            PC-control-active bit2, E-STOP-active bit3
  - the PC drive path requires a fresh frame with both sensor-valid bits set
  - this single filtered frame avoids using the high-rate diagnostic frames as
    a control heartbeat

- 0x131 VehicleManagerStatus (TC275 -> PC, standard CAN, DLC 8, little-endian, 50 Hz)
  - byte 0: Vehicle Manager state (Init=0, Standby=1, Centering=2,
            ManualPs2=3, AutoCan=4, CanRearm=5, FaultTorqueOff=6, Estop=7,
            ServiceSpeedTest=8, Validation=9)
  - byte 1: owner (None=0, Ps2=1, Can=2, ServiceTest=3)
  - byte 2..5: full Vehicle Manager fault mask, uint32 little-endian
  - byte 6: bit0 drive enable, bit1 steering target valid,
            bit2 lateral-speed limit active, bit3 steering limit active,
            bit4 CAN rearm required
  - byte 7: rolling telemetry sequence

- 0x132 VehicleManagerTargets (TC275 -> PC, standard CAN, DLC 8, little-endian, 50 Hz)
  - byte 0..1: post-source-clamp requested speed x100 [km/h], signed
  - byte 2..3: final target speed x100 [km/h], signed
  - byte 4..5: post-range-clamp requested steering x10 [deg], signed
  - byte 6..7: final target steering x10 [deg], signed

- 0x133 VehicleManagerEnvelope (TC275 -> PC, standard CAN, DLC 8, little-endian, 50 Hz)
  - byte 0..1: effective steering magnitude x10 [deg]
  - byte 2..3: speed-dependent steering limit x10 [deg]
  - byte 4..5: curvature-dependent speed limit x100 [km/h]
  - byte 6..7: predicted lateral acceleration x1000 [m/s2]
  - curvature [1/m] remains available as g_vehicleManagerCurvatureInvM in the debugger
  - the three Vehicle Manager frames use one coherent snapshot; a busy mailbox
    is retried without consuming the 20 ms period or rolling sequence
  - after bus-off recovery, any pending snapshot is discarded and the current
    Vehicle Manager result is captured immediately

- 0x134 GraphTestStatus (TC275 -> PC, standard CAN, DLC 8, little-endian, 50 Hz)
  - byte 0..3: mode, state, phase, abort reason
  - byte 4..5: phase elapsed time [ms], uint16 little-endian
  - byte 6: bit0 active, bit1 service owner, bit2 speed healthy,
            bit3 steering healthy, bit4 lateral limit, bit5 steering limit,
            bit6 phase-transition event, bit7 complete
  - byte 7: rolling sequence; increments only after mailbox send success
  - modes: None=0, SpeedTracking=1, VmValidationActuation=2
  - states: Idle=0, Running=1, Complete=2, Aborted=3, Rejected=4
  - SpeedTracking phase 1..13 is the straight-line km/h step (5 s each)
  - VmValidationActuation phases are (speed km/h, steering deg), 5 s each:
    1=(3,0), 2=(3,+20), 3=(5,+20), 4=(8,+10),
    5=(13,0), 6=(13,+5), 7=(0,0)
  - mode 2 uses actual feedback, normal VM envelope, speed PI and steering PID;
    it is an actuator test and must be run only with the vehicle safely lifted
  - a phase event bypasses the periodic gate and is retained across a busy retry

- 0x300 AckermannCommand (PC -> TC275, standard CAN, DLC 8, little-endian)
  - byte 0..1: TargetSpeedKph x100, signed, -70..1300 (-0.70 reverse..+13.00 km/h forward)
  - byte 2..3: TargetSteeringDeg x10, signed, -270..+270
  - byte 4..5: redundant SteeringPulseUs
  - byte 6: rolling CommandSequence
  - byte 7: 0xA0 signature plus bit0 enable; bits 1..3 must be zero
  - steering sign is +LEFT / -RIGHT
  - +27 deg = 1150 us / POT 4095, 0 deg = 1500 us / POT 2929,
    -27 deg = 1800 us / POT 922
  - pulse and angle must agree within 3 us or the command is rejected
  - valid enabled example: +0.60 km/h, 0 deg, sequence 1 = 3C 00 00 00 DC 05 01 A1
  - valid reverse example: -0.70 km/h, 0 deg, sequence 42 = BA FF 00 00 DC 05 2A A1
  - canonical STOP example, sequence 2 = 00 00 00 00 DC 05 02 A0

PC command safety/arbitration
-----------------------------
- Only correctly signed 0x300 frames are accepted. The old all-zero 0x300 heartbeat is invalid.
- A duplicate sequence does not refresh the command watchdog.
- A valid unique command must arrive within 250 ms; otherwise drive stops and steering returns to center.
- A malformed 0x300 frame after PC command mode has started causes immediate stop/center.
- The PS2 wireless joystick always has priority. Its first bad packet requests zero speed immediately while ownership remains latched through the disconnect confirmation window.
- PS2 speed test mapping:
  - D-pad UP rising edge: selected speed 0 -> 1 -> 2 ... -> 13 -> 1 km/h.
  - CROSS (X), while held: drive at the selected integer speed; releasing it requests zero speed.
  - D-pad DOWN: -1.0 km/h reverse; SELECT: zero speed.
  - TRIANGLE/SQUARE/CIRCLE retain their existing 4.0/2.0/0.6 km/h mappings.
  - Speed selection is blocked while a traction/stop button is held, so UP cannot change the target during a run.
- After joystick disconnect or a safety fault, CAN control requires a new canonical disabled frame followed by a forward-sequence enabled frame.
- When PC command mode has never been entered, existing debugger/unit-test operation is preserved.
- Do not run the legacy ROS2 all-zero 0x300 alive publisher together with PC lane control.

Vehicle Manager
---------------
- VehicleManager.c and VehicleManager.h contain the standalone deterministic manager core.
- The core runs in the existing CPU0 5 ms path; a 50 ms CAN command is held as the latest validated request between receptions.
- Vehicle dimensions: front track A = 0.51 m, wheelbase B = 0.44 m.
- Ownership priority: E-STOP, feedback fault, PS2, CAN, then stationary service tests.
- CAN and PS2 forward requests are capped at 13.0 km/h; PS2 reverse is capped at 1.0 km/h and CAN reverse remains capped at 0.7 km/h.
- Steering is capped at +/-27 deg. The manager limits both speed and steering to a provisional 1.5 m/s2 lateral-acceleration envelope.
- Until the potentiometer angle is identified as inner-wheel, outer-wheel, or bicycle-equivalent, curvature uses the conservative outer-wheel interpretation with the 0.51 m track.
- Vehicle speed is the mean of both rear-wheel encoder magnitudes; both feedback channels must be valid.
- Fault/timeout torque action is PWM zero (coast), with closed-loop center return when steering feedback is healthy.
- Forward/reverse changes reset duty and PID, then require measured speed below 5 rpm for 100 ms before ramping from zero.
