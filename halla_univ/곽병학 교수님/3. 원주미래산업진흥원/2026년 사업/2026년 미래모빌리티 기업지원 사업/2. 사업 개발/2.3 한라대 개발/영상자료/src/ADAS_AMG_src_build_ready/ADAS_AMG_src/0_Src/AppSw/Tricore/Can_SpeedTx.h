#ifndef CAN_SPEED_TX_H_
#define CAN_SPEED_TX_H_

#include "Ifx_Types.h"
#include "Speed_Calculate.h"
#include "VehicleManager.h"

#define ACKERMANN_CAN_WHEEL_SPEED_ID   0x120U
#define ACKERMANN_CAN_STEERING_ID      0x121U
#define ACKERMANN_CAN_DRIVE_STATUS_ID  0x122U
#define ACKERMANN_CAN_ENCODER_COUNT_ID 0x123U
#define ACKERMANN_CAN_ENCODER_DEBUG_ID 0x124U
#define ACKERMANN_CAN_SPEED_CONTROL_ID 0x125U
#define ACKERMANN_CAN_RPM_DEBUG_ID     0x126U
#define ACKERMANN_CAN_STEER_FB_ID      0x127U
#define ACKERMANN_CAN_BUFFER_TEST_ID   0x128U
#define ACKERMANN_CAN_DEADBAND_TEST_ID 0x129U
#define ACKERMANN_CAN_DEADBAND_RESULT_ID 0x12AU
#define ACKERMANN_CAN_MAX_SPEED_TEST_ID 0x12BU
#define ACKERMANN_CAN_MAX_SPEED_RESULT_ID 0x12CU
#define ACKERMANN_CAN_JOYSTICK_CMD_ID 0x12DU
#define ACKERMANN_CAN_SENSOR_FEEDBACK_ID 0x130U
#define ACKERMANN_CAN_VM_STATUS_ID     0x131U
#define ACKERMANN_CAN_VM_TARGETS_ID    0x132U
#define ACKERMANN_CAN_VM_ENVELOPE_ID   0x133U
#define ACKERMANN_CAN_GRAPH_TEST_STATUS_ID 0x134U
#define ACKERMANN_CAN_PC_COMMAND_ID 0x300U

/* Vehicle Manager status byte-6 flags. */
#define ACKERMANN_CAN_VM_DRIVE_ENABLE_MASK          0x01u
#define ACKERMANN_CAN_VM_STEERING_VALID_MASK        0x02u
#define ACKERMANN_CAN_VM_LATERAL_LIMIT_MASK         0x04u
#define ACKERMANN_CAN_VM_STEERING_LIMIT_MASK        0x08u
#define ACKERMANN_CAN_VM_CAN_REARM_MASK             0x10u

/* Generic graph-test status byte-6 flags. */
#define ACKERMANN_CAN_GRAPH_TEST_ACTIVE_MASK          0x01u
#define ACKERMANN_CAN_GRAPH_TEST_SERVICE_OWNER_MASK   0x02u
#define ACKERMANN_CAN_GRAPH_TEST_SPEED_HEALTHY_MASK   0x04u
#define ACKERMANN_CAN_GRAPH_TEST_STEERING_HEALTHY_MASK 0x08u
#define ACKERMANN_CAN_GRAPH_TEST_LATERAL_LIMIT_MASK   0x10u
#define ACKERMANN_CAN_GRAPH_TEST_STEERING_LIMIT_MASK  0x20u
#define ACKERMANN_CAN_GRAPH_TEST_PHASE_EVENT_MASK     0x40u
#define ACKERMANN_CAN_GRAPH_TEST_COMPLETE_MASK        0x80u

/* TC275 -> PC joystick diagnostic, standard CAN, DLC 8.
 * byte 0:    pad present
 * byte 1:    receiver alive
 * byte 2..3: pressed-button mask, little-endian
 * byte 4..5: requested wheel RPM x10, signed little-endian
 * byte 6..7: requested steering pulse [us], little-endian
 */

/* TC275 -> PC consolidated sensor feedback, standard CAN, DLC 8,
 * little-endian, transmitted at 50 Hz.
 * byte 0..1: measured vehicle speed [0.01 km/h], signed
 * byte 2..3: potentiometer-derived measured steering [0.1 deg], signed
 * byte 4..5: steering potentiometer ADC raw
 * byte 6:    rolling sequence
 * byte 7:    0xD0 signature | bit3 E-stop | bit2 PC control active
 *            | bit1 steering potentiometer valid | bit0 encoder speed valid
 */
#define ACKERMANN_CAN_SENSOR_FEEDBACK_SIGNATURE        0xD0u
#define ACKERMANN_CAN_SENSOR_FEEDBACK_SPEED_VALID_MASK 0x01u
#define ACKERMANN_CAN_SENSOR_FEEDBACK_STEERING_VALID_MASK 0x02u
#define ACKERMANN_CAN_SENSOR_FEEDBACK_PC_ACTIVE_MASK   0x04u
#define ACKERMANN_CAN_SENSOR_FEEDBACK_ESTOP_MASK       0x08u

/* PC -> TC275 command, standard CAN, DLC 8, little-endian.
 * byte 0..1: signed speed [0.01 km/h], -70..1300
 *             negative = reverse, -70 means -0.70 km/h
 * byte 2..3: steering [0.1 deg], signed, +LEFT / -RIGHT, -270..270
 * byte 4..5: expected steering servo pulse [us], 1300..1950
 * byte 6:    rolling sequence
 * byte 7:    0xA0 signature | bit0 enable (bits 1..3 must be zero)
 */
#define ACKERMANN_CAN_PC_COMMAND_TIMEOUT_MS       250u
#define ACKERMANN_CAN_PC_COMMAND_MIN_SPEED_X100   (-70)
#define ACKERMANN_CAN_PC_COMMAND_MAX_SPEED_X100   1300
#define ACKERMANN_CAN_PC_COMMAND_MAX_STEER_X10    270
#define ACKERMANN_CAN_PC_COMMAND_SIGNATURE        0xA0u
#define ACKERMANN_CAN_PC_COMMAND_ENABLE_MASK      0x01u

void CanSpeedTx_Init(void);
void CanSpeedTx_PollReceive(void);
void CanSpeedTx_InvalidatePcCommandForManualOverride(void);
void CanSpeedTx_SendWheelSpeed(const VehicleSpeedValue *speed);
void CanSpeedTx_SendRpmDebug(const VehicleSpeedValue *speed);
void CanSpeedTx_SendDriveStatus(uint8 dutyPercent,
                                uint8 driveDirection,
                                uint8 estopActive,
                                uint8 speedBufferCount,
                                sint32 leftDeltaCount,
                                sint32 rightDeltaCount);
void CanSpeedTx_SendEncoderCount(sint32 leftCount, sint32 rightCount);
void CanSpeedTx_SendEncoderDebug(uint16 leftHwCount,
                                 uint16 rightHwCount,
                                 sint32 leftHwDelta,
                                 sint32 rightHwDelta);
void CanSpeedTx_SendSpeedControlStatus(float32 targetRpm,
                                       float32 currentRpm,
                                       float32 dutyCommand,
                                       float32 feedforwardDuty);
void CanSpeedTx_SendSteeringStatus(float32 targetDeg,
                                   float32 measuredDeg,
                                   uint16 adcRaw,
                                   uint16 pulseUs,
                                   uint8 stateCode);
void CanSpeedTx_SendSteeringFeedback(float32 currentDeg,
                                     uint16 adcRaw,
                                     float32 adcMv,
                                     float32 errorDeg);
void CanSpeedTx_SendSensorFeedback(const VehicleSpeedValue *speed,
                                   float32 measuredSteeringDeg,
                                   uint16 steeringPotAdcRaw,
                                   uint8 steeringPotValid,
                                   uint8 pcControlActive,
                                   uint8 estopActive);
void CanSpeedTx_SendVehicleManagerTelemetry(const VehicleManagerOutput *output);
void CanSpeedTx_SendGraphTestStatus(uint8 mode,
                                    uint8 state,
                                    uint8 phase,
                                    uint8 abortReason,
                                    uint16 phaseElapsedMs,
                                    uint8 flags);
void CanSpeedTx_SendBufferTestStatus(uint8 bufferSize,
                                     uint8 stageIndex,
                                     uint8 dutyPercent,
                                     uint8 statusCode,
                                     float32 avgVehicleRpm,
                                     uint16 elapsedMs);
void CanSpeedTx_SendDriveDeadbandTestStatus(uint8 testCaseId,
                                            uint8 statusCode,
                                            uint8 currentDutyPercent,
                                            uint8 lastStoppedDutyPercent,
                                            uint8 firstMovingDutyPercent,
                                            float32 currentRpm,
                                            uint8 elapsedSec);
void CanSpeedTx_SendDriveDeadbandResult(uint8 testCaseId,
                                        uint8 statusCode,
                                        uint8 lastStoppedDutyPercent,
                                        uint8 firstMovingDutyPercent,
                                        float32 firstMovingRpm,
                                        float32 firstMovingKph);
void CanSpeedTx_SendMaxSpeedTestStatus(uint8 testCaseId,
                                       uint8 statusCode,
                                       uint8 phase,
                                       uint8 dutyPercent,
                                       float32 currentRpm,
                                       float32 currentKph);
void CanSpeedTx_SendMaxSpeedResult(uint8 testCaseId,
                                   uint8 statusCode,
                                   uint8 dutyPercent,
                                   uint8 elapsedSec,
                                   float32 avgRpm,
                                   float32 avgKph);
void CanSpeedTx_SendJoystickCommand(uint8 present,
                                    uint8 receiverAlive,
                                    uint16 buttons,
                                    float32 targetRpm,
                                    uint16 steeringPulseUs);

extern volatile uint8  g_canInitDone;
extern volatile uint8  g_canTransceiverNENLow;
extern volatile uint32 g_canNodeInitStatus;
extern volatile uint32 g_canMsgObjInitStatus;
extern volatile uint32 g_canTxAttemptCount;
extern volatile uint32 g_canTxOkCount;
extern volatile uint32 g_canTxBusyCount;
extern volatile uint32 g_canTxErrorCount;
extern volatile uint32 g_canLastTxStatus;
extern volatile uint32 g_canLastTxId;
extern volatile uint32 g_canLastDataLow;
extern volatile uint32 g_canLastDataHigh;
extern volatile uint8  g_canRxAlive;
extern volatile uint32 g_canRxCount;
extern volatile uint32 g_canRxEmptyCount;
extern volatile uint32 g_canRxLostCount;
extern volatile uint32 g_canRxLastStatus;
extern volatile uint32 g_canRxLastId;
extern volatile uint32 g_canRxLastDataLow;
extern volatile uint32 g_canRxLastDataHigh;
extern volatile uint32 g_canRxLastTick100us;
extern volatile uint32 g_canRxLastAgeMs;
extern volatile uint8  g_canBusOffRecoveryActive;
extern volatile uint32 g_canBusOffDetectCount;
extern volatile uint32 g_canBusOffRecoveryAttemptCount;
extern volatile uint32 g_canBusOffRecoveryOkCount;
extern volatile uint32 g_canBusOffLastStatus;
extern volatile uint32 g_sensorFeedbackTxOkCount;
extern volatile uint32 g_sensorFeedbackTxRetryCount;
extern volatile uint32 g_vehicleManagerTelemetryTxOkCount;
extern volatile uint32 g_vehicleManagerTelemetryTxRetryCount;
extern volatile uint8  g_vehicleManagerTelemetrySequence;
extern volatile uint32 g_graphTestStatusTxOkCount;
extern volatile uint32 g_graphTestStatusTxRetryCount;
extern volatile uint8  g_graphTestStatusSequence;
/* bits 0..4: MRM, Collision, Parking, Traffic, Nominal RX object init. */
extern volatile uint8  g_adasCanRxInitOkMask;
extern volatile uint8  g_adasCanRxInitErrorMask;

/* Validated PC command shadow. ControlLoop owns actuator arbitration. */
extern volatile uint8  g_pcCommandModeSeen;
extern volatile uint8  g_pcCommandValid;
extern volatile uint8  g_pcCommandEnabled;
extern volatile sint16 g_pcCommandSpeedKphX100;
extern volatile sint16 g_pcCommandSteeringDegX10;
extern volatile uint16 g_pcCommandSteeringPulseUs;
extern volatile uint8  g_pcCommandSequence;
extern volatile uint32 g_pcCommandLastValidTick100us;
extern volatile uint32 g_pcCommandAgeMs;
extern volatile uint32 g_pcCommandValidCount;
extern volatile uint32 g_pcCommandInvalidCount;
extern volatile uint32 g_pcCommandDuplicateCount;
extern volatile uint32 g_pcCommandWatchdogCount;
extern volatile uint32 g_pcCommandManualBlockCount;

#endif /* CAN_SPEED_TX_H_ */
