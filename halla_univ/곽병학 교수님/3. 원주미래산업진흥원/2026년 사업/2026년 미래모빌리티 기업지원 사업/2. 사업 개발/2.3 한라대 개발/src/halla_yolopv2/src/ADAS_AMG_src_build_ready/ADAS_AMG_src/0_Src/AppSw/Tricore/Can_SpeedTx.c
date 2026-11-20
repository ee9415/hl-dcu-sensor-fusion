#include "Can_SpeedTx.h"

#include "Adas_Manager.h"
#include "Board_Pins.h"
#include "CCU6_PWM_Generation.h"
#include "Can_TrajectoryFd.h"
#include "GPT12_Timer_Interrupt.h"
#include "IfxPort.h"
#include "Multican/Can/IfxMultican_Can.h"

#define ACKERMANN_CAN_BAUDRATE       500000U
#define ACKERMANN_CAN_NODE_ID        IfxMultican_NodeId_0
#define ACKERMANN_CAN_RX_PIN         (&IfxMultican_RXD0B_P20_7_IN)
#define ACKERMANN_CAN_TX_PIN         (&IfxMultican_TXD0_P20_8_OUT)
#define ACKERMANN_CAN_SPEED_OBJ_ID   0u
#define ACKERMANN_CAN_STEER_OBJ_ID   1u
#define ACKERMANN_CAN_DRIVE_OBJ_ID   2u
#define ACKERMANN_CAN_ENCODER_OBJ_ID 3u
#define ACKERMANN_CAN_ENCDBG_OBJ_ID  4u
#define ACKERMANN_CAN_SPEED_CTRL_OBJ_ID 5u
#define ACKERMANN_CAN_RPM_DEBUG_OBJ_ID 6u
#define ACKERMANN_CAN_STEER_FB_OBJ_ID 7u
#define ACKERMANN_CAN_BUFFER_TEST_OBJ_ID 8u
#define ACKERMANN_CAN_DEADBAND_TEST_OBJ_ID 9u
#define ACKERMANN_CAN_DEADBAND_RESULT_OBJ_ID 10u
#define ACKERMANN_CAN_MAX_SPEED_TEST_OBJ_ID 11u
#define ACKERMANN_CAN_MAX_SPEED_RESULT_OBJ_ID 12u
#define ACKERMANN_CAN_JOYSTICK_CMD_OBJ_ID 13u
#define ACKERMANN_CAN_PC_COMMAND_OBJ_ID 14u
#define ACKERMANN_CAN_SENSOR_FEEDBACK_OBJ_ID 15u
#define ACKERMANN_CAN_VM_STATUS_OBJ_ID  16u
#define ACKERMANN_CAN_VM_TARGETS_OBJ_ID 17u
#define ACKERMANN_CAN_VM_ENVELOPE_OBJ_ID 18u
#define ACKERMANN_CAN_GRAPH_TEST_STATUS_OBJ_ID 19u
#define ACKERMANN_CAN_ADAS_MRM_OBJ_ID       20u
#define ACKERMANN_CAN_ADAS_COLLISION_OBJ_ID 21u
#define ACKERMANN_CAN_ADAS_PARKING_OBJ_ID   22u
#define ACKERMANN_CAN_ADAS_TRAFFIC_OBJ_ID   23u
/* 24u는 예전 classic-CAN Nominal(0x314) RX 오브젝트가 쓰던 번호. v7에서
 * Nominal 슬롯 자체가 없어지면서(카메라/GPS가 TRAJ_FD로 각자 슬롯을 직접
 * 채움) 더 이상 안 씀 - TRAJ_FD_CAMERA_MSG_OBJ_ID(25)/TRAJ_FD_GPS_MSG_OBJ_ID(26)와
 * 안 겹치게 자리만 비워둠. */
#define ACKERMANN_CAN_RX_ALIVE_TIMEOUT_MS 500u
#define ACKERMANN_CAN_SENSOR_FEEDBACK_PERIOD_TICK100US 200u
#define ACKERMANN_CAN_VM_TELEMETRY_PERIOD_TICK100US 200u
#define ACKERMANN_CAN_VM_PENDING_STATUS   0x01u
#define ACKERMANN_CAN_VM_PENDING_TARGETS  0x02u
#define ACKERMANN_CAN_VM_PENDING_ENVELOPE 0x04u
#define ACKERMANN_CAN_VM_PENDING_ALL      0x07u
#define ACKERMANN_CAN_GRAPH_TEST_STATUS_PERIOD_TICK100US 200u

static IfxMultican_Can        g_canModule;
static IfxMultican_Can_Node   g_canNode;
static IfxMultican_Can_MsgObj g_txObjSpeed;
static IfxMultican_Can_MsgObj g_txObjSteering;
static IfxMultican_Can_MsgObj g_txObjDriveStatus;
static IfxMultican_Can_MsgObj g_txObjEncoderCount;
static IfxMultican_Can_MsgObj g_txObjEncoderDebug;
static IfxMultican_Can_MsgObj g_txObjSpeedControl;
static IfxMultican_Can_MsgObj g_txObjRpmDebug;
static IfxMultican_Can_MsgObj g_txObjSteeringFeedback;
static IfxMultican_Can_MsgObj g_txObjBufferTest;
static IfxMultican_Can_MsgObj g_txObjDeadbandTest;
static IfxMultican_Can_MsgObj g_txObjDeadbandResult;
static IfxMultican_Can_MsgObj g_txObjMaxSpeedTest;
static IfxMultican_Can_MsgObj g_txObjMaxSpeedResult;
static IfxMultican_Can_MsgObj g_txObjJoystickCommand;
static IfxMultican_Can_MsgObj g_rxObjPcCommand;
static IfxMultican_Can_MsgObj g_txObjSensorFeedback;
static IfxMultican_Can_MsgObj g_txObjVehicleManagerStatus;
static IfxMultican_Can_MsgObj g_txObjVehicleManagerTargets;
static IfxMultican_Can_MsgObj g_txObjVehicleManagerEnvelope;
static IfxMultican_Can_MsgObj g_txObjGraphTestStatus;
static IfxMultican_Can_MsgObj g_rxObjAdasMrm;
static IfxMultican_Can_MsgObj g_rxObjAdasCollision;
static IfxMultican_Can_MsgObj g_rxObjAdasParking;
static IfxMultican_Can_MsgObj g_rxObjAdasTraffic;

volatile uint8  g_canInitDone = 0u;
volatile uint8  g_canTransceiverNENLow = 0u;
volatile uint32 g_canNodeInitStatus = 0xFFFFFFFFu;
volatile uint32 g_canMsgObjInitStatus = 0xFFFFFFFFu;
volatile uint32 g_canTxAttemptCount = 0u;
volatile uint32 g_canTxOkCount = 0u;
volatile uint32 g_canTxBusyCount = 0u;
volatile uint32 g_canTxErrorCount = 0u;
volatile uint32 g_canLastTxStatus = 0xFFFFFFFFu;
volatile uint32 g_canLastTxId = 0u;
volatile uint32 g_canLastDataLow = 0u;
volatile uint32 g_canLastDataHigh = 0u;
volatile uint8  g_canRxAlive = 0u;
volatile uint32 g_canRxCount = 0u;
volatile uint32 g_canRxEmptyCount = 0u;
volatile uint32 g_canRxLostCount = 0u;
volatile uint32 g_canRxLastStatus = 0xFFFFFFFFu;
volatile uint32 g_canRxLastId = 0u;
volatile uint32 g_canRxLastDataLow = 0u;
volatile uint32 g_canRxLastDataHigh = 0u;
volatile uint32 g_canRxLastTick100us = 0u;
volatile uint32 g_canRxLastAgeMs = 0xFFFFFFFFu;
volatile uint8  g_canBusOffRecoveryActive = 0u;
volatile uint32 g_canBusOffDetectCount = 0u;
volatile uint32 g_canBusOffRecoveryAttemptCount = 0u;
volatile uint32 g_canBusOffRecoveryOkCount = 0u;
volatile uint32 g_canBusOffLastStatus = 0xFFFFFFFFu;
volatile uint32 g_sensorFeedbackTxOkCount = 0u;
volatile uint32 g_sensorFeedbackTxRetryCount = 0u;
volatile uint32 g_vehicleManagerTelemetryTxOkCount = 0u;
volatile uint32 g_vehicleManagerTelemetryTxRetryCount = 0u;
volatile uint8  g_vehicleManagerTelemetrySequence = 0u;
volatile uint32 g_graphTestStatusTxOkCount = 0u;
volatile uint32 g_graphTestStatusTxRetryCount = 0u;
volatile uint8  g_graphTestStatusSequence = 0u;
volatile uint8  g_adasCanRxInitOkMask = 0u;
volatile uint8  g_adasCanRxInitErrorMask = 0u;
volatile uint8  g_pcCommandModeSeen = 0u;
volatile uint8  g_pcCommandValid = 0u;
volatile uint8  g_pcCommandEnabled = 0u;
volatile sint16 g_pcCommandSpeedKphX100 = 0;
volatile sint16 g_pcCommandSteeringDegX10 = 0;
volatile uint16 g_pcCommandSteeringPulseUs = STEERING_SERVO_CENTER_US;
volatile uint8  g_pcCommandSequence = 0u;
volatile uint32 g_pcCommandLastValidTick100us = 0u;
volatile uint32 g_pcCommandAgeMs = 0xFFFFFFFFu;
volatile uint32 g_pcCommandValidCount = 0u;
volatile uint32 g_pcCommandInvalidCount = 0u;
volatile uint32 g_pcCommandDuplicateCount = 0u;
volatile uint32 g_pcCommandWatchdogCount = 0u;
volatile uint32 g_pcCommandManualBlockCount = 0u;

static uint8 g_pcCommandHasSequence = 0u;
static uint8 g_pcCommandWatchdogLatched = 0u;
static uint8 g_sensorFeedbackSequence = 0u;
static uint8 g_sensorFeedbackScheduleValid = 0u;
static uint32 g_sensorFeedbackLastTick100us = 0u;
static uint8 g_vehicleManagerTelemetryScheduleValid = 0u;
static uint8 g_vehicleManagerTelemetryPendingMask = 0u;
static uint32 g_vehicleManagerTelemetryLastTick100us = 0u;
static uint8 g_vehicleManagerStatusData[8] = {0u};
static uint8 g_vehicleManagerTargetsData[8] = {0u};
static uint8 g_vehicleManagerEnvelopeData[8] = {0u};
static uint8 g_graphTestStatusScheduleValid = 0u;
static uint8 g_graphTestStatusPending = 0u;
static uint8 g_graphTestStatusPendingIsEvent = 0u;
static uint32 g_graphTestStatusLastTick100us = 0u;
static uint8 g_graphTestStatusData[8] = {0u};

static void CanSpeedTx_EnableTransceiver(void)
{
#if (CAN_TRANSCEIVER_ENABLE != 0u)
    IfxPort_setPinModeOutput(CAN_TRANSCEIVER_NEN_PORT,
                             CAN_TRANSCEIVER_NEN_PIN,
                             IfxPort_OutputMode_pushPull,
                             IfxPort_OutputIdx_general);
    IfxPort_setPinLow(CAN_TRANSCEIVER_NEN_PORT, CAN_TRANSCEIVER_NEN_PIN);
    g_canTransceiverNENLow = 1u;
#endif
}

static sint16 CanSpeedTx_ClampFloatToS16(float32 value)
{
    if (value > 32767.0f)
    {
        return 32767;
    }

    if (value < -32768.0f)
    {
        return -32768;
    }

    return (sint16)value;
}

static uint16 CanSpeedTx_ClampFloatToU16(float32 value)
{
    if (value > 65535.0f)
    {
        return 65535u;
    }

    if (value < 0.0f)
    {
        return 0u;
    }

    return (uint16)value;
}

static sint16 CanSpeedTx_ClampS32ToS16(sint32 value)
{
    if (value > 32767)
    {
        return 32767;
    }

    if (value < -32768)
    {
        return -32768;
    }

    return (sint16)value;
}

static uint8 CanSpeedTx_EncodePulseOffset5us(uint16 pulseUs)
{
    uint16 offset;

    if (pulseUs < 900u)
    {
        pulseUs = 900u;
    }
    else if (pulseUs > 2175u)
    {
        pulseUs = 2175u;
    }

    offset = (uint16)((pulseUs - 900u) / 5u);
    return (uint8)offset;
}

static uint32 CanSpeedTx_PackU32Le(const uint8 *data)
{
    return ((uint32)data[0]) |
           ((uint32)data[1] << 8) |
           ((uint32)data[2] << 16) |
           ((uint32)data[3] << 24);
}

static void CanSpeedTx_PutS16(uint8 *data, uint8 index, sint16 value)
{
    data[index] = (uint8)((uint16)value & 0xFFu);
    data[index + 1u] = (uint8)(((uint16)value >> 8u) & 0xFFu);
}

static void CanSpeedTx_PutU16(uint8 *data, uint8 index, uint16 value)
{
    data[index] = (uint8)(value & 0xFFu);
    data[index + 1u] = (uint8)((value >> 8u) & 0xFFu);
}

static void CanSpeedTx_PutS32(uint8 *data, uint8 index, sint32 value)
{
    uint32 raw = (uint32)value;

    data[index] = (uint8)(raw & 0xFFu);
    data[index + 1u] = (uint8)((raw >> 8u) & 0xFFu);
    data[index + 2u] = (uint8)((raw >> 16u) & 0xFFu);
    data[index + 3u] = (uint8)((raw >> 24u) & 0xFFu);
}

static void CanSpeedTx_PutU32(uint8 *data, uint8 index, uint32 value)
{
    data[index] = (uint8)(value & 0xFFu);
    data[index + 1u] = (uint8)((value >> 8u) & 0xFFu);
    data[index + 2u] = (uint8)((value >> 16u) & 0xFFu);
    data[index + 3u] = (uint8)((value >> 24u) & 0xFFu);
}

static uint16 CanSpeedTx_PcCommandExpectedPulseUs(sint16 steeringDegX10)
{
    uint32 magnitude;
    uint32 offset;

    if (steeringDegX10 >= 0)
    {
        magnitude = (uint32)steeringDegX10;
        offset = ((magnitude * (uint32)(STEERING_SERVO_CENTER_US - STEERING_SERVO_LEFT_US)) +
                  ((uint32)ACKERMANN_CAN_PC_COMMAND_MAX_STEER_X10 / 2u)) /
                 (uint32)ACKERMANN_CAN_PC_COMMAND_MAX_STEER_X10;
        return (uint16)((uint32)STEERING_SERVO_CENTER_US - offset);
    }

    magnitude = (uint32)(-(sint32)steeringDegX10);
    offset = ((magnitude * (uint32)(STEERING_SERVO_RIGHT_US - STEERING_SERVO_CENTER_US)) +
              ((uint32)ACKERMANN_CAN_PC_COMMAND_MAX_STEER_X10 / 2u)) /
             (uint32)ACKERMANN_CAN_PC_COMMAND_MAX_STEER_X10;
    return (uint16)((uint32)STEERING_SERVO_CENTER_US + offset);
}

static boolean CanSpeedTx_IsPcCommandPayloadValid(sint16 speedKphX100,
                                                   sint16 steeringDegX10,
                                                   uint16 steeringPulseUs,
                                                   uint8 flags)
{
    uint16 expectedPulseUs;
    uint16 pulseErrorUs;
    uint8 enabled;

    if ((flags & 0xFEu) != ACKERMANN_CAN_PC_COMMAND_SIGNATURE)
    {
        return FALSE;
    }

    if ((speedKphX100 < ACKERMANN_CAN_PC_COMMAND_MIN_SPEED_X100) ||
        (speedKphX100 > ACKERMANN_CAN_PC_COMMAND_MAX_SPEED_X100))
    {
        return FALSE;
    }

    if ((steeringDegX10 < -ACKERMANN_CAN_PC_COMMAND_MAX_STEER_X10) ||
        (steeringDegX10 > ACKERMANN_CAN_PC_COMMAND_MAX_STEER_X10))
    {
        return FALSE;
    }

    expectedPulseUs = CanSpeedTx_PcCommandExpectedPulseUs(steeringDegX10);
    pulseErrorUs = (steeringPulseUs >= expectedPulseUs) ?
                   (uint16)(steeringPulseUs - expectedPulseUs) :
                   (uint16)(expectedPulseUs - steeringPulseUs);
    if (pulseErrorUs > 3u)
    {
        return FALSE;
    }

    enabled = flags & ACKERMANN_CAN_PC_COMMAND_ENABLE_MASK;
    if ((enabled == 0u) &&
        ((speedKphX100 != 0) ||
         (steeringDegX10 != 0) ||
         (steeringPulseUs != STEERING_SERVO_CENTER_US)))
    {
        return FALSE;
    }

    return TRUE;
}

static void CanSpeedTx_UpdatePcCommandWatchdog(uint32 nowTick100us)
{
    uint32 ageTick100us;

    if (g_pcCommandModeSeen == 0u)
    {
        g_pcCommandAgeMs = 0xFFFFFFFFu;
        return;
    }

    ageTick100us = nowTick100us - g_pcCommandLastValidTick100us;
    g_pcCommandAgeMs = ageTick100us / 10u;
    if (g_pcCommandAgeMs > ACKERMANN_CAN_PC_COMMAND_TIMEOUT_MS)
    {
        if ((g_pcCommandValid != 0u) && (g_pcCommandWatchdogLatched == 0u))
        {
            g_pcCommandWatchdogCount++;
        }

        g_pcCommandValid = 0u;
        g_pcCommandEnabled = 0u;
        g_pcCommandWatchdogLatched = 1u;
    }
}

static boolean CanSpeedTx_SendRaw(IfxMultican_Can_MsgObj *txObj,
                                  uint32 id,
                                  const uint8 data[8])
{
    IfxMultican_Message msg;
    IfxMultican_Status status;
    uint32 dataLow;
    uint32 dataHigh;

    if ((txObj == 0) || (data == 0))
    {
        return FALSE;
    }

    dataLow = CanSpeedTx_PackU32Le(&data[0]);
    dataHigh = CanSpeedTx_PackU32Le(&data[4]);
    g_canTxAttemptCount++;
    g_canLastTxId = id;
    g_canLastDataLow = dataLow;
    g_canLastDataHigh = dataHigh;

    IfxMultican_Message_init(&msg,
                             id,
                             dataLow,
                             dataHigh,
                             IfxMultican_DataLengthCode_8);
    status = IfxMultican_Can_MsgObj_sendMessage(txObj, &msg);
    g_canLastTxStatus = (uint32)status;

    if (status == IfxMultican_Status_ok)
    {
        g_canTxOkCount++;
        return TRUE;
    }

    if (status == IfxMultican_Status_notSentBusy)
    {
        g_canTxBusyCount++;
    }
    else
    {
        g_canTxErrorCount++;
    }

    return FALSE;
}

static boolean CanSpeedTx_ServiceBusOffRecovery(void)
{
    IfxMultican_Status status;

    if ((g_canNode.node == 0) ||
        ((g_canNode.node->SR.B.BOFF == 0u) &&
         (g_canBusOffRecoveryActive == 0u)))
    {
        return TRUE;
    }

    if (g_canBusOffRecoveryActive == 0u)
    {
        g_canBusOffRecoveryActive = 1u;
        g_canBusOffDetectCount++;
    }

    status = IfxMultican_Can_Node_recoverBusOff(&g_canNode);
    g_canBusOffRecoveryAttemptCount++;
    g_canBusOffLastStatus = (uint32)status;

    if (status == IfxMultican_Status_ok)
    {
        g_canBusOffRecoveryActive = 0u;
        g_canBusOffRecoveryOkCount++;
        /* Send fresh diagnostics immediately after the node is active. */
        g_sensorFeedbackScheduleValid = 0u;
        g_vehicleManagerTelemetryPendingMask = 0u;
        g_vehicleManagerTelemetryScheduleValid = 0u;
        g_graphTestStatusPending = 0u;
        g_graphTestStatusPendingIsEvent = 0u;
        g_graphTestStatusScheduleValid = 0u;
        return TRUE;
    }

    return FALSE;
}

static void CanSpeedTx_InitTxObject(IfxMultican_Can_MsgObj *obj,
                                    uint32 objectId,
                                    uint32 messageId)
{
    IfxMultican_Can_MsgObjConfig msgConfig;
    IfxMultican_Status status;

    IfxMultican_Can_MsgObj_initConfig(&msgConfig, &g_canNode);
    msgConfig.msgObjId = objectId;
    msgConfig.messageId = messageId;
    msgConfig.acceptanceMask = 0x7FFFFFFFul;
    msgConfig.frame = IfxMultican_Frame_transmit;
    msgConfig.control.messageLen = IfxMultican_DataLengthCode_8;
    msgConfig.control.extendedFrame = FALSE;
    msgConfig.control.matchingId = TRUE;
    status = IfxMultican_Can_MsgObj_init(obj, &msgConfig);
    g_canMsgObjInitStatus = (uint32)status;
}

static void CanSpeedTx_InitPcCommandRxObject(void)
{
    IfxMultican_Can_MsgObjConfig msgConfig;
    IfxMultican_Status status;

    IfxMultican_Can_MsgObj_initConfig(&msgConfig, &g_canNode);
    msgConfig.msgObjId = ACKERMANN_CAN_PC_COMMAND_OBJ_ID;
    msgConfig.messageId = ACKERMANN_CAN_PC_COMMAND_ID;
    msgConfig.acceptanceMask = 0x7FFFFFFFul;
    msgConfig.frame = IfxMultican_Frame_receive;
    msgConfig.control.messageLen = IfxMultican_DataLengthCode_8;
    msgConfig.control.extendedFrame = FALSE;
    msgConfig.control.matchingId = TRUE;
    status = IfxMultican_Can_MsgObj_init(&g_rxObjPcCommand, &msgConfig);
    g_canMsgObjInitStatus = (uint32)status;
}

static void CanSpeedTx_InitAdasRxObject(IfxMultican_Can_MsgObj *obj,
                                        uint32 objectId,
                                        uint32 messageId,
                                        uint8 slotMask)
{
    IfxMultican_Can_MsgObjConfig msgConfig;
    IfxMultican_Status status;

    IfxMultican_Can_MsgObj_initConfig(&msgConfig, &g_canNode);
    msgConfig.msgObjId = objectId;
    msgConfig.messageId = messageId;
    msgConfig.acceptanceMask = 0x7FFFFFFFul;
    msgConfig.frame = IfxMultican_Frame_receive;
    msgConfig.control.messageLen = IfxMultican_DataLengthCode_8;
    msgConfig.control.extendedFrame = FALSE;
    msgConfig.control.matchingId = TRUE;
    status = IfxMultican_Can_MsgObj_init(obj, &msgConfig);
    g_canMsgObjInitStatus = (uint32)status;

    if (status == IfxMultican_Status_ok)
    {
        g_adasCanRxInitOkMask |= slotMask;
    }
    else
    {
        g_adasCanRxInitErrorMask |= slotMask;
    }
}

static void CanSpeedTx_PollAdasRxObject(IfxMultican_Can_MsgObj *obj,
                                        uint32 expectedId)
{
    IfxMultican_Message msg;
    IfxMultican_Status status;
    uint8 data[ADAS_MANAGER_CAN_DLC];
    uint8 index;
    boolean messageLost;

    IfxMultican_Message_init(&msg,
                             0u,
                             0u,
                             0u,
                             IfxMultican_DataLengthCode_8);
    status = IfxMultican_Can_MsgObj_readMessage(obj, &msg);

    if ((status & IfxMultican_Status_newData) != 0u)
    {
        for (index = 0u; index < 4u; index++)
        {
            data[index] = (uint8)((msg.data[0] >> (index * 8u)) & 0xFFu);
            data[index + 4u] = (uint8)((msg.data[1] >> (index * 8u)) & 0xFFu);
        }

        g_canRxCount++;
        g_canRxLastStatus = (uint32)status;
        g_canRxLastId = msg.id;
        g_canRxLastDataLow = msg.data[0];
        g_canRxLastDataHigh = msg.data[1];
        g_canRxLastTick100us = tick100us;

        messageLost = ((status & IfxMultican_Status_messageLost) != 0u) ?
                      TRUE : FALSE;
        if (messageLost != FALSE)
        {
            g_canRxLostCount++;
        }

        if (msg.id == expectedId)
        {
            (void)AdasManager_OnCanFrame((uint16)msg.id,
                                         data,
                                         (uint8)msg.lengthCode,
                                         messageLost,
                                         tick100us);
        }
        else
        {
            g_canRxLostCount++;
        }
    }
    else if ((status & IfxMultican_Status_messageLost) != 0u)
    {
        g_canRxLostCount++;
    }
}

void CanSpeedTx_Init(void)
{
    IfxMultican_Can_Config canConfig;
    IfxMultican_Can_NodeConfig nodeConfig;
    IfxMultican_Status status;

    g_canInitDone = 0u;
    g_canNodeInitStatus = 0xFFFFFFFFu;
    g_canMsgObjInitStatus = 0xFFFFFFFFu;
    g_canRxAlive = 0u;
    g_canRxLastTick100us = 0u;
    g_canRxLastAgeMs = 0xFFFFFFFFu;
    g_canBusOffRecoveryActive = 0u;
    g_canBusOffDetectCount = 0u;
    g_canBusOffRecoveryAttemptCount = 0u;
    g_canBusOffRecoveryOkCount = 0u;
    g_canBusOffLastStatus = 0xFFFFFFFFu;
    g_sensorFeedbackTxOkCount = 0u;
    g_sensorFeedbackTxRetryCount = 0u;
    g_vehicleManagerTelemetryTxOkCount = 0u;
    g_vehicleManagerTelemetryTxRetryCount = 0u;
    g_vehicleManagerTelemetrySequence = 0u;
    g_graphTestStatusTxOkCount = 0u;
    g_graphTestStatusTxRetryCount = 0u;
    g_graphTestStatusSequence = 0u;
    g_adasCanRxInitOkMask = 0u;
    g_adasCanRxInitErrorMask = 0u;
    g_pcCommandModeSeen = 0u;
    g_pcCommandValid = 0u;
    g_pcCommandEnabled = 0u;
    g_pcCommandSpeedKphX100 = 0;
    g_pcCommandSteeringDegX10 = 0;
    g_pcCommandSteeringPulseUs = STEERING_SERVO_CENTER_US;
    g_pcCommandSequence = 0u;
    g_pcCommandLastValidTick100us = 0u;
    g_pcCommandAgeMs = 0xFFFFFFFFu;
    g_pcCommandHasSequence = 0u;
    g_pcCommandWatchdogLatched = 0u;
    g_sensorFeedbackSequence = 0u;
    g_sensorFeedbackScheduleValid = 0u;
    g_sensorFeedbackLastTick100us = 0u;
    g_vehicleManagerTelemetryScheduleValid = 0u;
    g_vehicleManagerTelemetryPendingMask = 0u;
    g_vehicleManagerTelemetryLastTick100us = 0u;
    g_graphTestStatusScheduleValid = 0u;
    g_graphTestStatusPending = 0u;
    g_graphTestStatusPendingIsEvent = 0u;
    g_graphTestStatusLastTick100us = 0u;
    CanSpeedTx_EnableTransceiver();

    IfxMultican_Can_initModuleConfig(&canConfig, &MODULE_CAN);
    IfxMultican_Can_initModule(&g_canModule, &canConfig);

    IfxMultican_Can_Node_initConfig(&nodeConfig, &g_canModule);
    nodeConfig.nodeId = ACKERMANN_CAN_NODE_ID;
    nodeConfig.baudrate = ACKERMANN_CAN_BAUDRATE;
    nodeConfig.rxPin = ACKERMANN_CAN_RX_PIN;
    nodeConfig.rxPinMode = IfxPort_InputMode_pullUp;
    nodeConfig.txPin = ACKERMANN_CAN_TX_PIN;
    nodeConfig.txPinMode = IfxPort_OutputMode_pushPull;

    /* CAN FD 확장 (TRAJ_FD, 0x320) - 기존 0x300/0x130 등 classic 오브젝트는
     * messageLen=DataLengthCode_8, fastBitRate=FALSE로 그대로라 프레임
     * 포맷/타이밍 불변. 이 버스에 TC275-PC(Kvaser) 둘뿐이라 다른 classic
     * 전용 노드를 깨뜨릴 위험 없음 (TRAJ_FD_PROTOCOL.md 확인 사항 참고). */
    nodeConfig.flexibleDataRate = TRUE;
    nodeConfig.fdConfig.nominalBaudrate = ACKERMANN_CAN_BAUDRATE;
    nodeConfig.fdConfig.nominalSynchJumpWidth = 8000u;
    nodeConfig.fdConfig.nominalSamplePoint = 8000u;
    nodeConfig.fdConfig.fastBaudrate = 2000000u;
    nodeConfig.fdConfig.fastSynchJumpWidth = 8000u;
    nodeConfig.fdConfig.fastSamplePoint = 8000u;
    nodeConfig.fdConfig.loopDelayOffset = 0u;

    status = IfxMultican_Can_Node_init(&g_canNode, &nodeConfig);
    g_canNodeInitStatus = (uint32)status;

    CanSpeedTx_InitTxObject(&g_txObjSpeed,
                            ACKERMANN_CAN_SPEED_OBJ_ID,
                            ACKERMANN_CAN_WHEEL_SPEED_ID);
    CanSpeedTx_InitTxObject(&g_txObjSteering,
                            ACKERMANN_CAN_STEER_OBJ_ID,
                            ACKERMANN_CAN_STEERING_ID);
    CanSpeedTx_InitTxObject(&g_txObjDriveStatus,
                            ACKERMANN_CAN_DRIVE_OBJ_ID,
                            ACKERMANN_CAN_DRIVE_STATUS_ID);
    CanSpeedTx_InitTxObject(&g_txObjEncoderCount,
                            ACKERMANN_CAN_ENCODER_OBJ_ID,
                            ACKERMANN_CAN_ENCODER_COUNT_ID);
    CanSpeedTx_InitTxObject(&g_txObjEncoderDebug,
                            ACKERMANN_CAN_ENCDBG_OBJ_ID,
                            ACKERMANN_CAN_ENCODER_DEBUG_ID);
    CanSpeedTx_InitTxObject(&g_txObjSpeedControl,
                            ACKERMANN_CAN_SPEED_CTRL_OBJ_ID,
                            ACKERMANN_CAN_SPEED_CONTROL_ID);
    CanSpeedTx_InitTxObject(&g_txObjRpmDebug,
                            ACKERMANN_CAN_RPM_DEBUG_OBJ_ID,
                            ACKERMANN_CAN_RPM_DEBUG_ID);
    CanSpeedTx_InitTxObject(&g_txObjSteeringFeedback,
                            ACKERMANN_CAN_STEER_FB_OBJ_ID,
                            ACKERMANN_CAN_STEER_FB_ID);
    CanSpeedTx_InitTxObject(&g_txObjBufferTest,
                            ACKERMANN_CAN_BUFFER_TEST_OBJ_ID,
                            ACKERMANN_CAN_BUFFER_TEST_ID);
    CanSpeedTx_InitTxObject(&g_txObjDeadbandTest,
                            ACKERMANN_CAN_DEADBAND_TEST_OBJ_ID,
                            ACKERMANN_CAN_DEADBAND_TEST_ID);
    CanSpeedTx_InitTxObject(&g_txObjDeadbandResult,
                            ACKERMANN_CAN_DEADBAND_RESULT_OBJ_ID,
                            ACKERMANN_CAN_DEADBAND_RESULT_ID);
    CanSpeedTx_InitTxObject(&g_txObjMaxSpeedTest,
                            ACKERMANN_CAN_MAX_SPEED_TEST_OBJ_ID,
                            ACKERMANN_CAN_MAX_SPEED_TEST_ID);
    CanSpeedTx_InitTxObject(&g_txObjMaxSpeedResult,
                            ACKERMANN_CAN_MAX_SPEED_RESULT_OBJ_ID,
                            ACKERMANN_CAN_MAX_SPEED_RESULT_ID);
    CanSpeedTx_InitTxObject(&g_txObjJoystickCommand,
                            ACKERMANN_CAN_JOYSTICK_CMD_OBJ_ID,
                            ACKERMANN_CAN_JOYSTICK_CMD_ID);
    CanSpeedTx_InitTxObject(&g_txObjSensorFeedback,
                            ACKERMANN_CAN_SENSOR_FEEDBACK_OBJ_ID,
                            ACKERMANN_CAN_SENSOR_FEEDBACK_ID);
    CanSpeedTx_InitTxObject(&g_txObjVehicleManagerStatus,
                            ACKERMANN_CAN_VM_STATUS_OBJ_ID,
                            ACKERMANN_CAN_VM_STATUS_ID);
    CanSpeedTx_InitTxObject(&g_txObjVehicleManagerTargets,
                            ACKERMANN_CAN_VM_TARGETS_OBJ_ID,
                            ACKERMANN_CAN_VM_TARGETS_ID);
    CanSpeedTx_InitTxObject(&g_txObjVehicleManagerEnvelope,
                            ACKERMANN_CAN_VM_ENVELOPE_OBJ_ID,
                            ACKERMANN_CAN_VM_ENVELOPE_ID);
    CanSpeedTx_InitTxObject(&g_txObjGraphTestStatus,
                            ACKERMANN_CAN_GRAPH_TEST_STATUS_OBJ_ID,
                            ACKERMANN_CAN_GRAPH_TEST_STATUS_ID);
    CanSpeedTx_InitPcCommandRxObject();
    CanSpeedTx_InitAdasRxObject(&g_rxObjAdasMrm,
                                ACKERMANN_CAN_ADAS_MRM_OBJ_ID,
                                ADAS_MANAGER_CAN_ID_MRM,
                                0x01u);
    CanSpeedTx_InitAdasRxObject(&g_rxObjAdasCollision,
                                ACKERMANN_CAN_ADAS_COLLISION_OBJ_ID,
                                ADAS_MANAGER_CAN_ID_COLLISION,
                                0x02u);
    CanSpeedTx_InitAdasRxObject(&g_rxObjAdasParking,
                                ACKERMANN_CAN_ADAS_PARKING_OBJ_ID,
                                ADAS_MANAGER_CAN_ID_PARKING,
                                0x04u);
    CanSpeedTx_InitAdasRxObject(&g_rxObjAdasTraffic,
                                ACKERMANN_CAN_ADAS_TRAFFIC_OBJ_ID,
                                ADAS_MANAGER_CAN_ID_TRAFFIC,
                                0x08u);
    /* Nominal(0x314) classic-CAN RX 오브젝트는 v7에서 완전히 제거 - 카메라/GPS가
     * 각자 own 슬롯(0x320/0x321)을 TRAJ_FD로 직접 채우면서 Adas_Manager 쪽
     * Nominal 슬롯 자체가 없어졌다 (Adas_Manager.h/.c 참고). */

    /* TRAJ_FD 카메라(0x320)/GPS(0x321) 수신 오브젝트 둘 다 초기화.
     * g_canNode가 위에서 이미 flexibleDataRate=TRUE로 초기화된 뒤에
     * 호출해야 한다. */
    CanTrajectoryFd_InitAll(&g_canNode);

    g_canInitDone = 1u;
}

void CanSpeedTx_PollReceive(void)
{
    IfxMultican_Message msg;
    IfxMultican_Status status;
    uint32 nowTick100us;
    uint32 ageTick100us;
    sint16 speedKphX100;
    sint16 steeringDegX10;
    uint16 steeringPulseUs;
    uint8 sequence;
    uint8 flags;
    boolean payloadValid;

    if (g_canInitDone == 0u)
    {
        g_canRxAlive = 0u;
        g_canRxLastAgeMs = 0xFFFFFFFFu;
        g_pcCommandValid = 0u;
        g_pcCommandEnabled = 0u;
        g_pcCommandAgeMs = 0xFFFFFFFFu;
        return;
    }

    if (CanSpeedTx_ServiceBusOffRecovery() == FALSE)
    {
        nowTick100us = tick100us;
        g_canRxAlive = 0u;
        g_canRxLastAgeMs = 0xFFFFFFFFu;
        CanSpeedTx_UpdatePcCommandWatchdog(nowTick100us);
        return;
    }

    /* Independent mailboxes prevent one application's burst from overwriting
     * another before this 5 ms polling loop runs. */
    if ((g_adasCanRxInitOkMask & 0x01u) != 0u)
    {
        CanSpeedTx_PollAdasRxObject(&g_rxObjAdasMrm, ADAS_MANAGER_CAN_ID_MRM);
    }
    if ((g_adasCanRxInitOkMask & 0x02u) != 0u)
    {
        CanSpeedTx_PollAdasRxObject(&g_rxObjAdasCollision, ADAS_MANAGER_CAN_ID_COLLISION);
    }
    if ((g_adasCanRxInitOkMask & 0x04u) != 0u)
    {
        CanSpeedTx_PollAdasRxObject(&g_rxObjAdasParking, ADAS_MANAGER_CAN_ID_PARKING);
    }
    if ((g_adasCanRxInitOkMask & 0x08u) != 0u)
    {
        CanSpeedTx_PollAdasRxObject(&g_rxObjAdasTraffic, ADAS_MANAGER_CAN_ID_TRAFFIC);
    }
    /* Camera(0x320)/Gps(0x321) 슬롯은 classic CAN으로 안 받음 - TRAJ_FD 32바이트
     * 프레임을 CanTrajectoryFd_PollChannel()로 받아서, ControlLoop.c에서
     * AdasManager_OnCanFrame()으로 직접 주입한다 (v7, Adas_Manager 자체 6슬롯
     * 우선순위: Mrm>Collision>Parking>Traffic>Camera>Gps). */

    IfxMultican_Message_init(&msg,
                             0u,
                             0u,
                             0u,
                             IfxMultican_DataLengthCode_8);
    status = IfxMultican_Can_MsgObj_readMessage(&g_rxObjPcCommand, &msg);
    g_canRxLastStatus = (uint32)status;

    if ((status & IfxMultican_Status_newData) != 0u)
    {
        g_canRxCount++;
        g_canRxLastId = msg.id;
        g_canRxLastDataLow = msg.data[0];
        g_canRxLastDataHigh = msg.data[1];
        g_canRxLastTick100us = tick100us;

        speedKphX100 = (sint16)(msg.data[0] & 0xFFFFu);
        steeringDegX10 = (sint16)((msg.data[0] >> 16u) & 0xFFFFu);
        steeringPulseUs = (uint16)(msg.data[1] & 0xFFFFu);
        sequence = (uint8)((msg.data[1] >> 16u) & 0xFFu);
        flags = (uint8)((msg.data[1] >> 24u) & 0xFFu);

        payloadValid = ((status & IfxMultican_Status_messageLost) == 0u) &&
                       (msg.id == ACKERMANN_CAN_PC_COMMAND_ID) &&
                       (msg.lengthCode == IfxMultican_DataLengthCode_8) &&
                       (CanSpeedTx_IsPcCommandPayloadValid(speedKphX100,
                                                           steeringDegX10,
                                                           steeringPulseUs,
                                                           flags) != FALSE);

        if (payloadValid != FALSE)
        {
            if ((g_pcCommandHasSequence != 0u) &&
                (sequence == g_pcCommandSequence))
            {
                /* A repeated enabled command must not extend the watchdog. */
                g_pcCommandDuplicateCount++;
            }
            else
            {
                g_pcCommandModeSeen = 1u;
                g_pcCommandValid = 1u;
                g_pcCommandEnabled = flags & ACKERMANN_CAN_PC_COMMAND_ENABLE_MASK;
                g_pcCommandSpeedKphX100 = speedKphX100;
                g_pcCommandSteeringDegX10 = steeringDegX10;
                g_pcCommandSteeringPulseUs = steeringPulseUs;
                g_pcCommandSequence = sequence;
                g_pcCommandLastValidTick100us = tick100us;
                g_pcCommandAgeMs = 0u;
                g_pcCommandValidCount++;
                g_pcCommandHasSequence = 1u;
                g_pcCommandWatchdogLatched = 0u;
            }
        }
        else
        {
            g_pcCommandInvalidCount++;
            if (g_pcCommandModeSeen != 0u)
            {
                /* A malformed frame on the command ID is an immediate STOP. */
                g_pcCommandValid = 0u;
                g_pcCommandEnabled = 0u;
            }
        }
    }
    else if ((status & IfxMultican_Status_receiveEmpty) != 0u)
    {
        g_canRxEmptyCount++;
    }

    if ((status & IfxMultican_Status_messageLost) != 0u)
    {
        g_canRxLostCount++;
    }

    nowTick100us = tick100us;
    if (g_canRxLastTick100us == 0u)
    {
        g_canRxAlive = 0u;
        g_canRxLastAgeMs = 0xFFFFFFFFu;
        CanSpeedTx_UpdatePcCommandWatchdog(nowTick100us);
        return;
    }

    ageTick100us = nowTick100us - g_canRxLastTick100us;
    g_canRxLastAgeMs = ageTick100us / 10u;
    g_canRxAlive = (g_canRxLastAgeMs <= ACKERMANN_CAN_RX_ALIVE_TIMEOUT_MS) ? 1u : 0u;
    CanSpeedTx_UpdatePcCommandWatchdog(nowTick100us);
}

void CanSpeedTx_InvalidatePcCommandForManualOverride(void)
{
    if ((g_pcCommandValid != 0u) || (g_pcCommandEnabled != 0u))
    {
        g_pcCommandManualBlockCount++;
    }

    /* Keep the sequence history so a replayed packet cannot resume control. */
    g_pcCommandValid = 0u;
    g_pcCommandEnabled = 0u;
}

void CanSpeedTx_SendWheelSpeed(const VehicleSpeedValue *speed)
{
    uint8 data[8] = {0u};
    sint16 leftRpmX10;
    sint16 rightRpmX10;
    sint16 vehicleMpsX1000;

    if (speed == 0)
    {
        return;
    }

    leftRpmX10 = CanSpeedTx_ClampFloatToS16(speed->left.avgRpm * 10.0f);
    rightRpmX10 = CanSpeedTx_ClampFloatToS16(speed->right.avgRpm * 10.0f);
    vehicleMpsX1000 = CanSpeedTx_ClampFloatToS16(speed->vehicleMps * 1000.0f);

    CanSpeedTx_PutS16(data, 0u, leftRpmX10);
    CanSpeedTx_PutS16(data, 2u, rightRpmX10);
    CanSpeedTx_PutS16(data, 4u, vehicleMpsX1000);
    data[6] = (speed->valid == TRUE) ? 1u : 0u;
    data[7] = 0u;

    CanSpeedTx_SendRaw(&g_txObjSpeed, ACKERMANN_CAN_WHEEL_SPEED_ID, data);
}

void CanSpeedTx_SendRpmDebug(const VehicleSpeedValue *speed)
{
    uint8 data[8] = {0u};

    if (speed == 0)
    {
        return;
    }

    CanSpeedTx_PutS16(data, 0u, CanSpeedTx_ClampFloatToS16(speed->left.rawRpm * 10.0f));
    CanSpeedTx_PutS16(data, 2u, CanSpeedTx_ClampFloatToS16(speed->left.avgRpm * 10.0f));
    CanSpeedTx_PutS16(data, 4u, CanSpeedTx_ClampFloatToS16(speed->right.rawRpm * 10.0f));
    CanSpeedTx_PutS16(data, 6u, CanSpeedTx_ClampFloatToS16(speed->right.avgRpm * 10.0f));

    CanSpeedTx_SendRaw(&g_txObjRpmDebug, ACKERMANN_CAN_RPM_DEBUG_ID, data);
}

void CanSpeedTx_SendDriveStatus(uint8 dutyPercent,
                                uint8 driveDirection,
                                uint8 estopActive,
                                uint8 speedBufferCount,
                                sint32 leftDeltaCount,
                                sint32 rightDeltaCount)
{
    uint8 data[8] = {0u};

    data[0] = dutyPercent;
    data[1] = driveDirection;
    data[2] = estopActive;
    data[3] = speedBufferCount;
    CanSpeedTx_PutS16(data, 4u, CanSpeedTx_ClampS32ToS16(leftDeltaCount));
    CanSpeedTx_PutS16(data, 6u, CanSpeedTx_ClampS32ToS16(rightDeltaCount));

    CanSpeedTx_SendRaw(&g_txObjDriveStatus, ACKERMANN_CAN_DRIVE_STATUS_ID, data);
}

void CanSpeedTx_SendEncoderCount(sint32 leftCount, sint32 rightCount)
{
    uint8 data[8] = {0u};

    CanSpeedTx_PutS32(data, 0u, leftCount);
    CanSpeedTx_PutS32(data, 4u, rightCount);

    CanSpeedTx_SendRaw(&g_txObjEncoderCount, ACKERMANN_CAN_ENCODER_COUNT_ID, data);
}

void CanSpeedTx_SendEncoderDebug(uint16 leftHwCount,
                                 uint16 rightHwCount,
                                 sint32 leftHwDelta,
                                 sint32 rightHwDelta)
{
    uint8 data[8] = {0u};

    CanSpeedTx_PutU16(data, 0u, leftHwCount);
    CanSpeedTx_PutU16(data, 2u, rightHwCount);
    CanSpeedTx_PutS16(data, 4u, CanSpeedTx_ClampS32ToS16(leftHwDelta));
    CanSpeedTx_PutS16(data, 6u, CanSpeedTx_ClampS32ToS16(rightHwDelta));

    CanSpeedTx_SendRaw(&g_txObjEncoderDebug, ACKERMANN_CAN_ENCODER_DEBUG_ID, data);
}

void CanSpeedTx_SendSpeedControlStatus(float32 targetRpm,
                                       float32 currentRpm,
                                       float32 dutyCommand,
                                       float32 feedforwardDuty)
{
    uint8 data[8] = {0u};

    CanSpeedTx_PutS16(data, 0u, CanSpeedTx_ClampFloatToS16(targetRpm * 10.0f));
    CanSpeedTx_PutS16(data, 2u, CanSpeedTx_ClampFloatToS16(currentRpm * 10.0f));
    CanSpeedTx_PutU16(data, 4u, CanSpeedTx_ClampFloatToU16(dutyCommand * 10.0f));
    CanSpeedTx_PutU16(data, 6u, CanSpeedTx_ClampFloatToU16(feedforwardDuty * 10.0f));

    CanSpeedTx_SendRaw(&g_txObjSpeedControl, ACKERMANN_CAN_SPEED_CONTROL_ID, data);
}

void CanSpeedTx_SendSteeringStatus(float32 targetDeg,
                                   float32 measuredDeg,
                                   uint16 adcRaw,
                                   uint16 pulseUs,
                                   uint8 stateCode)
{
    uint8 data[8] = {0u};
    sint16 targetDegX10;
    sint16 measuredDegX10;

    targetDegX10 = CanSpeedTx_ClampFloatToS16(targetDeg * 10.0f);
    measuredDegX10 = CanSpeedTx_ClampFloatToS16(measuredDeg * 10.0f);

    CanSpeedTx_PutS16(data, 0u, targetDegX10);
    CanSpeedTx_PutS16(data, 2u, measuredDegX10);
    CanSpeedTx_PutU16(data, 4u, adcRaw);
    data[6] = CanSpeedTx_EncodePulseOffset5us(pulseUs);
    data[7] = stateCode;

    CanSpeedTx_SendRaw(&g_txObjSteering, ACKERMANN_CAN_STEERING_ID, data);
}

void CanSpeedTx_SendSteeringFeedback(float32 currentDeg,
                                     uint16 adcRaw,
                                     float32 adcMv,
                                     float32 errorDeg)
{
    uint8 data[8] = {0u};

    CanSpeedTx_PutS16(data, 0u, CanSpeedTx_ClampFloatToS16(currentDeg * 10.0f));
    CanSpeedTx_PutU16(data, 2u, adcRaw);
    CanSpeedTx_PutU16(data, 4u, CanSpeedTx_ClampFloatToU16(adcMv));
    CanSpeedTx_PutS16(data, 6u, CanSpeedTx_ClampFloatToS16(errorDeg * 10.0f));

    CanSpeedTx_SendRaw(&g_txObjSteeringFeedback, ACKERMANN_CAN_STEER_FB_ID, data);
}

void CanSpeedTx_SendSensorFeedback(const VehicleSpeedValue *speed,
                                   float32 measuredSteeringDeg,
                                   uint16 steeringPotAdcRaw,
                                   uint8 steeringPotValid,
                                   uint8 pcControlActive,
                                   uint8 estopActive)
{
    uint8 data[8] = {0u};
    uint8 flags = ACKERMANN_CAN_SENSOR_FEEDBACK_SIGNATURE;
    uint32 nowTick100us = tick100us;
    float32 measuredSpeedKph = 0.0f;

    if ((g_sensorFeedbackScheduleValid != 0u) &&
        ((nowTick100us - g_sensorFeedbackLastTick100us) <
         ACKERMANN_CAN_SENSOR_FEEDBACK_PERIOD_TICK100US))
    {
        return;
    }

    if ((speed != 0) &&
        (speed->valid == TRUE) &&
        (speed->left.valid == TRUE) &&
        (speed->right.valid == TRUE))
    {
        measuredSpeedKph = speed->vehicleMpsSigned * 3.6f;
        flags |= ACKERMANN_CAN_SENSOR_FEEDBACK_SPEED_VALID_MASK;
    }

    if (steeringPotValid != 0u)
    {
        flags |= ACKERMANN_CAN_SENSOR_FEEDBACK_STEERING_VALID_MASK;
    }

    if (pcControlActive != 0u)
    {
        flags |= ACKERMANN_CAN_SENSOR_FEEDBACK_PC_ACTIVE_MASK;
    }

    if (estopActive != 0u)
    {
        flags |= ACKERMANN_CAN_SENSOR_FEEDBACK_ESTOP_MASK;
    }

    CanSpeedTx_PutS16(data, 0u,
                      CanSpeedTx_ClampFloatToS16(measuredSpeedKph * 100.0f));
    CanSpeedTx_PutS16(data, 2u,
                      CanSpeedTx_ClampFloatToS16(measuredSteeringDeg * 10.0f));
    CanSpeedTx_PutU16(data, 4u, steeringPotAdcRaw);
    data[6] = g_sensorFeedbackSequence;
    data[7] = flags;

    if (CanSpeedTx_SendRaw(&g_txObjSensorFeedback,
                           ACKERMANN_CAN_SENSOR_FEEDBACK_ID,
                           data) != FALSE)
    {
        g_sensorFeedbackScheduleValid = 1u;
        g_sensorFeedbackLastTick100us = nowTick100us;
        g_sensorFeedbackSequence++;
        g_sensorFeedbackTxOkCount++;
    }
    else
    {
        /* Do not consume the period or sequence; retry on the next loop. */
        g_sensorFeedbackTxRetryCount++;
    }
}

static void CanSpeedTx_CaptureVehicleManagerTelemetry(
    const VehicleManagerOutput *output)
{
    uint8 flags = 0u;

    if (output->driveEnable != FALSE)
    {
        flags |= ACKERMANN_CAN_VM_DRIVE_ENABLE_MASK;
    }
    if (output->steeringTargetValid != FALSE)
    {
        flags |= ACKERMANN_CAN_VM_STEERING_VALID_MASK;
    }
    if (output->lateralLimitActive != FALSE)
    {
        flags |= ACKERMANN_CAN_VM_LATERAL_LIMIT_MASK;
    }
    if (output->steeringLimitActive != FALSE)
    {
        flags |= ACKERMANN_CAN_VM_STEERING_LIMIT_MASK;
    }
    if (output->canRearmRequired != FALSE)
    {
        flags |= ACKERMANN_CAN_VM_CAN_REARM_MASK;
    }

    g_vehicleManagerStatusData[0] = (uint8)output->state;
    g_vehicleManagerStatusData[1] = (uint8)output->owner;
    CanSpeedTx_PutU32(g_vehicleManagerStatusData, 2u, output->faultMask);
    g_vehicleManagerStatusData[6] = flags;
    g_vehicleManagerStatusData[7] = g_vehicleManagerTelemetrySequence;

    CanSpeedTx_PutS16(g_vehicleManagerTargetsData, 0u,
                      CanSpeedTx_ClampFloatToS16(output->requestedSpeedKph * 100.0f));
    CanSpeedTx_PutS16(g_vehicleManagerTargetsData, 2u,
                      CanSpeedTx_ClampFloatToS16(output->targetSpeedKph * 100.0f));
    CanSpeedTx_PutS16(g_vehicleManagerTargetsData, 4u,
                      CanSpeedTx_ClampFloatToS16(output->requestedSteeringDeg * 10.0f));
    CanSpeedTx_PutS16(g_vehicleManagerTargetsData, 6u,
                      CanSpeedTx_ClampFloatToS16(output->targetSteeringDeg * 10.0f));

    CanSpeedTx_PutU16(g_vehicleManagerEnvelopeData, 0u,
                      CanSpeedTx_ClampFloatToU16(output->effectiveSteeringDeg * 10.0f));
    CanSpeedTx_PutU16(g_vehicleManagerEnvelopeData, 2u,
                      CanSpeedTx_ClampFloatToU16(output->steeringLimitDeg * 10.0f));
    CanSpeedTx_PutU16(g_vehicleManagerEnvelopeData, 4u,
                      CanSpeedTx_ClampFloatToU16(output->turnSpeedLimitKph * 100.0f));
    CanSpeedTx_PutU16(g_vehicleManagerEnvelopeData, 6u,
                      CanSpeedTx_ClampFloatToU16(
                          output->predictedLateralAccelMps2 * 1000.0f));

    g_vehicleManagerTelemetryPendingMask = ACKERMANN_CAN_VM_PENDING_ALL;
}

void CanSpeedTx_SendVehicleManagerTelemetry(const VehicleManagerOutput *output)
{
    uint32 nowTick100us = tick100us;

    if (output == 0)
    {
        return;
    }

    if (g_vehicleManagerTelemetryPendingMask == 0u)
    {
        if ((g_vehicleManagerTelemetryScheduleValid != 0u) &&
            ((nowTick100us - g_vehicleManagerTelemetryLastTick100us) <
             ACKERMANN_CAN_VM_TELEMETRY_PERIOD_TICK100US))
        {
            return;
        }

        /* Keep the three frames coherent even when a busy mailbox needs retry. */
        CanSpeedTx_CaptureVehicleManagerTelemetry(output);
    }

    if ((g_vehicleManagerTelemetryPendingMask &
         ACKERMANN_CAN_VM_PENDING_STATUS) != 0u)
    {
        if (CanSpeedTx_SendRaw(&g_txObjVehicleManagerStatus,
                               ACKERMANN_CAN_VM_STATUS_ID,
                               g_vehicleManagerStatusData) != FALSE)
        {
            g_vehicleManagerTelemetryPendingMask &=
                (uint8)(~ACKERMANN_CAN_VM_PENDING_STATUS);
        }
    }

    if ((g_vehicleManagerTelemetryPendingMask &
         ACKERMANN_CAN_VM_PENDING_TARGETS) != 0u)
    {
        if (CanSpeedTx_SendRaw(&g_txObjVehicleManagerTargets,
                               ACKERMANN_CAN_VM_TARGETS_ID,
                               g_vehicleManagerTargetsData) != FALSE)
        {
            g_vehicleManagerTelemetryPendingMask &=
                (uint8)(~ACKERMANN_CAN_VM_PENDING_TARGETS);
        }
    }

    if ((g_vehicleManagerTelemetryPendingMask &
         ACKERMANN_CAN_VM_PENDING_ENVELOPE) != 0u)
    {
        if (CanSpeedTx_SendRaw(&g_txObjVehicleManagerEnvelope,
                               ACKERMANN_CAN_VM_ENVELOPE_ID,
                               g_vehicleManagerEnvelopeData) != FALSE)
        {
            g_vehicleManagerTelemetryPendingMask &=
                (uint8)(~ACKERMANN_CAN_VM_PENDING_ENVELOPE);
        }
    }

    if (g_vehicleManagerTelemetryPendingMask == 0u)
    {
        /* Consume the common period and heartbeat only after all three send. */
        g_vehicleManagerTelemetryScheduleValid = 1u;
        g_vehicleManagerTelemetryLastTick100us = nowTick100us;
        g_vehicleManagerTelemetrySequence++;
        g_vehicleManagerTelemetryTxOkCount++;
    }
    else
    {
        g_vehicleManagerTelemetryTxRetryCount++;
    }
}

static void CanSpeedTx_CaptureGraphTestStatus(uint8 mode,
                                               uint8 state,
                                               uint8 phase,
                                               uint8 abortReason,
                                               uint16 phaseElapsedMs,
                                               uint8 flags)
{
    g_graphTestStatusData[0] = mode;
    g_graphTestStatusData[1] = state;
    g_graphTestStatusData[2] = phase;
    g_graphTestStatusData[3] = abortReason;
    CanSpeedTx_PutU16(g_graphTestStatusData, 4u, phaseElapsedMs);
    g_graphTestStatusData[6] = flags;
    g_graphTestStatusData[7] = g_graphTestStatusSequence;
    g_graphTestStatusPending = 1u;
    g_graphTestStatusPendingIsEvent =
        ((flags & ACKERMANN_CAN_GRAPH_TEST_PHASE_EVENT_MASK) != 0u) ? 1u : 0u;
}

void CanSpeedTx_SendGraphTestStatus(uint8 mode,
                                    uint8 state,
                                    uint8 phase,
                                    uint8 abortReason,
                                    uint16 phaseElapsedMs,
                                    uint8 flags)
{
    uint32 nowTick100us = tick100us;
    uint8 eventRequested =
        ((flags & ACKERMANN_CAN_GRAPH_TEST_PHASE_EVENT_MASK) != 0u) ? 1u : 0u;

    if ((eventRequested != 0u) &&
        (g_graphTestStatusPendingIsEvent == 0u))
    {
        /* A transition event may replace an unsent periodic snapshot. */
        CanSpeedTx_CaptureGraphTestStatus(mode,
                                          state,
                                          phase,
                                          abortReason,
                                          phaseElapsedMs,
                                          flags);
    }
    else if (g_graphTestStatusPending == 0u)
    {
        if ((eventRequested == 0u) &&
            (g_graphTestStatusScheduleValid != 0u) &&
            ((nowTick100us - g_graphTestStatusLastTick100us) <
             ACKERMANN_CAN_GRAPH_TEST_STATUS_PERIOD_TICK100US))
        {
            return;
        }

        CanSpeedTx_CaptureGraphTestStatus(mode,
                                          state,
                                          phase,
                                          abortReason,
                                          phaseElapsedMs,
                                          flags);
    }

    if (CanSpeedTx_SendRaw(&g_txObjGraphTestStatus,
                           ACKERMANN_CAN_GRAPH_TEST_STATUS_ID,
                           g_graphTestStatusData) != FALSE)
    {
        g_graphTestStatusPending = 0u;
        g_graphTestStatusPendingIsEvent = 0u;
        g_graphTestStatusScheduleValid = 1u;
        g_graphTestStatusLastTick100us = nowTick100us;
        g_graphTestStatusSequence++;
        g_graphTestStatusTxOkCount++;
    }
    else
    {
        /* Keep the packed snapshot and sequence for the next 5 ms retry. */
        g_graphTestStatusTxRetryCount++;
    }
}

void CanSpeedTx_SendBufferTestStatus(uint8 bufferSize,
                                     uint8 stageIndex,
                                     uint8 dutyPercent,
                                     uint8 statusCode,
                                     float32 avgVehicleRpm,
                                     uint16 elapsedMs)
{
    uint8 data[8] = {0u};

    data[0] = bufferSize;
    data[1] = stageIndex;
    data[2] = dutyPercent;
    data[3] = statusCode;
    CanSpeedTx_PutS16(data, 4u, CanSpeedTx_ClampFloatToS16(avgVehicleRpm * 10.0f));
    CanSpeedTx_PutU16(data, 6u, elapsedMs);

    CanSpeedTx_SendRaw(&g_txObjBufferTest, ACKERMANN_CAN_BUFFER_TEST_ID, data);
}

void CanSpeedTx_SendDriveDeadbandTestStatus(uint8 testCaseId,
                                            uint8 statusCode,
                                            uint8 currentDutyPercent,
                                            uint8 lastStoppedDutyPercent,
                                            uint8 firstMovingDutyPercent,
                                            float32 currentRpm,
                                            uint8 elapsedSec)
{
    uint8 data[8] = {0u};

    data[0] = testCaseId;
    data[1] = statusCode;
    data[2] = currentDutyPercent;
    data[3] = lastStoppedDutyPercent;
    data[4] = firstMovingDutyPercent;
    CanSpeedTx_PutS16(data, 5u, CanSpeedTx_ClampFloatToS16(currentRpm * 10.0f));
    data[7] = elapsedSec;

    CanSpeedTx_SendRaw(&g_txObjDeadbandTest, ACKERMANN_CAN_DEADBAND_TEST_ID, data);
}

void CanSpeedTx_SendDriveDeadbandResult(uint8 testCaseId,
                                        uint8 statusCode,
                                        uint8 lastStoppedDutyPercent,
                                        uint8 firstMovingDutyPercent,
                                        float32 firstMovingRpm,
                                        float32 firstMovingKph)
{
    uint8 data[8] = {0u};

    data[0] = testCaseId;
    data[1] = statusCode;
    data[2] = lastStoppedDutyPercent;
    data[3] = firstMovingDutyPercent;
    CanSpeedTx_PutS16(data, 4u, CanSpeedTx_ClampFloatToS16(firstMovingRpm * 10.0f));
    CanSpeedTx_PutU16(data, 6u, CanSpeedTx_ClampFloatToU16(firstMovingKph * 100.0f));

    CanSpeedTx_SendRaw(&g_txObjDeadbandResult, ACKERMANN_CAN_DEADBAND_RESULT_ID, data);
}

void CanSpeedTx_SendMaxSpeedTestStatus(uint8 testCaseId,
                                       uint8 statusCode,
                                       uint8 phase,
                                       uint8 dutyPercent,
                                       float32 currentRpm,
                                       float32 currentKph)
{
    uint8 data[8] = {0u};

    data[0] = testCaseId;
    data[1] = statusCode;
    data[2] = phase;
    data[3] = dutyPercent;
    CanSpeedTx_PutS16(data, 4u, CanSpeedTx_ClampFloatToS16(currentRpm * 10.0f));
    CanSpeedTx_PutU16(data, 6u, CanSpeedTx_ClampFloatToU16(currentKph * 100.0f));

    CanSpeedTx_SendRaw(&g_txObjMaxSpeedTest, ACKERMANN_CAN_MAX_SPEED_TEST_ID, data);
}

void CanSpeedTx_SendMaxSpeedResult(uint8 testCaseId,
                                   uint8 statusCode,
                                   uint8 dutyPercent,
                                   uint8 elapsedSec,
                                   float32 avgRpm,
                                   float32 avgKph)
{
    uint8 data[8] = {0u};

    data[0] = testCaseId;
    data[1] = statusCode;
    data[2] = dutyPercent;
    data[3] = elapsedSec;
    CanSpeedTx_PutS16(data, 4u, CanSpeedTx_ClampFloatToS16(avgRpm * 10.0f));
    CanSpeedTx_PutU16(data, 6u, CanSpeedTx_ClampFloatToU16(avgKph * 100.0f));

    CanSpeedTx_SendRaw(&g_txObjMaxSpeedResult, ACKERMANN_CAN_MAX_SPEED_RESULT_ID, data);
}

void CanSpeedTx_SendJoystickCommand(uint8 present,
                                    uint8 receiverAlive,
                                    uint16 buttons,
                                    float32 targetRpm,
                                    uint16 steeringPulseUs)
{
    uint8 data[8] = {0u};

    data[0] = present;
    data[1] = receiverAlive;
    CanSpeedTx_PutU16(data, 2u, buttons);
    CanSpeedTx_PutS16(data, 4u, CanSpeedTx_ClampFloatToS16(targetRpm * 10.0f));
    CanSpeedTx_PutU16(data, 6u, steeringPulseUs);

    CanSpeedTx_SendRaw(&g_txObjJoystickCommand, ACKERMANN_CAN_JOYSTICK_CMD_ID, data);
}
