#include "Ps2_Controller.h"

#include "Board_Pins.h"
#include "CCU6_PWM_Generation.h"
#include "Can_SpeedTx.h"
#include "ControlLoop.h"
#include "GPT12_Timer_Interrupt.h"
#include "IfxPort.h"
#include "Stm/Std/IfxStm.h"
#include "Steering_ADC.h"

#define PS2_POLL_PERIOD_MS          (20u)
#define PS2_BIT_DELAY_US            (8u)
#define PS2_AXIS_DEADZONE_RAW       (10u)
#define PS2_STEERING_AXIS_RX        (7u)
#define PS2_STEERING_INVERT_DEFAULT (0u)
#define PS2_STEERING_FIXED_MODE_DEFAULT (0u)
#define PS2_STEERING_LEFT_THRESHOLD_RAW (90u)
#define PS2_STEERING_RIGHT_THRESHOLD_RAW (166u)
#define PS2_STEERING_DECISION_LEFT  (0u)
#define PS2_STEERING_DECISION_CENTER (1u)
#define PS2_STEERING_DECISION_RIGHT (2u)
#define PS2_REVERSE_TARGET_KPH_X10  (-10)
#define PS2_SPEED_TEST_MIN_KPH       (1u)
#define PS2_SPEED_TEST_MAX_KPH       (13u)
#define PS2_SPEED_TEST_KPH_TO_X10    (10)
#define PS2_CONNECT_CONFIRM_CNT     (1u)
/*
 * The wireless pad is polled every 20 ms. A single corrupt packet must not
 * release manual steering authority to the PC path. Confirm loss for five
 * consecutive polls (about 100 ms); valid data immediately cancels the count.
 */
#define PS2_OFF_CONFIRM_CNT         (5u)
#define STATUS_LED_MODE_PS2         (0u)
#define STATUS_LED_MODE_CAN         (1u)
#define STATUS_LED_MODE_PS2_AND_CAN (2u)
#define STATUS_LED_MODE_PS2_OR_CAN  (3u)

volatile uint8   g_ps2AckermannPresent = 0u;
volatile uint8   g_ps2AckermannReceiverAlive = 0u;
volatile uint8   g_ps2AckermannMode = 0u;
volatile uint8   g_ps2AckermannStatus = 0u;
volatile uint16  g_ps2AckermannButtons = 0u;
volatile uint8   g_ps2AckermannRx = 0x80u;
volatile uint8   g_ps2AckermannRy = 0x80u;
volatile uint8   g_ps2AckermannLx = 0x80u;
volatile uint8   g_ps2AckermannLy = 0x80u;
volatile uint16  g_ps2AckermannSteeringPulseUs = STEERING_SERVO_NO_JOYSTICK_US;
volatile uint16  g_ps2AckermannSteeringTargetRaw = STEERING_ADC_RAW_CENTER;
volatile sint16  g_ps2AckermannTargetKphX10 = 0;
volatile uint8   g_ps2SpeedTestSelectedKph = 0u;
volatile uint32  g_ps2AckermannOkCount = 0u;
volatile uint32  g_ps2AckermannErrCount = 0u;
volatile uint32  g_ps2AckermannLastAgeMs = 0xFFFFFFFFu;
volatile uint32  g_ps2AckermannPollCount = 0u;
volatile uint32  g_ps2AckermannDataHighCount = 0u;
volatile uint32  g_ps2AckermannDataLowCount = 0u;
volatile uint8   g_ps2AckermannDataPin = 1u;
volatile uint8   g_ps2AckermannCmdPin = 1u;
volatile uint8   g_ps2AckermannAttPin = 1u;
volatile uint8   g_ps2AckermannClkPin = 1u;
volatile uint8   g_ps2AckermannLastRxByte = 0xFFu;
volatile uint8   g_ps2AckermannRaw0 = 0xFFu;
volatile uint8   g_ps2AckermannRaw1 = 0xFFu;
volatile uint8   g_ps2AckermannRaw2 = 0xFFu;
volatile uint8   g_ps2AckermannRaw3 = 0xFFu;
volatile uint8   g_ps2AckermannRaw4 = 0xFFu;
volatile uint8   g_ps2AckermannRaw5 = 0xFFu;
volatile uint8   g_ps2AckermannRaw6 = 0xFFu;
volatile uint8   g_ps2AckermannRaw7 = 0xFFu;
volatile uint8   g_ps2AckermannRaw8 = 0xFFu;
volatile uint8   g_ps2AckermannStableConnected = 0u;
volatile uint8   g_statusLedMode = STATUS_LED_MODE_PS2_OR_CAN;
volatile uint8   g_statusLedConnected = 0u;
volatile uint8   g_ps2SteeringFixedPulseMode = PS2_STEERING_FIXED_MODE_DEFAULT;
volatile uint8   g_ps2SteeringLeftThresholdRaw = PS2_STEERING_LEFT_THRESHOLD_RAW;
volatile uint8   g_ps2SteeringRightThresholdRaw = PS2_STEERING_RIGHT_THRESHOLD_RAW;
volatile uint8   g_ps2SteeringInvert = PS2_STEERING_INVERT_DEFAULT;
volatile uint16  g_ps2SteeringFixedLeftPulseUs = STEERING_SERVO_LEFT_US;
volatile uint16  g_ps2SteeringFixedCenterPulseUs = STEERING_SERVO_CENTER_US;
volatile uint16  g_ps2SteeringFixedRightPulseUs = STEERING_SERVO_RIGHT_US;
volatile uint8   g_ps2SteeringDecision = PS2_STEERING_DECISION_CENTER;

static uint16 g_ps2PreviousPressedButtons = 0u;
static boolean g_ps2SpeedTestUpArmed = FALSE;

static uint8 g_raw[PS2_PACKET_SIZE] = {0u};
static uint32 g_lastPollTick100us = 0u;
static uint32 g_lastGoodTick100us = 0u;
static uint32 g_lastPadGoodTick100us = 0u;
static boolean g_present = FALSE;
static boolean g_receiverAlive = FALSE;
static boolean g_seenValid = FALSE;
static boolean g_seenPadConnected = FALSE;
static uint8 g_ps2ConnectConfirmCount = 0u;
static uint8 g_ps2OffConfirmCount = 0u;

/*
 * LED is wired as active-low on the current board.
 * LOW  = ON
 * HIGH = OFF
 */
static void Ps2_LedConnected(void)
{
    IfxPort_setPinLow(LED_BLUE_PORT, LED_BLUE_PIN);
    IfxPort_setPinHigh(LED_RED_PORT, LED_RED_PIN);
}

static void Ps2_LedWaiting(void)
{
    IfxPort_setPinHigh(LED_BLUE_PORT, LED_BLUE_PIN);
    IfxPort_setPinLow(LED_RED_PORT, LED_RED_PIN);
}

static void Ps2_UpdateLedByStableState(boolean connected)
{
    boolean ledConnected = connected;

    if (g_statusLedMode == STATUS_LED_MODE_CAN)
    {
        ledConnected = (g_canRxAlive != 0u) ? TRUE : FALSE;
    }
    else if (g_statusLedMode == STATUS_LED_MODE_PS2_AND_CAN)
    {
        ledConnected = ((connected != FALSE) && (g_canRxAlive != 0u)) ? TRUE : FALSE;
    }
    else if (g_statusLedMode == STATUS_LED_MODE_PS2_OR_CAN)
    {
        ledConnected = ((connected != FALSE) || (g_canRxAlive != 0u)) ? TRUE : FALSE;
    }

    g_statusLedConnected = (ledConnected != FALSE) ? 1u : 0u;

    if (ledConnected != FALSE)
    {
        Ps2_LedConnected();
    }
    else
    {
        Ps2_LedWaiting();
    }
}

static uint32 Ps2_NowTick100us(void)
{
    return tick100us;
}

static uint32 Ps2_MsToTick100us(uint32 ms)
{
    return ms * 10u;
}

static uint32 Ps2_Tick100usToMs(uint32 ticks)
{
    return ticks / 10u;
}

static void Ps2_DelayUs(uint32 us)
{
    sint32 ticks = IfxStm_getTicksFromMicroseconds(&MODULE_STM0, us);

    if (ticks <= 0)
    {
        ticks = 1;
    }

    IfxStm_waitTicks(&MODULE_STM0, (uint32)ticks);
}

static void Ps2_DelayMs(uint32 ms)
{
    while (ms > 0u)
    {
        Ps2_DelayUs(1000u);
        ms--;
    }
}

static void Ps2_SetCmd(uint8 high)
{
    if (high != 0u)
    {
        IfxPort_setPinHigh(PS2_CMD_PORT, PS2_CMD_PIN);
        g_ps2AckermannCmdPin = 1u;
    }
    else
    {
        IfxPort_setPinLow(PS2_CMD_PORT, PS2_CMD_PIN);
        g_ps2AckermannCmdPin = 0u;
    }
}

static uint8 Ps2_ReadData(void)
{
    uint8 high = (IfxPort_getPinState(PS2_DATA_PORT, PS2_DATA_PIN) != FALSE) ? 1u : 0u;

    g_ps2AckermannDataPin = high;
    if (high != 0u)
    {
        g_ps2AckermannDataHighCount++;
    }
    else
    {
        g_ps2AckermannDataLowCount++;
    }

    return high;
}

static void Ps2_UpdatePinWatch(void)
{
    g_ps2AckermannDataPin = (IfxPort_getPinState(PS2_DATA_PORT, PS2_DATA_PIN) != FALSE) ? 1u : 0u;
    g_ps2AckermannCmdPin = (IfxPort_getPinState(PS2_CMD_PORT, PS2_CMD_PIN) != FALSE) ? 1u : 0u;
    g_ps2AckermannAttPin = (IfxPort_getPinState(PS2_ATT_PORT, PS2_ATT_PIN) != FALSE) ? 1u : 0u;
    g_ps2AckermannClkPin = (IfxPort_getPinState(PS2_CLK_PORT, PS2_CLK_PIN) != FALSE) ? 1u : 0u;
}

static uint8 Ps2_XferByte(uint8 tx)
{
    uint8 rx = 0u;
    uint8 bit;

    for (bit = 0u; bit < 8u; bit++)
    {
        Ps2_SetCmd(((tx >> bit) & 1u) ? 1u : 0u);
        Ps2_DelayUs(PS2_BIT_DELAY_US);
        IfxPort_setPinLow(PS2_CLK_PORT, PS2_CLK_PIN);
        g_ps2AckermannClkPin = 0u;
        Ps2_DelayUs(PS2_BIT_DELAY_US);
        if (Ps2_ReadData() != 0u)
        {
            rx |= (uint8)(1u << bit);
        }
        IfxPort_setPinHigh(PS2_CLK_PORT, PS2_CLK_PIN);
        g_ps2AckermannClkPin = 1u;
        Ps2_DelayUs(PS2_BIT_DELAY_US);
    }

    Ps2_SetCmd(1u);
    g_ps2AckermannLastRxByte = rx;
    return rx;
}

static void Ps2_Transaction(const uint8 *tx, uint8 *rx, uint8 len)
{
    uint8 i;

    if ((tx == 0) || (rx == 0) || (len == 0u))
    {
        return;
    }

    IfxPort_setPinHigh(PS2_CLK_PORT, PS2_CLK_PIN);
    Ps2_SetCmd(1u);
    IfxPort_setPinLow(PS2_ATT_PORT, PS2_ATT_PIN);
    g_ps2AckermannClkPin = 1u;
    g_ps2AckermannAttPin = 0u;
    Ps2_DelayUs(PS2_BIT_DELAY_US * 2u);

    for (i = 0u; i < len; i++)
    {
        rx[i] = Ps2_XferByte(tx[i]);
    }

    IfxPort_setPinHigh(PS2_ATT_PORT, PS2_ATT_PIN);
    g_ps2AckermannAttPin = 1u;
    Ps2_SetCmd(1u);
    Ps2_UpdatePinWatch();
    Ps2_DelayUs(PS2_BIT_DELAY_US * 2u);
}

static void Ps2_ConfigAnalog(void)
{
    static const uint8 enterCfg[5] = {0x01u, 0x43u, 0x00u, 0x01u, 0x00u};
    static const uint8 setAna[9]   = {0x01u, 0x44u, 0x00u, 0x01u, 0x03u, 0x00u, 0x00u, 0x00u, 0x00u};
    static const uint8 setMot[9]   = {0x01u, 0x4Du, 0x00u, 0x00u, 0x01u, 0xFFu, 0xFFu, 0xFFu, 0xFFu};
    static const uint8 exitCfg[9]  = {0x01u, 0x43u, 0x00u, 0x00u, 0x5Au, 0x5Au, 0x5Au, 0x5Au, 0x5Au};
    uint8 rx[PS2_PACKET_SIZE];
    uint8 retry;

    for (retry = 0u; retry < 3u; retry++)
    {
        Ps2_Transaction(enterCfg, rx, 5u);
        Ps2_DelayMs(10u);
        Ps2_Transaction(setAna, rx, 9u);
        Ps2_DelayMs(10u);
        Ps2_Transaction(setMot, rx, 9u);
        Ps2_DelayMs(10u);
        Ps2_Transaction(exitCfg, rx, 9u);
        Ps2_DelayMs(10u);
    }
}

static boolean Ps2_IsReceiverValidPacket(const uint8 *rx)
{
    if (rx == 0)
    {
        return FALSE;
    }

    if (!((rx[1] == 0x41u) || (rx[1] == 0x73u) || (rx[1] == 0x79u)))
    {
        return FALSE;
    }

    return (rx[2] == 0x5Au) ? TRUE : FALSE;
}

static boolean Ps2_ReadRaw(uint8 *rx)
{
    static const uint8 poll[PS2_PACKET_SIZE] = {0x01u, 0x42u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u};

    Ps2_Transaction(poll, rx, PS2_PACKET_SIZE);
    return Ps2_IsReceiverValidPacket(rx);
}

static boolean Ps2_IsPadDisconnectedPattern(const uint8 *rx)
{
    if (rx == 0)
    {
        return FALSE;
    }

    if ((rx[0] == 0xFFu) &&
        (rx[1] == 0x73u) &&
        (rx[2] == 0x5Au) &&
        (rx[3] == 0xFFu) &&
        (rx[4] == 0xFFu) &&
        (rx[5] == 0x80u) &&
        (rx[6] == 0x80u) &&
        (rx[7] == 0x80u) &&
        (rx[8] == 0x80u))
    {
        return TRUE;
    }

    return FALSE;
}

static boolean Ps2_IsPadConnectedPacket(const uint8 *rx)
{
    if (Ps2_IsReceiverValidPacket(rx) == FALSE)
    {
        return FALSE;
    }

    return (Ps2_IsPadDisconnectedPattern(rx) == FALSE) ? TRUE : FALSE;
}

static boolean Ps2_Analog(uint8 mode)
{
    return ((mode == 0x73u) || (mode == 0x79u)) ? TRUE : FALSE;
}

static uint16 Ps2_Pressed(const uint8 *rx)
{
    uint16 released = (uint16)(((uint16)rx[4] << 8) | rx[3]);
    return (uint16)(~released);
}

static uint8 Ps2_ApplyDeadzone(uint8 raw)
{
    sint16 centered = (sint16)raw - 128;

    if (centered < 0)
    {
        centered = (sint16)(-centered);
    }

    return ((uint16)centered <= PS2_AXIS_DEADZONE_RAW) ? 128u : raw;
}

static uint16 Ps2_ClampSteeringPulse(uint32 pulse)
{
    if (pulse < STEERING_SERVO_LEFT_US)
    {
        pulse = STEERING_SERVO_LEFT_US;
    }
    else if (pulse > STEERING_SERVO_RIGHT_US)
    {
        pulse = STEERING_SERVO_RIGHT_US;
    }

    return (uint16)pulse;
}

static uint16 Ps2_MapSteeringRawToPulse(uint8 raw)
{
    uint32 pulse;
    uint8 mapped = raw;

    if (g_ps2SteeringInvert != 0u)
    {
        mapped = (uint8)(255u - mapped);
    }

    if (g_ps2SteeringFixedPulseMode != 0u)
    {
        if (mapped <= g_ps2SteeringLeftThresholdRaw)
        {
            g_ps2SteeringDecision = PS2_STEERING_DECISION_LEFT;
            return Ps2_ClampSteeringPulse((uint32)g_ps2SteeringFixedLeftPulseUs);
        }

        if (mapped >= g_ps2SteeringRightThresholdRaw)
        {
            g_ps2SteeringDecision = PS2_STEERING_DECISION_RIGHT;
            return Ps2_ClampSteeringPulse((uint32)g_ps2SteeringFixedRightPulseUs);
        }

        g_ps2SteeringDecision = PS2_STEERING_DECISION_CENTER;
        return Ps2_ClampSteeringPulse((uint32)g_ps2SteeringFixedCenterPulseUs);
    }

    mapped = Ps2_ApplyDeadzone(mapped);
    if (mapped > 128u)
    {
        g_ps2SteeringDecision = PS2_STEERING_DECISION_RIGHT;
    }
    else if (mapped < 128u)
    {
        g_ps2SteeringDecision = PS2_STEERING_DECISION_LEFT;
    }
    else
    {
        g_ps2SteeringDecision = PS2_STEERING_DECISION_CENTER;
    }

    if (mapped >= 128u)
    {
        pulse = (uint32)STEERING_SERVO_CENTER_US +
                ((((uint32)(mapped - 128u) *
                   (uint32)(STEERING_SERVO_RIGHT_US - STEERING_SERVO_CENTER_US)) + 63u) / 127u);
    }
    else
    {
        pulse = (uint32)STEERING_SERVO_CENTER_US -
                ((((uint32)(128u - mapped) *
                   (uint32)(STEERING_SERVO_CENTER_US - STEERING_SERVO_LEFT_US)) + 64u) / 128u);
    }

    return Ps2_ClampSteeringPulse(pulse);
}

static uint16 Ps2_MapSteeringRawToPotTarget(uint8 raw)
{
    uint32 targetRaw;
    uint32 deltaRaw;
    uint8 mapped = raw;
    uint16 rawMin = g_steeringPotRawMin;
    uint16 rawCenter = g_steeringPotRawCenter;
    uint16 rawMax = g_steeringPotRawMax;

    if (g_ps2SteeringInvert != 0u)
    {
        mapped = (uint8)(255u - mapped);
    }

    if ((rawMin >= rawCenter) || (rawCenter >= rawMax))
    {
        return STEERING_ADC_RAW_CENTER;
    }

    if (g_ps2SteeringFixedPulseMode != 0u)
    {
        if (mapped <= g_ps2SteeringLeftThresholdRaw)
        {
            return rawMax;
        }

        if (mapped >= g_ps2SteeringRightThresholdRaw)
        {
            return rawMin;
        }

        return rawCenter;
    }

    mapped = Ps2_ApplyDeadzone(mapped);

    if (mapped >= 128u)
    {
        deltaRaw = ((uint32)(mapped - 128u) * (uint32)(rawCenter - rawMin) + 63u) / 127u;
        targetRaw = (uint32)rawCenter - deltaRaw;
    }
    else
    {
        deltaRaw = ((uint32)(128u - mapped) * (uint32)(rawMax - rawCenter) + 64u) / 128u;
        targetRaw = (uint32)rawCenter + deltaRaw;
    }

    if (targetRaw < rawMin)
    {
        targetRaw = rawMin;
    }
    else if (targetRaw > rawMax)
    {
        targetRaw = rawMax;
    }

    return (uint16)targetRaw;
}

static uint8 Ps2_SteeringRawFromPacket(const uint8 *rx, uint16 buttons)
{
    if (Ps2_Analog(rx[1]) != FALSE)
    {
        return rx[PS2_STEERING_AXIS_RX];
    }

    if ((buttons & PS2_BTN_LEFT) != 0u)
    {
        return 0u;
    }

    if ((buttons & PS2_BTN_RIGHT) != 0u)
    {
        return 255u;
    }

    return 128u;
}

static void Ps2_UpdateWatch(const uint8 *rx, boolean receiverValid, boolean padConnected)
{
    g_ps2AckermannRaw0 = rx[0];
    g_ps2AckermannRaw1 = rx[1];
    g_ps2AckermannRaw2 = rx[2];
    g_ps2AckermannRaw3 = rx[3];
    g_ps2AckermannRaw4 = rx[4];
    g_ps2AckermannRaw5 = rx[5];
    g_ps2AckermannRaw6 = rx[6];
    g_ps2AckermannRaw7 = rx[7];
    g_ps2AckermannRaw8 = rx[8];
    g_ps2AckermannMode = rx[1];
    g_ps2AckermannStatus = rx[2];
    g_ps2AckermannReceiverAlive = (receiverValid != FALSE) ? 1u : 0u;
    g_ps2AckermannPresent = (padConnected != FALSE) ? 1u : 0u;
    g_ps2AckermannButtons = Ps2_Pressed(rx);

    if ((receiverValid != FALSE) && (padConnected != FALSE) && (Ps2_Analog(rx[1]) != FALSE))
    {
        g_ps2AckermannRx = rx[5];
        g_ps2AckermannRy = rx[6];
        g_ps2AckermannLx = rx[7];
        g_ps2AckermannLy = rx[8];
    }
    else
    {
        g_ps2AckermannRx = 0x80u;
        g_ps2AckermannRy = 0x80u;
        g_ps2AckermannLx = 0x80u;
        g_ps2AckermannLy = 0x80u;
    }
}

static void Ps2_UpdateStableConnectionState(boolean receiverValid, boolean padConnected)
{
    boolean connectedCandidate = ((receiverValid != FALSE) && (padConnected != FALSE)) ? TRUE : FALSE;
    boolean offCandidate = ((receiverValid == FALSE) || (padConnected == FALSE)) ? TRUE : FALSE;

    if (connectedCandidate != FALSE)
    {
        if (g_ps2ConnectConfirmCount < PS2_CONNECT_CONFIRM_CNT)
        {
            g_ps2ConnectConfirmCount++;
        }
        g_ps2OffConfirmCount = 0u;
    }
    else if (offCandidate != FALSE)
    {
        if (g_ps2OffConfirmCount < PS2_OFF_CONFIRM_CNT)
        {
            g_ps2OffConfirmCount++;
        }
        g_ps2ConnectConfirmCount = 0u;
    }
    else
    {
        g_ps2ConnectConfirmCount = 0u;
        g_ps2OffConfirmCount = 0u;
        return;
    }

    if (g_ps2ConnectConfirmCount >= PS2_CONNECT_CONFIRM_CNT)
    {
        g_ps2AckermannStableConnected = 1u;
        Ps2_UpdateLedByStableState(TRUE);
    }

    if (g_ps2OffConfirmCount >= PS2_OFF_CONFIRM_CNT)
    {
        g_ps2AckermannStableConnected = 0u;
        Ps2_UpdateLedByStableState(FALSE);
    }
}

static void Ps2_StopOnLoss(void)
{
    g_ps2AckermannTargetKphX10 = 0u;
    g_ps2AckermannSteeringPulseUs = STEERING_SERVO_NO_JOYSTICK_US;
    g_ps2AckermannSteeringTargetRaw = g_steeringPotRawCenter;
    g_ps2PreviousPressedButtons = 0u;
    g_ps2SpeedTestUpArmed = FALSE;
    /* VehicleManager owns torque-off, center return, and source handover. */
}

static void Ps2_ApplyAckermann(const uint8 *rx)
{
    uint16 buttons = Ps2_Pressed(rx);
    uint8 steeringRaw = Ps2_SteeringRawFromPacket(rx, buttons);
    uint16 steeringPulseUs = Ps2_MapSteeringRawToPulse(steeringRaw);
    uint16 steeringTargetRaw = Ps2_MapSteeringRawToPotTarget(steeringRaw);
    sint16 targetKphX10 = 0;
    boolean speedIncrementEdge;
    boolean speedSelectionOnly;

    if (g_joystickDriveModeEnabled == 0u)
    {
        g_ps2PreviousPressedButtons = buttons;
        g_ps2SpeedTestUpArmed = FALSE;
        g_ps2AckermannTargetKphX10 = 0;
        g_ps2AckermannSteeringPulseUs = steeringPulseUs;
        g_ps2AckermannSteeringTargetRaw = steeringTargetRaw;
        return;
    }

    if ((buttons & PS2_BTN_UP) == 0u)
    {
        g_ps2SpeedTestUpArmed = TRUE;
    }

    speedIncrementEdge =
        (((buttons & PS2_BTN_UP) != 0u) &&
         ((g_ps2PreviousPressedButtons & PS2_BTN_UP) == 0u) &&
         (g_ps2SpeedTestUpArmed != FALSE)) ? TRUE : FALSE;
    speedSelectionOnly =
        ((buttons & (PS2_BTN_SELECT |
                     PS2_BTN_DOWN |
                     PS2_BTN_TRIANGLE |
                     PS2_BTN_SQUARE |
                     PS2_BTN_CIRCLE |
                     PS2_BTN_CROSS)) == 0u) ? TRUE : FALSE;

    if (speedIncrementEdge != FALSE)
    {
        if (speedSelectionOnly != FALSE)
        {
            if ((g_ps2SpeedTestSelectedKph < PS2_SPEED_TEST_MIN_KPH) ||
                (g_ps2SpeedTestSelectedKph >= PS2_SPEED_TEST_MAX_KPH))
            {
                g_ps2SpeedTestSelectedKph = PS2_SPEED_TEST_MIN_KPH;
            }
            else
            {
                g_ps2SpeedTestSelectedKph++;
            }
        }
        g_ps2SpeedTestUpArmed = FALSE;
    }
    g_ps2PreviousPressedButtons = buttons;

    if ((buttons & PS2_BTN_SELECT) != 0u)
    {
        targetKphX10 = 0;
    }
    else if ((buttons & PS2_BTN_UP) != 0u)
    {
        /* Speed selection is always torque-free, including UP+CROSS. */
        targetKphX10 = 0;
    }
    else if ((buttons & PS2_BTN_DOWN) != 0u)
    {
        targetKphX10 = PS2_REVERSE_TARGET_KPH_X10;
    }
    else if ((buttons & PS2_BTN_TRIANGLE) != 0u)
    {
        targetKphX10 = 40;
    }
    else if ((buttons & PS2_BTN_SQUARE) != 0u)
    {
        targetKphX10 = 20;
    }
    else if ((buttons & PS2_BTN_CIRCLE) != 0u)
    {
        targetKphX10 = 10;
    }
    else if ((buttons & PS2_BTN_CROSS) != 0u)
    {
        targetKphX10 = (sint16)g_ps2SpeedTestSelectedKph *
                       PS2_SPEED_TEST_KPH_TO_X10;
    }

    g_ps2AckermannTargetKphX10 = targetKphX10;
    g_ps2AckermannSteeringPulseUs = steeringPulseUs;
    g_ps2AckermannSteeringTargetRaw = steeringTargetRaw;
}

void Ps2Controller_Init(void)
{
    uint8 i;

    for (i = 0u; i < PS2_PACKET_SIZE; i++)
    {
        g_raw[i] = 0xFFu;
    }

    IfxPort_setPinModeInput(PS2_DATA_PORT, PS2_DATA_PIN, IfxPort_InputMode_pullUp);
    IfxPort_setPinModeOutput(PS2_CMD_PORT, PS2_CMD_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(PS2_ATT_PORT, PS2_ATT_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(PS2_CLK_PORT, PS2_CLK_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(LED_BLUE_PORT, LED_BLUE_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(LED_RED_PORT, LED_RED_PIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinHigh(PS2_CMD_PORT, PS2_CMD_PIN);
    IfxPort_setPinHigh(PS2_ATT_PORT, PS2_ATT_PIN);
    IfxPort_setPinHigh(PS2_CLK_PORT, PS2_CLK_PIN);

    g_present = FALSE;
    g_receiverAlive = FALSE;
    g_seenValid = FALSE;
    g_seenPadConnected = FALSE;
    g_ps2AckermannPresent = 0u;
    g_ps2AckermannReceiverAlive = 0u;
    g_ps2AckermannStableConnected = 0u;
    g_statusLedConnected = 0u;
    g_ps2ConnectConfirmCount = 0u;
    g_ps2OffConfirmCount = 0u;
    Ps2_LedWaiting();
    g_ps2AckermannPollCount = 0u;
    g_ps2AckermannDataHighCount = 0u;
    g_ps2AckermannDataLowCount = 0u;
    g_ps2AckermannTargetKphX10 = 0u;
    g_ps2SpeedTestSelectedKph = 0u;
    g_ps2PreviousPressedButtons = 0u;
    g_ps2SpeedTestUpArmed = FALSE;
    g_ps2AckermannSteeringPulseUs = STEERING_SERVO_NO_JOYSTICK_US;
    g_ps2AckermannSteeringTargetRaw = g_steeringPotRawCenter;
    Ps2_StopOnLoss();
    Ps2_UpdatePinWatch();

    Ps2_DelayMs(100u);
    Ps2_ConfigAnalog();

    g_lastPollTick100us = Ps2_NowTick100us();
    g_lastGoodTick100us = g_lastPollTick100us;
    g_lastPadGoodTick100us = g_lastPollTick100us;
}

void Ps2Controller_Poll(void)
{
    uint32 now = Ps2_NowTick100us();
    boolean receiverValid;
    boolean padConnected;

    if ((uint32)(now - g_lastPollTick100us) < Ps2_MsToTick100us(PS2_POLL_PERIOD_MS))
    {
        return;
    }

    g_lastPollTick100us = now;
    g_ps2AckermannPollCount++;
    receiverValid = Ps2_ReadRaw(g_raw);
    padConnected = (receiverValid != FALSE) ? Ps2_IsPadConnectedPacket(g_raw) : FALSE;
    Ps2_UpdateWatch(g_raw, receiverValid, padConnected);
    Ps2_UpdateStableConnectionState(receiverValid, padConnected);
    Ps2_UpdatePinWatch();

    if (receiverValid != FALSE)
    {
        g_receiverAlive = TRUE;
        g_seenValid = TRUE;
        g_lastGoodTick100us = now;
        g_ps2AckermannOkCount++;

        if (padConnected != FALSE)
        {
            g_present = TRUE;
            g_seenPadConnected = TRUE;
            g_lastPadGoodTick100us = now;
            Ps2_ApplyAckermann(g_raw);
        }
        else
        {
            /*
             * Version-16 behavior: keep manual ownership and the last valid
             * command during the short loss-confirmation window.
             */
            if ((g_ps2AckermannStableConnected == 0u) &&
                (g_present != FALSE))
            {
                Ps2_StopOnLoss();
            }
            if (g_ps2AckermannStableConnected == 0u)
            {
                g_present = FALSE;
            }
            g_ps2AckermannErrCount++;
        }
    }
    else
    {
        /*
         * Version-16 behavior: one bad receiver packet is not a disconnect.
         * Keep the last valid command until PS2_OFF_CONFIRM_CNT failures.
         */
        if ((g_ps2AckermannStableConnected == 0u) &&
            ((g_present != FALSE) || (g_receiverAlive != FALSE)))
        {
            Ps2_StopOnLoss();
        }
        if (g_ps2AckermannStableConnected == 0u)
        {
            g_present = FALSE;
            g_receiverAlive = FALSE;
        }
        g_ps2AckermannErrCount++;
    }

    g_ps2AckermannReceiverAlive = (g_receiverAlive != FALSE) ? 1u : 0u;
    g_ps2AckermannPresent = (g_present != FALSE) ? 1u : 0u;
    Ps2_UpdateLedByStableState((g_ps2AckermannStableConnected != 0u) ? TRUE : FALSE);
    g_ps2AckermannLastAgeMs = (g_present != FALSE) ?
        Ps2_Tick100usToMs((uint32)(now - g_lastPadGoodTick100us)) : 0xFFFFFFFFu;
}

boolean Ps2Controller_IsPresent(void)
{
    return g_present;
}

boolean Ps2Controller_IsLinkAlive(uint32 timeoutMs)
{
    uint32 ageTick100us;

    if (g_seenValid == FALSE)
    {
        return FALSE;
    }

    ageTick100us = (uint32)(Ps2_NowTick100us() - g_lastGoodTick100us);
    return (ageTick100us <= Ps2_MsToTick100us(timeoutMs)) ? TRUE : FALSE;
}

boolean Ps2Controller_IsPadConnected(uint32 timeoutMs)
{
    uint32 ageTick100us;

    if ((g_seenPadConnected == FALSE) || (g_present == FALSE))
    {
        return FALSE;
    }

    ageTick100us = (uint32)(Ps2_NowTick100us() - g_lastPadGoodTick100us);
    return (ageTick100us <= Ps2_MsToTick100us(timeoutMs)) ? TRUE : FALSE;
}
