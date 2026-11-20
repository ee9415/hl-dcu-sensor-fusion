#include "ERU_Interrupt.h"

#include "Board_Pins.h"
#include "IfxCpu.h"
#include "IfxGpt12.h"
#include "IfxPort.h"

volatile sint32 encoderLeftPos = 0;
volatile sint32 encoderRightPos = 0;
volatile uint32 dbgEncoderPollCount = 0u;
volatile uint32 dbgEncoderLeftEventCount = 0u;
volatile uint32 dbgEncoderRightEventCount = 0u;
volatile uint32 dbgEncoderLeftRawChangeCount = 0u;
volatile uint32 dbgEncoderRightRawChangeCount = 0u;
volatile uint16 dbgEncoderLeftHwCount = 0u;
volatile uint16 dbgEncoderRightHwCount = 0u;
volatile sint32 dbgEncoderLeftHwDelta = 0;
volatile sint32 dbgEncoderRightHwDelta = 0;
volatile uint32 dbgEncoderLeftALowCount = 0u;
volatile uint32 dbgEncoderLeftBLowCount = 0u;
volatile uint32 dbgEncoderRightALowCount = 0u;
volatile uint32 dbgEncoderRightBLowCount = 0u;
volatile uint8  dbgEncoderLeftState = 0u;
volatile uint8  dbgEncoderRightState = 0u;
volatile uint8  dbgEncoderLeftAHigh = 0u;
volatile uint8  dbgEncoderLeftBHigh = 0u;
volatile uint8  dbgEncoderRightAHigh = 0u;
volatile uint8  dbgEncoderRightBHigh = 0u;

static uint8 g_prevLeftState = 0u;
static uint8 g_prevRightState = 0u;
static uint16 g_prevLeftHwCount = 0u;
static uint16 g_prevRightHwCount = 0u;

static uint8 EncoderService_ReadState(Ifx_P *aPort,
                                      uint8 aPin,
                                      Ifx_P *bPort,
                                      uint8 bPin)
{
    uint8 a = IfxPort_getPinState(aPort, aPin) ? 1u : 0u;
    uint8 b = IfxPort_getPinState(bPort, bPin) ? 1u : 0u;

    return (uint8)((a << 1u) | b);
}

static sint32 EncoderService_ComputeDelta(uint16 previousCount, uint16 currentCount)
{
    return (sint32)((sint16)(currentCount - previousCount));
}

static void EncoderService_InitGpt12Counters(void)
{
    IfxGpt12_enableModule(&MODULE_GPT120);
    IfxGpt12_setGpt1BlockPrescaler(&MODULE_GPT120, IfxGpt12_Gpt1BlockPrescaler_4);

    IfxGpt12_T3_run(&MODULE_GPT120, IfxGpt12_TimerRun_stop);
    IfxGpt12_T2_run(&MODULE_GPT120, IfxGpt12_TimerRun_stop);

    IfxGpt12_initTxInPinWithPadLevel(&IfxGpt120_T3INA_P02_6_IN,
                                     IfxPort_InputMode_pullUp,
                                     IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxGpt12_initTxEudInPinWithPadLevel(&IfxGpt120_T3EUDA_P02_7_IN,
                                        IfxPort_InputMode_pullUp,
                                        IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxGpt12_initTxInPinWithPadLevel(&IfxGpt120_T2INB_P33_7_IN,
                                     IfxPort_InputMode_pullUp,
                                     IfxPort_PadDriver_cmosAutomotiveSpeed1);
    IfxGpt12_initTxEudInPinWithPadLevel(&IfxGpt120_T2EUDB_P33_6_IN,
                                        IfxPort_InputMode_pullUp,
                                        IfxPort_PadDriver_cmosAutomotiveSpeed1);

    IfxGpt12_T3_setTimerValue(&MODULE_GPT120, 0u);
    IfxGpt12_T3_setMode(&MODULE_GPT120, IfxGpt12_Mode_incrementalInterfaceEdgeDetection);
    IfxGpt12_T3_setIncrementalInterfaceInputMode(&MODULE_GPT120,
                                                 IfxGpt12_IncrementalInterfaceInputMode_bothEdgesTxINOrTxEUD);
    IfxGpt12_T3_setDirectionSource(&MODULE_GPT120, IfxGpt12_TimerDirectionSource_external);
    IfxGpt12_T3_setTimerDirection(&MODULE_GPT120, IfxGpt12_TimerDirection_up);

    IfxGpt12_T2_setTimerValue(&MODULE_GPT120, 0u);
    IfxGpt12_T2_setMode(&MODULE_GPT120, IfxGpt12_Mode_incrementalInterfaceEdgeDetection);
    IfxGpt12_T2_setIncrementalInterfaceInputMode(&MODULE_GPT120,
                                                 IfxGpt12_IncrementalInterfaceInputMode_bothEdgesTxINOrTxEUD);
    IfxGpt12_T2_setDirectionSource(&MODULE_GPT120, IfxGpt12_TimerDirectionSource_external);
    IfxGpt12_T2_setTimerDirection(&MODULE_GPT120, IfxGpt12_TimerDirection_up);

    IfxGpt12_T3_run(&MODULE_GPT120, IfxGpt12_TimerRun_start);
    IfxGpt12_T2_run(&MODULE_GPT120, IfxGpt12_TimerRun_start);
}

void initPeripheralsAndERU(void)
{
    EncoderService_InitGpt12Counters();

    encoderLeftPos = 0;
    encoderRightPos = 0;
    dbgEncoderPollCount = 0u;
    dbgEncoderLeftEventCount = 0u;
    dbgEncoderRightEventCount = 0u;
    dbgEncoderLeftRawChangeCount = 0u;
    dbgEncoderRightRawChangeCount = 0u;
    dbgEncoderLeftHwCount = 0u;
    dbgEncoderRightHwCount = 0u;
    dbgEncoderLeftHwDelta = 0;
    dbgEncoderRightHwDelta = 0;
    dbgEncoderLeftALowCount = 0u;
    dbgEncoderLeftBLowCount = 0u;
    dbgEncoderRightALowCount = 0u;
    dbgEncoderRightBLowCount = 0u;

    g_prevLeftState = EncoderService_ReadState(ENCODER_LEFT_A_PORT,
                                               ENCODER_LEFT_A_PIN,
                                               ENCODER_LEFT_B_PORT,
                                               ENCODER_LEFT_B_PIN);
    g_prevRightState = EncoderService_ReadState(ENCODER_RIGHT_A_PORT,
                                                ENCODER_RIGHT_A_PIN,
                                                ENCODER_RIGHT_B_PORT,
                                                ENCODER_RIGHT_B_PIN);

    dbgEncoderLeftState = g_prevLeftState;
    dbgEncoderRightState = g_prevRightState;
    dbgEncoderLeftAHigh = (uint8)((g_prevLeftState >> 1u) & 0x1u);
    dbgEncoderLeftBHigh = (uint8)(g_prevLeftState & 0x1u);
    dbgEncoderRightAHigh = (uint8)((g_prevRightState >> 1u) & 0x1u);
    dbgEncoderRightBHigh = (uint8)(g_prevRightState & 0x1u);
    g_prevLeftHwCount = IfxGpt12_T3_getTimerValue(&MODULE_GPT120);
    g_prevRightHwCount = IfxGpt12_T2_getTimerValue(&MODULE_GPT120);
    dbgEncoderLeftHwCount = g_prevLeftHwCount;
    dbgEncoderRightHwCount = g_prevRightHwCount;
}

void EncoderService_100us(void)
{
    uint8 leftState;
    uint8 rightState;
    uint16 leftHwCount;
    uint16 rightHwCount;
    sint32 leftDelta;
    sint32 rightDelta;

    leftState = EncoderService_ReadState(ENCODER_LEFT_A_PORT,
                                         ENCODER_LEFT_A_PIN,
                                         ENCODER_LEFT_B_PORT,
                                         ENCODER_LEFT_B_PIN);
    rightState = EncoderService_ReadState(ENCODER_RIGHT_A_PORT,
                                          ENCODER_RIGHT_A_PIN,
                                          ENCODER_RIGHT_B_PORT,
                                          ENCODER_RIGHT_B_PIN);

    leftHwCount = IfxGpt12_T3_getTimerValue(&MODULE_GPT120);
    rightHwCount = IfxGpt12_T2_getTimerValue(&MODULE_GPT120);
    leftDelta = EncoderService_ComputeDelta(g_prevLeftHwCount, leftHwCount);
    rightDelta = EncoderService_ComputeDelta(g_prevRightHwCount, rightHwCount);
    g_prevLeftHwCount = leftHwCount;
    g_prevRightHwCount = rightHwCount;
    dbgEncoderLeftHwCount = leftHwCount;
    dbgEncoderRightHwCount = rightHwCount;
    dbgEncoderLeftHwDelta = leftDelta;
    dbgEncoderRightHwDelta = rightDelta;

    if (leftState != g_prevLeftState)
    {
        dbgEncoderLeftRawChangeCount++;
    }

    if (rightState != g_prevRightState)
    {
        dbgEncoderRightRawChangeCount++;
    }

    g_prevLeftState = leftState;
    g_prevRightState = rightState;
    dbgEncoderLeftState = leftState;
    dbgEncoderRightState = rightState;
    dbgEncoderLeftAHigh = (uint8)((leftState >> 1u) & 0x1u);
    dbgEncoderLeftBHigh = (uint8)(leftState & 0x1u);
    dbgEncoderRightAHigh = (uint8)((rightState >> 1u) & 0x1u);
    dbgEncoderRightBHigh = (uint8)(rightState & 0x1u);
    dbgEncoderPollCount++;

    if (dbgEncoderLeftAHigh == 0u)
    {
        dbgEncoderLeftALowCount++;
    }

    if (dbgEncoderLeftBHigh == 0u)
    {
        dbgEncoderLeftBLowCount++;
    }

    if (dbgEncoderRightAHigh == 0u)
    {
        dbgEncoderRightALowCount++;
    }

    if (dbgEncoderRightBHigh == 0u)
    {
        dbgEncoderRightBLowCount++;
    }

    if (leftDelta != 0)
    {
        encoderLeftPos += leftDelta * (sint32)ENCODER_LEFT_COUNT_SIGN;
        dbgEncoderLeftEventCount += (uint32)((leftDelta < 0) ? -leftDelta : leftDelta);
    }

    if (rightDelta != 0)
    {
        encoderRightPos += rightDelta * (sint32)ENCODER_RIGHT_COUNT_SIGN;
        dbgEncoderRightEventCount += (uint32)((rightDelta < 0) ? -rightDelta : rightDelta);
    }
}

void EncoderService_GetSnapshot(EncoderSnapshot *out)
{
    boolean intState;

    if (out == 0)
    {
        return;
    }

    intState = IfxCpu_disableInterrupts();
    out->leftCount = encoderLeftPos;
    out->rightCount = encoderRightPos;
    out->leftState = dbgEncoderLeftState;
    out->rightState = dbgEncoderRightState;
    IfxCpu_restoreInterrupts(intState);
}
