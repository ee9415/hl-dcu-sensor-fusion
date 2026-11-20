#include "GPT12_Timer_Interrupt.h"

#include "IfxCpu_Irq.h"
#include "IfxGpt12.h"
#include "IfxPort.h"
#include "IfxSrc.h"

#include "Board_Pins.h"
#include "ERU_Interrupt.h"

#define ISR_PRIORITY_GPT12_T6        6
#define GPT12_ISR_PERIOD_US          100u
#define GPT12_TICKS_PER_5MS          50u

volatile uint32 tick100us = 0u;
volatile uint8  flag_5ms  = 0u;

static uint8  g_tick5msDiv   = 0u;
static uint16 g_t6ReloadValue = 1u;

IFX_INTERRUPT(interruptGpt12, 0, ISR_PRIORITY_GPT12_T6);

static void initIsrCheckPin(void)
{
    IfxPort_setPinModeOutput(ISR_CHECK_PORT,
                             ISR_CHECK_PIN,
                             IfxPort_OutputMode_pushPull,
                             IfxPort_OutputIdx_general);
    IfxPort_setPinLow(ISR_CHECK_PORT, ISR_CHECK_PIN);
}

void initGpt12Timer(void)
{
    volatile Ifx_SRC_SRCR *src;
    float32 t6Freq;
    uint32 reloadTicks;

    initIsrCheckPin();

    IfxGpt12_enableModule(&MODULE_GPT120);
    IfxGpt12_setGpt2BlockPrescaler(&MODULE_GPT120, IfxGpt12_Gpt2BlockPrescaler_16);
    IfxGpt12_T6_setMode(&MODULE_GPT120, IfxGpt12_Mode_timer);
    IfxGpt12_T6_setDirectionSource(&MODULE_GPT120, IfxGpt12_TimerDirectionSource_internal);
    IfxGpt12_T6_setTimerDirection(&MODULE_GPT120, IfxGpt12_TimerDirection_down);
    IfxGpt12_T6_setTimerPrescaler(&MODULE_GPT120, IfxGpt12_TimerInputPrescaler_16);

    t6Freq = IfxGpt12_T6_getFrequency(&MODULE_GPT120);
    reloadTicks = (uint32)(t6Freq * ((float32)GPT12_ISR_PERIOD_US / 1000000.0f));

    if (reloadTicks == 0u)
    {
        reloadTicks = 1u;
    }
    else if (reloadTicks > 0xFFFFu)
    {
        reloadTicks = 0xFFFFu;
    }

    g_t6ReloadValue = (uint16)reloadTicks;
    IfxGpt12_T6_setTimerValue(&MODULE_GPT120, g_t6ReloadValue);

    src = IfxGpt12_T6_getSrc(&MODULE_GPT120);
    IfxSrc_init(src, IfxSrc_Tos_cpu0, ISR_PRIORITY_GPT12_T6);
    IfxSrc_enable(src);

    IfxGpt12_T6_run(&MODULE_GPT120, IfxGpt12_TimerRun_start);
}

void interruptGpt12(void)
{
    tick100us++;
    IfxPort_togglePin(ISR_CHECK_PORT, ISR_CHECK_PIN);

    EncoderService_100us();

    g_tick5msDiv++;
    if (g_tick5msDiv >= GPT12_TICKS_PER_5MS)
    {
        g_tick5msDiv = 0u;

        if (flag_5ms < 0xFFu)
        {
            flag_5ms++;
        }
    }

    IfxGpt12_T6_setTimerValue(&MODULE_GPT120, g_t6ReloadValue);
}
