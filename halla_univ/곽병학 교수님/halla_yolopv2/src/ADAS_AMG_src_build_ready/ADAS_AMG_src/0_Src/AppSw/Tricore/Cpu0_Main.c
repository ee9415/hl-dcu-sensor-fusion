#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "IfxPort.h"

#include "Board_Pins.h"
#include "CCU6_PWM_Generation.h"
#include "Can_SpeedTx.h"
#include "ControlLoop.h"
#include "Drive_PWM.h"
#include "ERU_Interrupt.h"
#include "GPT12_Timer_Interrupt.h"
#include "Ps2_Controller.h"
#include "Speed_Calculate.h"
#include "Steering_ADC.h"

IfxCpu_syncEvent g_cpuSyncEvent = 0;

#define MAIN_LOOP_PERIOD_TICK100US   (SPEED_SAMPLE_TICK100US)

static void initCheckPins(void)
{
    IfxPort_setPinModeOutput(ISR_CHECK_PORT,
                             ISR_CHECK_PIN,
                             IfxPort_OutputMode_pushPull,
                             IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(LOOP_CHECK_PORT,
                             LOOP_CHECK_PIN,
                             IfxPort_OutputMode_pushPull,
                             IfxPort_OutputIdx_general);
    IfxPort_setPinLow(ISR_CHECK_PORT, ISR_CHECK_PIN);
    IfxPort_setPinLow(LOOP_CHECK_PORT, LOOP_CHECK_PIN);
}

int core0_main(void)
{
    uint32 nextTick100us;

    IfxCpu_enableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);

    initCheckPins();
    initPeripheralsAndERU();
    SpeedCalc_Init();
#if (DRIVE_ONLY_TEST == 0u)
    SteeringAdc_Init();
    SteeringPwm_Init();
#endif
    DrivePwm_Init();
    CanSpeedTx_Init();
    ControlLoop_Init();
    Ps2Controller_Init();
    initGpt12Timer();

    nextTick100us = tick100us + MAIN_LOOP_PERIOD_TICK100US;

    while (1)
    {
        do
        {
        } while ((sint32)(tick100us - nextTick100us) < 0);

        CanSpeedTx_PollReceive();
        Ps2Controller_Poll();
        ControlLoop();
        nextTick100us += MAIN_LOOP_PERIOD_TICK100US;
    }

    return 0;
}
