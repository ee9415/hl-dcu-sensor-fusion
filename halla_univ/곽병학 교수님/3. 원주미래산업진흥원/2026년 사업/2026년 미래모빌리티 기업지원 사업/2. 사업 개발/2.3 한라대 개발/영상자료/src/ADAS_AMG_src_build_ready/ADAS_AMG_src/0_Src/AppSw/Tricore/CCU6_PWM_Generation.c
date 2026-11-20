#include "CCU6_PWM_Generation.h"

#include "Board_Pins.h"
#include "Drive_PWM.h"
#include "Gtm/Atom/Pwm/IfxGtm_Atom_Pwm.h"
#include "Gtm/Std/IfxGtm.h"
#include "Gtm/Std/IfxGtm_Atom.h"
#include "Gtm/Std/IfxGtm_Cmu.h"
#include "_PinMap/IfxGtm_PinMap.h"

#define PWM_GTM_CLOCK_HZ           (1000000.0f)

static IfxGtm_Atom_Pwm_Driver g_steeringPwm;
static IfxGtm_Atom_Pwm_Config g_steeringPwmConfig;
static uint16 g_currentPulseUs = STEERING_SERVO_CENTER_US;
static boolean g_pwmStarted = FALSE;
volatile uint16 g_steeringPwmRequestedPulseUs = STEERING_SERVO_CENTER_US;
volatile uint16 g_steeringPwmAppliedPulseUs = STEERING_SERVO_CENTER_US;
volatile uint16 g_steeringPwmMinPulseUs = STEERING_SERVO_MIN_US;
volatile uint16 g_steeringPwmMaxPulseUs = STEERING_SERVO_MAX_US;
volatile uint16 g_steeringPwmPeriodUs = STEERING_SERVO_PERIOD_US;
volatile uint8  g_steeringPwmClampActive = 0u;

static IfxGtm_Atom_Pwm_Driver g_drivePwm1;
static IfxGtm_Atom_Pwm_Driver g_drivePwm2;
static IfxGtm_Atom_Pwm_Config g_drivePwm1Config;
static IfxGtm_Atom_Pwm_Config g_drivePwm2Config;
static uint8 g_currentDriveDutyPercent = 0u;
static boolean g_gtmPwmClockStarted = FALSE;
static boolean g_drivePwmStarted = FALSE;

static void PwmGtm_InitClockOnce(void)
{
    float32 moduleFrequency;

    if (g_gtmPwmClockStarted != FALSE)
    {
        return;
    }

    IfxGtm_enable(&MODULE_GTM);
    while (MODULE_GTM.CLC.B.DISS != 0u)
    {
    }

    moduleFrequency = IfxGtm_Cmu_getModuleFrequency(&MODULE_GTM);
    IfxGtm_Cmu_setGclkFrequency(&MODULE_GTM, moduleFrequency);
    IfxGtm_Cmu_setClkFrequency(&MODULE_GTM, IfxGtm_Cmu_Clk_0, PWM_GTM_CLOCK_HZ);
    IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK0);

    g_gtmPwmClockStarted = TRUE;
}

static uint16 SteeringPwm_ClampPulse(uint16 pulseUs)
{
    if (pulseUs < STEERING_SERVO_MIN_US)
    {
        return STEERING_SERVO_MIN_US;
    }

    if (pulseUs > STEERING_SERVO_MAX_US)
    {
        return STEERING_SERVO_MAX_US;
    }

    return pulseUs;
}

static void SteeringPwm_ApplyConfig(uint16 pulseUs)
{
    g_steeringPwmRequestedPulseUs = pulseUs;
    g_steeringPwmMinPulseUs = STEERING_SERVO_MIN_US;
    g_steeringPwmMaxPulseUs = STEERING_SERVO_MAX_US;
    g_steeringPwmPeriodUs = STEERING_SERVO_PERIOD_US;
    g_currentPulseUs = SteeringPwm_ClampPulse(pulseUs);
    g_steeringPwmAppliedPulseUs = g_currentPulseUs;
    g_steeringPwmClampActive = (g_currentPulseUs != pulseUs) ? 1u : 0u;
    g_steeringPwmConfig.dutyCycle = (uint32)g_currentPulseUs;
    (void)IfxGtm_Atom_Pwm_init(&g_steeringPwm, &g_steeringPwmConfig);
    IfxGtm_Atom_Ch_setClockSource(&MODULE_GTM.ATOM[STEERING_PWM_ATOM],
                                  STEERING_PWM_ATOM_CHANNEL,
                                  IfxGtm_Cmu_Clk_0);
    IfxGtm_Atom_Pwm_start(&g_steeringPwm, TRUE);
}

void SteeringPwm_Init(void)
{
    PwmGtm_InitClockOnce();

    IfxGtm_Atom_Pwm_initConfig(&g_steeringPwmConfig, &MODULE_GTM);
    g_steeringPwmConfig.atom = STEERING_PWM_ATOM;
    g_steeringPwmConfig.atomChannel = STEERING_PWM_ATOM_CHANNEL;
    g_steeringPwmConfig.period = STEERING_SERVO_PERIOD_US;
    g_steeringPwmConfig.dutyCycle = STEERING_SERVO_CENTER_US;
    g_steeringPwmConfig.pin.outputPin = &IfxGtm_ATOM0_0_TOUT8_P02_8_OUT;
    g_steeringPwmConfig.pin.outputMode = IfxPort_OutputMode_pushPull;
    g_steeringPwmConfig.pin.padDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1;
    g_steeringPwmConfig.synchronousUpdateEnabled = TRUE;
    g_steeringPwmConfig.signalLevel = Ifx_ActiveState_high;

    g_pwmStarted = TRUE;
    SteeringPwm_ApplyConfig(STEERING_SERVO_CENTER_US);
}

void SteeringPwm_SetPulseUs(uint16 pulseUs)
{
    if (g_pwmStarted == FALSE)
    {
        SteeringPwm_Init();
    }

    SteeringPwm_ApplyConfig(pulseUs);
}

void SteeringPwm_SetNormalized(float32 command)
{
    uint16 pulseUs;

    if (command > 1.0f)
    {
        command = 1.0f;
    }
    else if (command < -1.0f)
    {
        command = -1.0f;
    }

    if (command >= 0.0f)
    {
        pulseUs = (uint16)((float32)STEERING_SERVO_CENTER_US +
                           command * (float32)(STEERING_SERVO_RIGHT_US - STEERING_SERVO_CENTER_US));
    }
    else
    {
        pulseUs = (uint16)((float32)STEERING_SERVO_CENTER_US +
                           command * (float32)(STEERING_SERVO_CENTER_US - STEERING_SERVO_LEFT_US));
    }

    SteeringPwm_SetPulseUs(pulseUs);
}

uint16 SteeringPwm_GetPulseUs(void)
{
    return g_currentPulseUs;
}

static uint8 DrivePwm_ClampDuty(uint8 dutyPercent)
{
    if (dutyPercent > 100u)
    {
        return 100u;
    }

    return dutyPercent;
}

static void DrivePwm_SetPinLevel(Ifx_P *port, uint8 pin, uint8 level)
{
    if (level != 0u)
    {
        IfxPort_setPinHigh(port, pin);
    }
    else
    {
        IfxPort_setPinLow(port, pin);
    }
}

static void DrivePwm_SetForwardDirection(void)
{
    DrivePwm_SetPinLevel(DRIVE_A_FORWARD_PORT, DRIVE_A_FORWARD_PIN, 1u);
    DrivePwm_SetPinLevel(DRIVE_A_REVERSE_PORT, DRIVE_A_REVERSE_PIN, 0u);
#if (DRIVE_B_DIRECTION_SIGN < 0)
    DrivePwm_SetPinLevel(DRIVE_B_FORWARD_PORT, DRIVE_B_FORWARD_PIN, 0u);
    DrivePwm_SetPinLevel(DRIVE_B_REVERSE_PORT, DRIVE_B_REVERSE_PIN, 1u);
#else
    DrivePwm_SetPinLevel(DRIVE_B_FORWARD_PORT, DRIVE_B_FORWARD_PIN, 1u);
    DrivePwm_SetPinLevel(DRIVE_B_REVERSE_PORT, DRIVE_B_REVERSE_PIN, 0u);
#endif
}

static void DrivePwm_SetReverseDirection(void)
{
    DrivePwm_SetPinLevel(DRIVE_A_FORWARD_PORT, DRIVE_A_FORWARD_PIN, 0u);
    DrivePwm_SetPinLevel(DRIVE_A_REVERSE_PORT, DRIVE_A_REVERSE_PIN, 1u);
#if (DRIVE_B_DIRECTION_SIGN < 0)
    DrivePwm_SetPinLevel(DRIVE_B_FORWARD_PORT, DRIVE_B_FORWARD_PIN, 1u);
    DrivePwm_SetPinLevel(DRIVE_B_REVERSE_PORT, DRIVE_B_REVERSE_PIN, 0u);
#else
    DrivePwm_SetPinLevel(DRIVE_B_FORWARD_PORT, DRIVE_B_FORWARD_PIN, 0u);
    DrivePwm_SetPinLevel(DRIVE_B_REVERSE_PORT, DRIVE_B_REVERSE_PIN, 1u);
#endif
}

static void DrivePwm_ApplyDuty(uint8 dutyPercent)
{
    uint32 dutyTicks;

    g_currentDriveDutyPercent = DrivePwm_ClampDuty(dutyPercent);
    dutyTicks = ((uint32)DRIVE_PWM_PERIOD_US * (uint32)g_currentDriveDutyPercent) / 100u;

    g_drivePwm1Config.dutyCycle = dutyTicks;
    g_drivePwm2Config.dutyCycle = dutyTicks;

    (void)IfxGtm_Atom_Pwm_init(&g_drivePwm1, &g_drivePwm1Config);
    IfxGtm_Atom_Ch_setClockSource(&MODULE_GTM.ATOM[DRIVE_PWM_ATOM],
                                  DRIVE_PWM_ATOM_CHANNEL,
                                  IfxGtm_Cmu_Clk_0);
    IfxGtm_Atom_Pwm_start(&g_drivePwm1, TRUE);

    (void)IfxGtm_Atom_Pwm_init(&g_drivePwm2, &g_drivePwm2Config);
    IfxGtm_Atom_Ch_setClockSource(&MODULE_GTM.ATOM[DRIVE_PWM2_ATOM],
                                  DRIVE_PWM2_ATOM_CHANNEL,
                                  IfxGtm_Cmu_Clk_0);
    IfxGtm_Atom_Pwm_start(&g_drivePwm2, TRUE);
}

void DrivePwm_Init(void)
{
    PwmGtm_InitClockOnce();

    IfxPort_setPinMode(DRIVE_A_FORWARD_PORT, DRIVE_A_FORWARD_PIN, IfxPort_Mode_outputPushPullGeneral);
    IfxPort_setPinMode(DRIVE_A_REVERSE_PORT, DRIVE_A_REVERSE_PIN, IfxPort_Mode_outputPushPullGeneral);
    IfxPort_setPinMode(DRIVE_B_FORWARD_PORT, DRIVE_B_FORWARD_PIN, IfxPort_Mode_outputPushPullGeneral);
    IfxPort_setPinMode(DRIVE_B_REVERSE_PORT, DRIVE_B_REVERSE_PIN, IfxPort_Mode_outputPushPullGeneral);
    DrivePwm_SetForwardDirection();

    IfxGtm_Atom_Pwm_initConfig(&g_drivePwm1Config, &MODULE_GTM);
    g_drivePwm1Config.atom = DRIVE_PWM_ATOM;
    g_drivePwm1Config.atomChannel = DRIVE_PWM_ATOM_CHANNEL;
    g_drivePwm1Config.period = DRIVE_PWM_PERIOD_US;
    g_drivePwm1Config.dutyCycle = 0u;
    g_drivePwm1Config.pin.outputPin = &IfxGtm_ATOM1_1_TOUT1_P02_1_OUT;
    g_drivePwm1Config.pin.outputMode = IfxPort_OutputMode_pushPull;
    g_drivePwm1Config.pin.padDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1;
    g_drivePwm1Config.synchronousUpdateEnabled = TRUE;
    g_drivePwm1Config.signalLevel = Ifx_ActiveState_high;

    IfxGtm_Atom_Pwm_initConfig(&g_drivePwm2Config, &MODULE_GTM);
    g_drivePwm2Config.atom = DRIVE_PWM2_ATOM;
    g_drivePwm2Config.atomChannel = DRIVE_PWM2_ATOM_CHANNEL;
    g_drivePwm2Config.period = DRIVE_PWM_PERIOD_US;
    g_drivePwm2Config.dutyCycle = 0u;
    g_drivePwm2Config.pin.outputPin = &IfxGtm_ATOM1_0_TOUT0_P02_0_OUT;
    g_drivePwm2Config.pin.outputMode = IfxPort_OutputMode_pushPull;
    g_drivePwm2Config.pin.padDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1;
    g_drivePwm2Config.synchronousUpdateEnabled = TRUE;
    g_drivePwm2Config.signalLevel = Ifx_ActiveState_high;

    g_drivePwmStarted = TRUE;
    DrivePwm_ApplyDuty(0u);
}

void DrivePwm_SetDutyPercent(uint8 dutyPercent)
{
    if (g_drivePwmStarted == FALSE)
    {
        DrivePwm_Init();
    }

    DrivePwm_ApplyDuty(dutyPercent);
}

void DrivePwm_Forward(uint8 dutyPercent)
{
    if (g_drivePwmStarted == FALSE)
    {
        DrivePwm_Init();
    }

    DrivePwm_SetForwardDirection();
    DrivePwm_ApplyDuty(dutyPercent);
}

void DrivePwm_Reverse(uint8 dutyPercent)
{
    if (g_drivePwmStarted == FALSE)
    {
        DrivePwm_Init();
    }

    DrivePwm_SetReverseDirection();
    DrivePwm_ApplyDuty(dutyPercent);
}

void DrivePwm_Stop(void)
{
    DrivePwm_SetDutyPercent(0u);
}

uint8 DrivePwm_GetDutyPercent(void)
{
    return g_currentDriveDutyPercent;
}

void initCCU6(void)
{
    SteeringPwm_Init();
}

void startPWMGeneration(void)
{
    SteeringPwm_Init();
}

void updatePWMDutyCycle(uint8 dutyPercent)
{
    float32 normalized = ((float32)dutyPercent / 50.0f) - 1.0f;
    SteeringPwm_SetNormalized(normalized);
}

void initMotorDirectionPins(void)
{
}

void stopMotor(void)
{
    SteeringPwm_SetPulseUs(STEERING_SERVO_CENTER_US);
}
