#include "Steering_ADC.h"

#include "Board_Pins.h"
#include "CCU6_PWM_Generation.h"
#include "GPT12_Timer_Interrupt.h"
#include "Vadc/Adc/IfxVadc_Adc.h"

#define STEERING_ADC_STALE_TIMEOUT_TICK100US 500u

/*
 * Despike (median-of-3) + single-pole low-pass applied to the raw steering
 * ADC counts before they reach the steering position PID.  ALPHA is derived
 * from the 5 ms control-loop sample period and a ~10 Hz cutoff:
 *   RC    = 1 / (2*pi*fc)  = 1 / (2*pi*10) ~= 0.01592 s
 *   ALPHA = dt / (RC + dt) = 0.005 / (0.01592 + 0.005) ~= 0.239
 * The median stage removes single-sample outliers (EMI/glitch spikes)
 * without adding lag; the low-pass then smooths what remains.
 */
#define STEERING_ADC_FILTER_ALPHA (0.239f)

static IfxVadc_Adc g_vadc;
static IfxVadc_Adc_Group g_adcGroup;
static IfxVadc_Adc_Channel g_steeringChannel;
static SteeringAdcValue g_lastValue = {0u, 0.0f, 0.0f, FALSE};
static uint32 g_lastValidTick100us = 0u;
static uint8 g_hasValidSample = 0u;
static uint16 g_steeringAdcHistory[2] = {0u, 0u};
static float32 g_steeringAdcFilteredRaw = 0.0f;

volatile uint16  g_dbgSteeringAdcRaw = 0u;
volatile float32 g_dbgSteeringAdcMv = 0.0f;
volatile float32 g_dbgSteeringAdcDeg = 0.0f;
volatile uint8   g_dbgSteeringAdcValid = 0u;
volatile uint16  g_steeringPotRaw = 0u;
volatile float32 g_steeringPotMv = 0.0f;
volatile float32 g_steeringPotDeg = 0.0f;
volatile uint8   g_steeringPotValid = 0u;
volatile sint32  g_steeringPotRawFromCenter = 0;
volatile float32 g_steeringPotPercent = 0.0f;
volatile uint16  g_steeringPotRawMin = STEERING_ADC_RAW_MIN;
volatile uint16  g_steeringPotRawCenter = STEERING_ADC_RAW_CENTER;
volatile uint16  g_steeringPotRawMax = STEERING_ADC_RAW_MAX;
volatile uint8   g_steeringPotSnapshotCommand = 0u;
volatile uint8   g_steeringPotSnapshotHold = 0u;
volatile uint16  g_steeringPotSnapshotRaw = 0u;
volatile float32 g_steeringPotSnapshotMv = 0.0f;
volatile float32 g_steeringPotSnapshotDeg = 0.0f;
volatile sint32  g_steeringPotSnapshotRawFromCenter = 0;
volatile float32 g_steeringPotSnapshotPercent = 0.0f;
volatile uint16  g_steeringPotSnapshotPulseUs = STEERING_SERVO_CENTER_US;
volatile uint8   g_steeringPotSnapshotValid = 0u;
volatile uint32  g_steeringPotSnapshotCount = 0u;

static float32 SteeringAdc_RawToAngle(uint16 raw)
{
    uint16 rawCenter = g_steeringPotRawCenter;

    if (raw >= rawCenter)
    {
        uint16 rawMax = g_steeringPotRawMax;
        uint16 span = (rawMax > rawCenter) ? (uint16)(rawMax - rawCenter) : 0u;

        if (raw >= rawMax)
        {
            return STEERING_MAX_ANGLE_DEG;
        }

        if (span == 0u)
        {
            return 0.0f;
        }

        return ((float32)(raw - rawCenter) * STEERING_MAX_ANGLE_DEG) / (float32)span;
    }
    else
    {
        uint16 rawMin = g_steeringPotRawMin;
        uint16 span = (rawCenter > rawMin) ? (uint16)(rawCenter - rawMin) : 0u;

        if (raw <= rawMin)
        {
            return -STEERING_MAX_ANGLE_DEG;
        }

        if (span == 0u)
        {
            return 0.0f;
        }

        return -((float32)(rawCenter - raw) * STEERING_MAX_ANGLE_DEG) / (float32)span;
    }
}

static float32 SteeringAdc_RawToPercent(uint16 raw)
{
    uint16 rawCenter = g_steeringPotRawCenter;

    if (raw >= rawCenter)
    {
        uint16 rawMax = g_steeringPotRawMax;
        uint16 span = (rawMax > rawCenter) ? (uint16)(rawMax - rawCenter) : 0u;

        if (span == 0u)
        {
            return 0.0f;
        }

        return ((float32)(raw - rawCenter) * 100.0f) / (float32)span;
    }
    else
    {
        uint16 rawMin = g_steeringPotRawMin;
        uint16 span = (rawCenter > rawMin) ? (uint16)(rawCenter - rawMin) : 0u;

        if (span == 0u)
        {
            return 0.0f;
        }

        return -((float32)(rawCenter - raw) * 100.0f) / (float32)span;
    }
}

static void SteeringAdc_CopySnapshot(uint8 valid)
{
    g_steeringPotSnapshotRaw = g_steeringPotRaw;
    g_steeringPotSnapshotMv = g_steeringPotMv;
    g_steeringPotSnapshotDeg = g_steeringPotDeg;
    g_steeringPotSnapshotRawFromCenter = g_steeringPotRawFromCenter;
    g_steeringPotSnapshotPercent = g_steeringPotPercent;
    g_steeringPotSnapshotPulseUs = SteeringPwm_GetPulseUs();
    g_steeringPotSnapshotValid = valid;
}

static void SteeringAdc_UpdateSnapshot(uint8 valid)
{
    if (g_steeringPotSnapshotCommand != 0u)
    {
        SteeringAdc_CopySnapshot(valid);
        g_steeringPotSnapshotHold = 1u;
        g_steeringPotSnapshotCommand = 0u;
        g_steeringPotSnapshotCount++;
        return;
    }

    if (g_steeringPotSnapshotHold == 0u)
    {
        SteeringAdc_CopySnapshot(valid);
    }
}

static uint16 SteeringAdc_Median3(uint16 a, uint16 b, uint16 c)
{
    uint16 lo = (a < b) ? a : b;
    uint16 hi = (a < b) ? b : a;

    if (c < lo)
    {
        return lo;
    }

    if (c > hi)
    {
        return hi;
    }

    return c;
}

void SteeringAdc_Init(void)
{
    IfxVadc_Adc_Config adcConfig;
    IfxVadc_Adc_GroupConfig groupConfig;
    IfxVadc_Adc_ChannelConfig channelConfig;

    g_lastValue.raw = 0u;
    g_lastValue.millivolt = 0.0f;
    g_lastValue.angleDeg = 0.0f;
    g_lastValue.valid = FALSE;
    g_lastValidTick100us = 0u;
    g_hasValidSample = 0u;
    g_steeringAdcHistory[0] = 0u;
    g_steeringAdcHistory[1] = 0u;
    g_steeringAdcFilteredRaw = 0.0f;

    IfxVadc_Adc_initModuleConfig(&adcConfig, &MODULE_VADC);
    (void)IfxVadc_Adc_initModule(&g_vadc, &adcConfig);

    IfxVadc_Adc_initGroupConfig(&groupConfig, &g_vadc);
    groupConfig.groupId = STEERING_ADC_GROUP_ID;
    groupConfig.master = STEERING_ADC_GROUP_ID;
    groupConfig.arbiter.requestSlotQueueEnabled = TRUE;
    groupConfig.queueRequest.triggerConfig.gatingMode = IfxVadc_GatingMode_always;
    (void)IfxVadc_Adc_initGroup(&g_adcGroup, &groupConfig);

    IfxVadc_Adc_initChannelConfig(&channelConfig, &g_adcGroup);
    channelConfig.channelId = STEERING_ADC_CHANNEL_ID;
    channelConfig.resultRegister = STEERING_ADC_RESULT_REG;
    (void)IfxVadc_Adc_initChannel(&g_steeringChannel, &channelConfig);
    IfxVadc_Adc_addToQueue(&g_steeringChannel, IFXVADC_QUEUE_REFILL);
    IfxVadc_Adc_startQueue(&g_adcGroup);
}

boolean SteeringAdc_Update(void)
{
    Ifx_VADC_RES result = IfxVadc_Adc_getResult(&g_steeringChannel);
    uint16 sampleRaw;
    uint16 despikedRaw;

    if (result.B.VF == 0u)
    {
        uint8 sampleStillFresh =
            ((g_hasValidSample != 0u) &&
             ((tick100us - g_lastValidTick100us) <=
              STEERING_ADC_STALE_TIMEOUT_TICK100US)) ? 1u : 0u;

        g_lastValue.valid = (sampleStillFresh != 0u) ? TRUE : FALSE;
        g_dbgSteeringAdcValid = sampleStillFresh;
        g_steeringPotValid = sampleStillFresh;
        SteeringAdc_UpdateSnapshot(sampleStillFresh);
        return FALSE;
    }

    sampleRaw = (uint16)result.B.RESULT;

    if (g_hasValidSample == 0u)
    {
        /* First sample after init, or the first sample after a stale gap:
         * seed the despike history and the low-pass filter with this
         * sample so they don't have to slowly crawl up from zero/old data. */
        g_steeringAdcHistory[0] = sampleRaw;
        g_steeringAdcHistory[1] = sampleRaw;
        g_steeringAdcFilteredRaw = (float32)sampleRaw;
    }

    despikedRaw = SteeringAdc_Median3(g_steeringAdcHistory[0],
                                      g_steeringAdcHistory[1],
                                      sampleRaw);
    g_steeringAdcHistory[0] = g_steeringAdcHistory[1];
    g_steeringAdcHistory[1] = sampleRaw;

    g_steeringAdcFilteredRaw += STEERING_ADC_FILTER_ALPHA *
                                ((float32)despikedRaw - g_steeringAdcFilteredRaw);

    g_lastValue.raw = (uint16)(g_steeringAdcFilteredRaw + 0.5f);
    g_lastValue.millivolt = ((float32)g_lastValue.raw * STEERING_ADC_REF_MV) / STEERING_ADC_FULL_SCALE;
    g_lastValue.angleDeg = SteeringAdc_RawToAngle(g_lastValue.raw);
    g_lastValue.valid = TRUE;
    g_lastValidTick100us = tick100us;
    g_hasValidSample = 1u;

    g_dbgSteeringAdcRaw = g_lastValue.raw;
    g_dbgSteeringAdcMv = g_lastValue.millivolt;
    g_dbgSteeringAdcDeg = g_lastValue.angleDeg;
    g_dbgSteeringAdcValid = 1u;
    g_steeringPotRaw = g_lastValue.raw;
    g_steeringPotMv = g_lastValue.millivolt;
    g_steeringPotDeg = g_lastValue.angleDeg;
    g_steeringPotValid = 1u;
    g_steeringPotRawFromCenter = (sint32)g_lastValue.raw - (sint32)g_steeringPotRawCenter;
    g_steeringPotPercent = SteeringAdc_RawToPercent(g_lastValue.raw);
    SteeringAdc_UpdateSnapshot(1u);

    return TRUE;
}

SteeringAdcValue SteeringAdc_Get(void)
{
    return g_lastValue;
}
