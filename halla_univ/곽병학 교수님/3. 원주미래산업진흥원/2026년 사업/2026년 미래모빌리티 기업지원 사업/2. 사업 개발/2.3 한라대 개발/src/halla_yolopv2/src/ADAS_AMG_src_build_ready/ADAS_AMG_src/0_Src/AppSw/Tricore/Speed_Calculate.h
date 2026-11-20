#ifndef SPEED_CALCULATE_H_
#define SPEED_CALCULATE_H_

#include "Ifx_Types.h"

#define ENCODER_PPR               550U
#define ENCODER_X4                4U
#define MOTOR_GEAR_RATIO          (18.0f)
#define COUNTS_PER_REV            ((float32)ENCODER_PPR * (float32)ENCODER_X4 * MOTOR_GEAR_RATIO)
#define SPEED_BUFFER_FIXED_COUNT  20U
#define SPEED_BUFFER_MAX_COUNT    SPEED_BUFFER_FIXED_COUNT
#define SPEED_BUFFER_SCAN_COUNT   1U
#define SPEED_BUFFER_SCAN_MAX_COUNT SPEED_BUFFER_FIXED_COUNT
#define SPEED_SAMPLE_TICK100US    50U
#define SPEED_SAMPLE_PERIOD_SEC   ((float32)SPEED_SAMPLE_TICK100US * 0.0001f)
#define WHEEL_DIAMETER_M          (0.265f)

typedef struct
{
    sint32  count;
    sint32  deltaCount;
    float32 rawRpm;
    float32 avgRpm;
    uint32  rawTick100us;
    uint32  avgTick100us;
    boolean valid;
} WheelSpeedValue;

typedef struct
{
    WheelSpeedValue left;
    WheelSpeedValue right;
    float32         vehicleRpm;
    float32         vehicleMps;
    float32         vehicleRpmSigned;
    float32         vehicleMpsSigned;
    float32         vehiclePositionM;
    boolean         valid;
} VehicleSpeedValue;

typedef struct
{
    uint16  bufferSize;
    float32 avgMaxMinRpm;
    float32 minAvgRpm;
    float32 maxAvgRpm;
    float32 meanAvgRpm;
    float32 stddevAvgRpm;
    uint32  validSampleCount;
    uint8   dutyPercent;
    uint8   reserved0;
    uint8   reserved1;
    uint8   reserved2;
} SpeedBufferScanResult;

void SpeedCalc_Init(void);
boolean SpeedCalc_Get(VehicleSpeedValue *out);
void SpeedCalc_SetAverageBufferSize(uint16 bufferSize, boolean resetSamples);
void SpeedCalc_ResetBufferScan(void);
void SpeedCalc_SetBufferScanDutyPercent(uint8 dutyPercent);

extern volatile sint32  g_dbgSpeedLeftCount;
extern volatile sint32  g_dbgSpeedRightCount;
extern volatile sint32  g_dbgSpeedLeftDeltaCount;
extern volatile sint32  g_dbgSpeedRightDeltaCount;
extern volatile float32 g_dbgSpeedLeftRawRpm;
extern volatile float32 g_dbgSpeedRightRawRpm;
extern volatile float32 g_dbgSpeedLeftAvgRpm;
extern volatile float32 g_dbgSpeedRightAvgRpm;
extern volatile float32 g_dbgSpeedVehicleMps;
extern volatile float32 g_dbgSpeedCountsPerRev;
extern volatile float32 g_dbgSpeedGearRatio;
extern volatile float32 g_dbgSpeedWheelDiameterM;
extern volatile uint32  g_dbgSpeedDeltaTick100us;
extern volatile uint8   g_dbgSpeedValid;
extern volatile uint8   g_dbgSpeedBufferCount;
extern volatile uint16  g_speedAverageBufferSize;
extern volatile uint8   g_speedBufferScanEnable;
extern volatile uint8   g_speedBufferScanReset;
extern volatile uint8   g_speedBufferScanDutyPercent;
extern volatile uint32  g_speedBufferScanRawSampleCount;
extern volatile float32 g_speedBufferScanRawVehicleRpm;
extern volatile SpeedBufferScanResult g_speedBufferScan[SPEED_BUFFER_SCAN_COUNT];

#endif /* SPEED_CALCULATE_H_ */
