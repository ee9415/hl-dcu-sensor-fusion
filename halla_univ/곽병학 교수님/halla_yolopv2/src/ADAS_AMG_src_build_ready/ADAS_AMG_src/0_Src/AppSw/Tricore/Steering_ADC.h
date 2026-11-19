#ifndef STEERING_ADC_H_
#define STEERING_ADC_H_

#include "Ifx_Types.h"

#define STEERING_ADC_RAW_MIN       813u   /* right -27 deg: 0x032D at 1950 us */
#define STEERING_ADC_RAW_CENTER    3054u  /* center 0 deg: 0x0BEE at 1650 us */
#define STEERING_ADC_RAW_MAX       4018u  /* left +27 deg: 0x0FB2 at 1350 us */
#define STEERING_MAX_ANGLE_DEG     (27.0f)
#define STEERING_ADC_FULL_SCALE    (4095.0f)
#define STEERING_ADC_REF_MV        (3300.0f)

typedef struct
{
    uint16  raw;
    float32 millivolt;
    float32 angleDeg;
    boolean valid;
} SteeringAdcValue;

void SteeringAdc_Init(void);
boolean SteeringAdc_Update(void);
SteeringAdcValue SteeringAdc_Get(void);

extern volatile uint16  g_dbgSteeringAdcRaw;
extern volatile float32 g_dbgSteeringAdcMv;
extern volatile float32 g_dbgSteeringAdcDeg;
extern volatile uint8   g_dbgSteeringAdcValid;
extern volatile uint16  g_steeringPotRaw;
extern volatile float32 g_steeringPotMv;
extern volatile float32 g_steeringPotDeg;
extern volatile uint8   g_steeringPotValid;
extern volatile sint32  g_steeringPotRawFromCenter;
extern volatile float32 g_steeringPotPercent;
extern volatile uint16  g_steeringPotRawMin;
extern volatile uint16  g_steeringPotRawCenter;
extern volatile uint16  g_steeringPotRawMax;
extern volatile uint8   g_steeringPotSnapshotCommand;
extern volatile uint8   g_steeringPotSnapshotHold;
extern volatile uint16  g_steeringPotSnapshotRaw;
extern volatile float32 g_steeringPotSnapshotMv;
extern volatile float32 g_steeringPotSnapshotDeg;
extern volatile sint32  g_steeringPotSnapshotRawFromCenter;
extern volatile float32 g_steeringPotSnapshotPercent;
extern volatile uint16  g_steeringPotSnapshotPulseUs;
extern volatile uint8   g_steeringPotSnapshotValid;
extern volatile uint32  g_steeringPotSnapshotCount;

#endif /* STEERING_ADC_H_ */
