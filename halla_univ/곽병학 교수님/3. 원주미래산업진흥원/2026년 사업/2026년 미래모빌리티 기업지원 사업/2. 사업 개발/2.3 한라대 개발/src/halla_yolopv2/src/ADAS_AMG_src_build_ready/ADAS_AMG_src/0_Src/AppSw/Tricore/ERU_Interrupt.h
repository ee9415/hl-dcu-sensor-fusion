#ifndef ERU_INTERRUPT_H_
#define ERU_INTERRUPT_H_

#include "Ifx_Types.h"

typedef struct
{
    sint32 leftCount;
    sint32 rightCount;
    uint8  leftState;
    uint8  rightState;
} EncoderSnapshot;

extern volatile sint32 encoderLeftPos;
extern volatile sint32 encoderRightPos;
extern volatile uint32 dbgEncoderPollCount;
extern volatile uint32 dbgEncoderLeftEventCount;
extern volatile uint32 dbgEncoderRightEventCount;
extern volatile uint32 dbgEncoderLeftRawChangeCount;
extern volatile uint32 dbgEncoderRightRawChangeCount;
extern volatile uint16 dbgEncoderLeftHwCount;
extern volatile uint16 dbgEncoderRightHwCount;
extern volatile sint32 dbgEncoderLeftHwDelta;
extern volatile sint32 dbgEncoderRightHwDelta;
extern volatile uint32 dbgEncoderLeftALowCount;
extern volatile uint32 dbgEncoderLeftBLowCount;
extern volatile uint32 dbgEncoderRightALowCount;
extern volatile uint32 dbgEncoderRightBLowCount;
extern volatile uint8  dbgEncoderLeftState;
extern volatile uint8  dbgEncoderRightState;
extern volatile uint8  dbgEncoderLeftAHigh;
extern volatile uint8  dbgEncoderLeftBHigh;
extern volatile uint8  dbgEncoderRightAHigh;
extern volatile uint8  dbgEncoderRightBHigh;

void initPeripheralsAndERU(void);
void EncoderService_100us(void);
void EncoderService_GetSnapshot(EncoderSnapshot *out);

#endif /* ERU_INTERRUPT_H_ */
