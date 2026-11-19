#ifndef GPT12_TIMER_INTERRUPT_H_
#define GPT12_TIMER_INTERRUPT_H_

#include "Ifx_Types.h"

void initGpt12Timer(void);
extern volatile uint32 tick100us;
extern volatile uint8  flag_5ms;

#endif /* GPT12_TIMER_INTERRUPT_H_ */
