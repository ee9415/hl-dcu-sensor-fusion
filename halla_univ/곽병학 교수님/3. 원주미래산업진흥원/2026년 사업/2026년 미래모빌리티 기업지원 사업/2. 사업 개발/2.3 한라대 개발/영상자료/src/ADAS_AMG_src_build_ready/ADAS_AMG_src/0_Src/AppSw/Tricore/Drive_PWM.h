#ifndef DRIVE_PWM_H_
#define DRIVE_PWM_H_

#include "Ifx_Types.h"

#define DRIVE_PWM_PERIOD_US              50u
#define DRIVE_DUTY_STAGE_DURATION_MS     4000u

void DrivePwm_Init(void);
void DrivePwm_SetDutyPercent(uint8 dutyPercent);
void DrivePwm_Forward(uint8 dutyPercent);
void DrivePwm_Reverse(uint8 dutyPercent);
void DrivePwm_Stop(void);
uint8 DrivePwm_GetDutyPercent(void);

#endif /* DRIVE_PWM_H_ */
