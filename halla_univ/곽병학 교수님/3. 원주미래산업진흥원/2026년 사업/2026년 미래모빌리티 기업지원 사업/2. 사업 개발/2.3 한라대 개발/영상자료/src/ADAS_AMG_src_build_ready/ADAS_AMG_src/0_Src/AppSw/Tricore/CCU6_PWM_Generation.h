#ifndef CCU6_PWM_GENERATION_H_
#define CCU6_PWM_GENERATION_H_

#include "Ifx_Types.h"

#define STEERING_SERVO_PERIOD_US       20000u
#define STEERING_SERVO_LEFT_US         1350u  /* left +27 deg */
#define STEERING_SERVO_CENTER_US       1650u  /* measured center: 0x0672 */
#define STEERING_SERVO_NO_JOYSTICK_US  1650u
#define STEERING_SERVO_RIGHT_US        1950u  /* right -27 deg */
#define STEERING_SERVO_MIN_US          1350u
#define STEERING_SERVO_MAX_US          1950u

void SteeringPwm_Init(void);
void SteeringPwm_SetPulseUs(uint16 pulseUs);
void SteeringPwm_SetNormalized(float32 command);
uint16 SteeringPwm_GetPulseUs(void);

extern volatile uint16 g_steeringPwmRequestedPulseUs;
extern volatile uint16 g_steeringPwmAppliedPulseUs;
extern volatile uint16 g_steeringPwmMinPulseUs;
extern volatile uint16 g_steeringPwmMaxPulseUs;
extern volatile uint16 g_steeringPwmPeriodUs;
extern volatile uint8  g_steeringPwmClampActive;

void initCCU6(void);
void startPWMGeneration(void);
void updatePWMDutyCycle(uint8 dutyPercent);
void initMotorDirectionPins(void);
void stopMotor(void);

#endif /* CCU6_PWM_GENERATION_H_ */
