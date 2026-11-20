#ifndef PS2_CONTROLLER_H_
#define PS2_CONTROLLER_H_

#include "Ifx_Types.h"

#define PS2_PACKET_SIZE         (9u)

#define PS2_BTN_SELECT          ((uint16)0x0001u)
#define PS2_BTN_L3              ((uint16)0x0002u)
#define PS2_BTN_R3              ((uint16)0x0004u)
#define PS2_BTN_START           ((uint16)0x0008u)
#define PS2_BTN_UP              ((uint16)0x0010u)
#define PS2_BTN_RIGHT           ((uint16)0x0020u)
#define PS2_BTN_DOWN            ((uint16)0x0040u)
#define PS2_BTN_LEFT            ((uint16)0x0080u)
#define PS2_BTN_L2              ((uint16)0x0100u)
#define PS2_BTN_R2              ((uint16)0x0200u)
#define PS2_BTN_L1              ((uint16)0x0400u)
#define PS2_BTN_R1              ((uint16)0x0800u)
#define PS2_BTN_TRIANGLE        ((uint16)0x1000u)
#define PS2_BTN_CIRCLE          ((uint16)0x2000u)
#define PS2_BTN_CROSS           ((uint16)0x4000u)
#define PS2_BTN_SQUARE          ((uint16)0x8000u)

extern volatile uint8   g_ps2AckermannPresent;
extern volatile uint8   g_ps2AckermannReceiverAlive;
extern volatile uint8   g_ps2AckermannMode;
extern volatile uint8   g_ps2AckermannStatus;
extern volatile uint16  g_ps2AckermannButtons;
extern volatile uint8   g_ps2AckermannRx;
extern volatile uint8   g_ps2AckermannRy;
extern volatile uint8   g_ps2AckermannLx;
extern volatile uint8   g_ps2AckermannLy;
extern volatile uint16  g_ps2AckermannSteeringPulseUs;
extern volatile uint16  g_ps2AckermannSteeringTargetRaw;
extern volatile sint16  g_ps2AckermannTargetKphX10;
extern volatile uint8   g_ps2SpeedTestSelectedKph;
extern volatile uint32  g_ps2AckermannOkCount;
extern volatile uint32  g_ps2AckermannErrCount;
extern volatile uint32  g_ps2AckermannLastAgeMs;
extern volatile uint32  g_ps2AckermannPollCount;
extern volatile uint32  g_ps2AckermannDataHighCount;
extern volatile uint32  g_ps2AckermannDataLowCount;
extern volatile uint8   g_ps2AckermannDataPin;
extern volatile uint8   g_ps2AckermannCmdPin;
extern volatile uint8   g_ps2AckermannAttPin;
extern volatile uint8   g_ps2AckermannClkPin;
extern volatile uint8   g_ps2AckermannLastRxByte;
extern volatile uint8   g_ps2AckermannRaw0;
extern volatile uint8   g_ps2AckermannRaw1;
extern volatile uint8   g_ps2AckermannRaw2;
extern volatile uint8   g_ps2AckermannRaw3;
extern volatile uint8   g_ps2AckermannRaw4;
extern volatile uint8   g_ps2AckermannRaw5;
extern volatile uint8   g_ps2AckermannRaw6;
extern volatile uint8   g_ps2AckermannRaw7;
extern volatile uint8   g_ps2AckermannRaw8;
extern volatile uint8   g_ps2AckermannStableConnected;
extern volatile uint8   g_statusLedMode;
extern volatile uint8   g_statusLedConnected;
extern volatile uint8   g_ps2SteeringFixedPulseMode;
extern volatile uint8   g_ps2SteeringLeftThresholdRaw;
extern volatile uint8   g_ps2SteeringRightThresholdRaw;
extern volatile uint8   g_ps2SteeringInvert;
extern volatile uint16  g_ps2SteeringFixedLeftPulseUs;
extern volatile uint16  g_ps2SteeringFixedCenterPulseUs;
extern volatile uint16  g_ps2SteeringFixedRightPulseUs;
extern volatile uint8   g_ps2SteeringDecision;

void Ps2Controller_Init(void);
void Ps2Controller_Poll(void);
boolean Ps2Controller_IsPresent(void);
boolean Ps2Controller_IsLinkAlive(uint32 timeoutMs);
boolean Ps2Controller_IsPadConnected(uint32 timeoutMs);

#endif /* PS2_CONTROLLER_H_ */
