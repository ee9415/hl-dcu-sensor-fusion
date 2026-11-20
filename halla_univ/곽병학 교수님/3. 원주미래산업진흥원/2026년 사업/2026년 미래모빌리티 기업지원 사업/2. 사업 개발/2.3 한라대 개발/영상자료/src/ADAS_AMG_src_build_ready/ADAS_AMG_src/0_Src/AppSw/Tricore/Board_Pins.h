#ifndef BOARD_PINS_H_
#define BOARD_PINS_H_

#include "Gtm/Std/IfxGtm_Atom.h"
#include "IfxPort.h"
#include "Vadc/Std/IfxVadc.h"

/*
 * AURIX TC275 Lite Kit wiring for the Ackermann car.
 *
 * Drive motor PWM:
 *   Motor A PWM1 = P02.1 / TOUT1
 *   Motor B PWM2 = P02.0 / TOUT0
 *
 * Drive motor direction:
 *   INA1 = P33.9,  INA2 = P33.11
 *   INB1 = P33.8,  INB2 = P33.10
 *
 * Wheel encoders:
 *   Encoder A PhA = P02.6, PhB = P02.7
 *   Encoder B PhA = P33.7, PhB = P33.6
 *
 * UART to ROS PC:
 *   RX = P14.0, TX = P14.1
 *
 * Emergency stop:
 *   E-STOP = P20.6, LOW = stop, internal pull-up enabled in ControlLoop_Init().
 *
 * Steering feedback:
 *   ADC feedback must be moved to AN1. P00.6 is LED2/GPIO, not ADC.
 *
 * Battery voltage:
 *   ADC = P40.1 / AN41 / VADC G3CH1. Use an external divider so the pin never exceeds 5 V.
 *
 * Steering servo PWM:
 *   Default is mikroBus PWM = P02.8 / TOUT8.
 *
 * PS2 6-pin wireless receiver:
 *   DATA = P10.1, CMD = P10.3, ATT/SEL/CS = P10.0, CLK = P10.2
 *
 * PS2 status LEDs:
 *   Connected = blue LED on P33.1
 *   Disconnected = red LED on P33.2
 */

#define ENCODER_LEFT_A_PORT        (&MODULE_P02)
#define ENCODER_LEFT_A_PIN         ((uint8)6)
#define ENCODER_LEFT_B_PORT        (&MODULE_P02)
#define ENCODER_LEFT_B_PIN         ((uint8)7)

#define ENCODER_RIGHT_A_PORT       (&MODULE_P33)
#define ENCODER_RIGHT_A_PIN        ((uint8)7)
#define ENCODER_RIGHT_B_PORT       (&MODULE_P33)
#define ENCODER_RIGHT_B_PIN        ((uint8)6)

#define ENCODER_LEFT_COUNT_SIGN    (-1)
#define ENCODER_RIGHT_COUNT_SIGN   (-1)

#define DRIVE_ONLY_TEST            (0u)
#define ESTOP_ENABLE               (0u)
#define CAN_TRANSCEIVER_ENABLE     (1u)

#define STEERING_ADC_GROUP_ID      IfxVadc_GroupId_0
#define STEERING_ADC_CHANNEL_ID    IfxVadc_ChannelId_1
#define STEERING_ADC_RESULT_REG    IfxVadc_ChannelResult_1

#define BATTERY_ADC_GROUP_ID       IfxVadc_GroupId_3
#define BATTERY_ADC_CHANNEL_ID     IfxVadc_ChannelId_1
#define BATTERY_ADC_RESULT_REG     IfxVadc_ChannelResult_1

#define STEERING_PWM_ATOM          IfxGtm_Atom_0
#define STEERING_PWM_ATOM_CHANNEL  IfxGtm_Atom_Ch_0

#define DRIVE_PWM_ATOM             IfxGtm_Atom_1
#define DRIVE_PWM_ATOM_CHANNEL     IfxGtm_Atom_Ch_1
#define DRIVE_PWM2_ATOM            IfxGtm_Atom_1
#define DRIVE_PWM2_ATOM_CHANNEL    IfxGtm_Atom_Ch_0

#define DRIVE_A_FORWARD_PORT       (&MODULE_P33)
#define DRIVE_A_FORWARD_PIN        ((uint8)9)
#define DRIVE_A_REVERSE_PORT       (&MODULE_P33)
#define DRIVE_A_REVERSE_PIN        ((uint8)11)
#define DRIVE_B_FORWARD_PORT       (&MODULE_P33)
#define DRIVE_B_FORWARD_PIN        ((uint8)8)
#define DRIVE_B_REVERSE_PORT       (&MODULE_P33)
#define DRIVE_B_REVERSE_PIN        ((uint8)10)

/* Motor B is mounted/wired opposite to motor A. */
#define DRIVE_B_DIRECTION_SIGN     (-1)

#define UART_RX_PORT               (&MODULE_P14)
#define UART_RX_PIN                ((uint8)0)
#define UART_TX_PORT               (&MODULE_P14)
#define UART_TX_PIN                ((uint8)1)

#define ESTOP_PORT                 (&MODULE_P20)
#define ESTOP_PIN                  ((uint8)6)

#define CAN_TRANSCEIVER_NEN_PORT   (&MODULE_P20)
#define CAN_TRANSCEIVER_NEN_PIN    ((uint8)6)

#define ISR_CHECK_PORT             (&MODULE_P10)
#define ISR_CHECK_PIN              ((uint8)4)
#define LOOP_CHECK_PORT            (&MODULE_P00)
#define LOOP_CHECK_PIN             ((uint8)1)

#define PS2_DATA_PORT              (&MODULE_P10)
#define PS2_DATA_PIN               ((uint8)1)
#define PS2_CMD_PORT               (&MODULE_P10)
#define PS2_CMD_PIN                ((uint8)3)
#define PS2_ATT_PORT               (&MODULE_P10)
#define PS2_ATT_PIN                ((uint8)0)
#define PS2_CLK_PORT               (&MODULE_P10)
#define PS2_CLK_PIN                ((uint8)2)

#define LED_BLUE_PORT              (&MODULE_P33)
#define LED_BLUE_PIN               ((uint8)1)
#define LED_RED_PORT               (&MODULE_P33)
#define LED_RED_PIN                ((uint8)2)

#endif /* BOARD_PINS_H_ */
