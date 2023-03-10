/***************************************************************************//**
 * @file
 * @brief PWM Driver
 *******************************************************************************
 * # License
 * <b>Copyright 2022 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#ifndef PWM_INIT_MOTOR1_CONFIG_H
#define PWM_INIT_MOTOR1_CONFIG_H

#ifdef __cplusplus
extern "C"
{
#endif

// <<< Use Configuration Wizard in Context Menu >>>

// <h>PWM configuration

// <o SL_PWM_MOTOR1_FREQUENCY> PWM frequency [Hz]
// <i> Default: 10000
#define SL_PWM_MOTOR1_FREQUENCY       40000

// <o SL_PWM_MOTOR1_POLARITY> Polarity
// <PWM_ACTIVE_HIGH=> Active high
// <PWM_ACTIVE_LOW=> Active low
// <i> Default: PWM_ACTIVE_HIGH
#define SL_PWM_MOTOR1_POLARITY        PWM_ACTIVE_HIGH
// </h> end pwm configuration

// <<< end of configuration section >>>

// <<< sl:start pin_tool >>>

// <timer channel=OUTPUT> SL_PWM_MOTOR1
// $[TIMER_SL_PWM_MOTOR1]
#define SL_PWM_MOTOR1_PERIPHERAL                 TIMER0
#define SL_PWM_MOTOR1_PERIPHERAL_NO              0

#define SL_PWM_MOTOR1_OUTPUT_CHANNEL0             0
// TIMER0 CC0 on PD9
#define SL_PWM_MOTOR1_CH0_OUTPUT_PORT                gpioPortD
#define SL_PWM_MOTOR1_CH0_OUTPUT_PIN                 9
#define SL_PWM_MOTOR1_CH0_OUTPUT_LOC                 17
// [TIMER_SL_PWM_MOTOR1]$

#define SL_PWM_MOTOR1_OUTPUT_CHANNEL1             1
// TIMER0 CC1 on PD10
#define SL_PWM_MOTOR1_CH1_OUTPUT_PORT                gpioPortD
#define SL_PWM_MOTOR1_CH1_OUTPUT_PIN                 10
#define SL_PWM_MOTOR1_CH1_OUTPUT_LOC                 17

#define SL_PWM_MOTOR1_OUTPUT_CHANNEL2             2
// TIMER0 CC2 on PD11
#define SL_PWM_MOTOR1_CH2_OUTPUT_PORT                gpioPortD
#define SL_PWM_MOTOR1_CH2_OUTPUT_PIN                 11
#define SL_PWM_MOTOR1_CH2_OUTPUT_LOC                 17
// <<< sl:end pin_tool >>>

#ifdef __cplusplus
}
#endif

#endif // PWM_INIT_MOTOR1_CONFIG_H
