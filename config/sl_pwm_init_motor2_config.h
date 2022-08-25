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

#ifndef PWM_INIT_MOTOR2_CONFIG_H
#define PWM_INIT_MOTOR2_CONFIG_H

#ifdef __cplusplus
extern "C"
{
#endif

// <<< Use Configuration Wizard in Context Menu >>>

// <h>PWM configuration

// <o SL_PWM_MOTOR2_FREQUENCY> PWM frequency [Hz]
// <i> Default: 10000
#define SL_PWM_MOTOR2_FREQUENCY       40000

// <o SL_PWM_MOTOR2_POLARITY> Polarity
// <PWM_ACTIVE_HIGH=> Active high
// <PWM_ACTIVE_LOW=> Active low
// <i> Default: PWM_ACTIVE_HIGH
#define SL_PWM_MOTOR2_POLARITY        PWM_ACTIVE_HIGH
// </h> end pwm configuration

// <<< end of configuration section >>>

// <<< sl:start pin_tool >>>

// <timer channel=OUTPUT> SL_PWM_MOTOR2
// $[TIMER_SL_PWM_MOTOR2]
#define SL_PWM_MOTOR2_PERIPHERAL                 TIMER1
#define SL_PWM_MOTOR2_PERIPHERAL_NO              1

#define SL_PWM_MOTOR2_OUTPUT_CHANNEL0             0
// TIMER1 CC0 on PD12
#define SL_PWM_MOTOR2_CH0_OUTPUT_PORT                gpioPortD
#define SL_PWM_MOTOR2_CH0_OUTPUT_PIN                 12
#define SL_PWM_MOTOR2_CH0_OUTPUT_LOC                 20

#define SL_PWM_MOTOR2_OUTPUT_CHANNEL1             1
// TIMER1 CC1 on PC9
#define SL_PWM_MOTOR2_CH1_OUTPUT_PORT                gpioPortC
#define SL_PWM_MOTOR2_CH1_OUTPUT_PIN                 9
#define SL_PWM_MOTOR2_CH1_OUTPUT_LOC                 13

#define SL_PWM_MOTOR2_OUTPUT_CHANNEL2             2
// TIMER1 CC2 on PC10
#define SL_PWM_MOTOR2_CH2_OUTPUT_PORT                gpioPortC
#define SL_PWM_MOTOR2_CH2_OUTPUT_PIN                 10
#define SL_PWM_MOTOR2_CH2_OUTPUT_LOC                 13
// [TIMER_SL_PWM_MOTOR2]$

// <<< sl:end pin_tool >>>

#ifdef __cplusplus
}
#endif

#endif // PWM_INIT_MOTOR2_CONFIG_H
