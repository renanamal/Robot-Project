/***************************************************************************//**
 * @file
 * @brief PWM Driver Instance Initialization
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
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

#include "sl_pwm.h"

#include "sl_pwm_init_motor1_config.h"

#include "sl_pwm_init_motor2_config.h"


#include "em_gpio.h"


sl_pwm_instance_t sl_pwm_motor1_ch0 = {
  .timer = SL_PWM_MOTOR1_PERIPHERAL,
  .channel = (uint8_t)(SL_PWM_MOTOR1_OUTPUT_CHANNEL0),
  .port = (uint8_t)(SL_PWM_MOTOR1_CH0_OUTPUT_PORT),
  .pin = (uint8_t)(SL_PWM_MOTOR1_CH0_OUTPUT_PIN),
#if defined(SL_PWM_MOTOR1_CH0_OUTPUT_LOC)
  .location = (uint8_t)(SL_PWM_MOTOR1_CH0_OUTPUT_LOC),
#endif
};

sl_pwm_instance_t sl_pwm_motor1_ch1 = {
  .timer = SL_PWM_MOTOR1_PERIPHERAL,
  .channel = (uint8_t)(SL_PWM_MOTOR1_OUTPUT_CHANNEL1),
  .port = (uint8_t)(SL_PWM_MOTOR1_CH1_OUTPUT_PORT),
  .pin = (uint8_t)(SL_PWM_MOTOR1_CH1_OUTPUT_PIN),
#if defined(SL_PWM_MOTOR1_CH1_OUTPUT_LOC)
  .location = (uint8_t)(SL_PWM_MOTOR1_CH1_OUTPUT_LOC),
#endif
};

sl_pwm_instance_t sl_pwm_motor1_ch2 = {
  .timer = SL_PWM_MOTOR1_PERIPHERAL,
  .channel = (uint8_t)(SL_PWM_MOTOR1_OUTPUT_CHANNEL2),
  .port = (uint8_t)(SL_PWM_MOTOR1_CH2_OUTPUT_PORT),
  .pin = (uint8_t)(SL_PWM_MOTOR1_CH2_OUTPUT_PIN),
#if defined(SL_PWM_MOTOR1_CH2_OUTPUT_LOC)
  .location = (uint8_t)(SL_PWM_MOTOR1_CH2_OUTPUT_LOC),
#endif
};


// motor2

sl_pwm_instance_t sl_pwm_motor2_ch0 = {
  .timer = SL_PWM_MOTOR2_PERIPHERAL,
  .channel = (uint8_t)(SL_PWM_MOTOR2_OUTPUT_CHANNEL0),
  .port = (uint8_t)(SL_PWM_MOTOR2_CH0_OUTPUT_PORT),
  .pin = (uint8_t)(SL_PWM_MOTOR2_CH0_OUTPUT_PIN),
#if defined(SL_PWM_MOTOR2_CH0_OUTPUT_LOC)
  .location = (uint8_t)(SL_PWM_MOTOR2_CH0_OUTPUT_LOC),
#endif
};

sl_pwm_instance_t sl_pwm_motor2_ch1 = {
  .timer = SL_PWM_MOTOR2_PERIPHERAL,
  .channel = (uint8_t)(SL_PWM_MOTOR2_OUTPUT_CHANNEL1),
  .port = (uint8_t)(SL_PWM_MOTOR2_CH1_OUTPUT_PORT),
  .pin = (uint8_t)(SL_PWM_MOTOR2_CH1_OUTPUT_PIN),
#if defined(SL_PWM_MOTOR2_CH1_OUTPUT_LOC)
  .location = (uint8_t)(SL_PWM_MOTOR2_CH1_OUTPUT_LOC),
#endif
};

sl_pwm_instance_t sl_pwm_motor2_ch2 = {
  .timer = SL_PWM_MOTOR2_PERIPHERAL,
  .channel = (uint8_t)(SL_PWM_MOTOR2_OUTPUT_CHANNEL2),
  .port = (uint8_t)(SL_PWM_MOTOR2_CH2_OUTPUT_PORT),
  .pin = (uint8_t)(SL_PWM_MOTOR2_CH2_OUTPUT_PIN),
#if defined(SL_PWM_MOTOR2_CH2_OUTPUT_LOC)
  .location = (uint8_t)(SL_PWM_MOTOR2_CH2_OUTPUT_LOC),
#endif
};


void sl_pwm_init_instances(void)
{

  sl_pwm_config_t pwm_motor1_config = {
    .frequency = SL_PWM_MOTOR1_FREQUENCY,
    .polarity = SL_PWM_MOTOR1_POLARITY,
  };

  sl_pwm_init(&sl_pwm_motor1_ch0, &pwm_motor1_config);
  sl_pwm_init(&sl_pwm_motor1_ch1, &pwm_motor1_config);
  sl_pwm_init(&sl_pwm_motor1_ch2, &pwm_motor1_config);

  sl_pwm_config_t pwm_motor2_config = {
    .frequency = SL_PWM_MOTOR2_FREQUENCY,
    .polarity = SL_PWM_MOTOR2_POLARITY,
  };

  sl_pwm_init(&sl_pwm_motor2_ch0, &pwm_motor2_config);
  sl_pwm_init(&sl_pwm_motor2_ch1, &pwm_motor2_config);
  sl_pwm_init(&sl_pwm_motor2_ch2, &pwm_motor2_config);

}
