#include "sl_pwm_instances.h"

void test_pwm()
{
  // Set duty cycle to 40%
  sl_pwm_set_duty_cycle(&sl_pwm_motor1_ch2, 40);

  // Enable PWM output
  sl_pwm_start(&sl_pwm_motor1_ch2);

}
