#ifndef PWM_INIT_MOTORS_CONFIG_H
#define PWM_INIT_MOTORS_CONFIG_H

#ifdef __cplusplus
extern "C"
{
#endif

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

#define SL_PWM_MOTOR1_OUTPUT_CHANNEL             0
// TIMER0 CC0 on PD9
#define SL_PWM_MOTOR1_OUTPUT_PORT                gpioPortD
#define SL_PWM_MOTOR1_OUTPUT_PIN                 9
#define SL_PWM_MOTOR1_OUTPUT_LOC                 17
// [TIMER_SL_PWM_MOTOR1]$

#define SL_PWM_MOTOR1_OUTPUT_CHANNEL             1
// TIMER0 CC1 on PD10
#define SL_PWM_MOTOR1_OUTPUT_PORT                gpioPortD
#define SL_PWM_MOTOR1_OUTPUT_PIN                 10
#define SL_PWM_MOTOR1_OUTPUT_LOC                 17

#define SL_PWM_MOTOR1_OUTPUT_CHANNEL             2
// TIMER0 CC2 on PD11
#define SL_PWM_MOTOR1_OUTPUT_PORT                gpioPortD
#define SL_PWM_MOTOR1_OUTPUT_PIN                 11
#define SL_PWM_MOTOR1_OUTPUT_LOC                 17
// <<< sl:end pin_tool >>>

// <h>PWM configuration

// <o SL_PWM_MOTOR2_FREQUENCY> PWM frequency [Hz]
// <i> Default: 10000
#define SL_PWM_MOTOR2_FREQUENCY       40000

// <o SL_PWM_MOTOR2_POLARITY> Polarity
// <PWM_ACTIVE_HIGH=> Active high
// <PWM_ACTIVE_LOW=> Active low
// <i> Default: PWM_ACTIVE_HIGH
#define SL_PWM_MOTOR2_POLARITY        PWM_ACTIVE_HIGH

// <timer channel=OUTPUT> SL_PWM_MOTOR2

#ifdef __cplusplus
}
#endif

#endif // PWM_INIT_MOTOR1_CONFIG_H
