#ifndef SRC_CALLBACKS_H_
#define SRC_CALLBACKS_H_

#include "generalDefines.h"
#include "rtcdriver.h"
#include <stddef.h>
#include "em_gpio.h"
#include "motorControlStateMachine.h"
#include "sl_emlib_gpio_init_PA6_config.h"
#include "sl_emlib_gpio_init_PA7_config.h"
#include "sl_emlib_gpio_init_PA8_config.h"
#include "sl_emlib_gpio_init_PA9_config.h"
#include "sl_emlib_gpio_init_PB6_config.h"
#include "sl_emlib_gpio_init_PB7_config.h"
#include "sl_emlib_gpio_init_PB8_config.h"
#include "sl_emlib_gpio_init_PB9_config.h"
#include "sl_emlib_gpio_init_PC11_config.h"
#include "sl_emlib_gpio_init_PC4_config.h"
#include "sl_emlib_gpio_init_PC5_config.h"
#include "sl_emlib_gpio_init_PD8_config.h"

typedef void(*timedCallbackFctPtr)(RTCDRV_TimerID_t, void *);
typedef void(*intCallbackFctPtr)(uint8_t);

typedef struct{
  timedCallbackFctPtr func;
  uint32_t millis;
  RTCDRV_TimerID_t id;
}STimedCallBacks;


typedef enum{
  motorHandle = 0,
  endOfTimedCallbacksFuncList,
}ETimedCallBacksdFunctions;


typedef enum{
  motor1U = 0,
  motor1V,
  motor1W,
  motor2U,
  motor2V,
  motor2W,
  endOfIntCallbacksFuncList
}EIntCallBacksdFunctions;


typedef struct{
  intCallbackFctPtr func;
  uint8_t pin;
  uint8_t port;
}SIntCallBacks;


// function protos
void callback_motor_handle(RTCDRV_TimerID_t id, void * user);
void init_callbacks_timed(void);
void setTimedCallBacksDB(void);
void callback_D8(uint8_t intNo);
void callback_A6(uint8_t intNo);
void callback_A7(uint8_t intNo);
void callback_B7(uint8_t intNo);
void callback_C11(uint8_t intNo);
void callback_D8(uint8_t intNo);
void callback_B8(uint8_t intNo);

#endif /* SRC_CALLBACKS_H_ */
