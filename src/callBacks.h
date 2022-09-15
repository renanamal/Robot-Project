#ifndef SRC_CALLBACKS_H_
#define SRC_CALLBACKS_H_

#include "generalDefines.h"
#include "rtcdriver.h"
#include <stddef.h>
#include "em_gpio.h"
#include "motorControlStateMachine.h"
#include "gpiointerrupt.h"
#include "sl_emlib_gpio_init_F13_config.h"
#include "sl_emlib_gpio_init_F14_config.h"
#include "sl_emlib_gpio_init_F15_config.h"
#include "sl_emlib_gpio_init_I1_config.h"
#include "sl_emlib_gpio_init_I2_config.h"
#include "sl_emlib_gpio_init_I3_config.h"
#include "sl_emlib_gpio_init_PA6_config.h"
#include "sl_emlib_gpio_init_PA7_config.h"
#include "sl_emlib_gpio_init_PB9_config.h"
#include "sl_emlib_gpio_init_PC4_config.h"
#include "sl_emlib_gpio_init_PC5_config.h"
#include "sl_emlib_gpio_init_PD8_config.h"
#include "debugFunctions.h"

typedef void(*timedCallbackFctPtr)(RTCDRV_TimerID_t, void *);

typedef struct{
  timedCallbackFctPtr func;
  uint32_t millis;
  RTCDRV_TimerID_t id;
}STimedCallBacks;


typedef enum{
  motorHandle = 0,
//  changeDir,
  endOfTimedCallbacksFuncList,
}ETimedCallBacksdFunctions;


typedef enum{
  motor1U = 0,
  motor1V,
  motor1W,
  motor2U,
  motor2V,
  motor2W,
  motor1EncA,
  motor1EncB,
  motor1EncI,
  motor2EncA,
  motor2EncB,
  motor2EncI,
  endOfIntCallbacksFuncList
}EIntCallBacksdFunctions;


typedef struct{
  GPIOINT_IrqCallbackPtr_t func;
  uint8_t pin;
  GPIO_Port_TypeDef port;
}SIntCallBacks;

// function protos
void callback_motor_handle(RTCDRV_TimerID_t id, void * user);
//void callback_motor_handle(void);

void setTimedCallBacksDB(void);
void init_callbacks_timed(void);

void setIntCallBacksDB(void);
void init_callbacks_GPIO(void);

void encoderHandle(EMotor motor);
void hullHandle(EMotor motor);

void callback_change_dir(RTCDRV_TimerID_t id, void * user);

void callback_pin1(uint8_t intNo);
void callback_pin2(uint8_t intNo);
void callback_pin3(uint8_t intNo);
void callback_pin4(uint8_t intNo);
void callback_pin5(uint8_t intNo);
void callback_pin6(uint8_t intNo);
void callback_pin7(uint8_t intNo);
void callback_pin8(uint8_t intNo);
void callback_pin9(uint8_t intNo);
void callback_pin13(uint8_t intNo);
void callback_pin14(uint8_t intNo);
void callback_pin15(uint8_t intNo);

#endif /* SRC_CALLBACKS_H_ */
