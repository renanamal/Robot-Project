#include "callBacks.h"


static STimedCallBacks timedCallBacksDB[endOfTimedCallbacksFuncList];
static SIntCallBacks  intCallBacksDB[endOfIntCallbacksFuncList];

// An array to track if given pin callback was called
volatile uint8_t pinInt[32];

void setTimedCallBacksDB(void)
{
  timedCallBacksDB[0].func = callback_motor_handle;
  timedCallBacksDB[0].millis = 1.0/MOTOR_SPEED_CONTROLLER_HZ*1000.0;
  timedCallBacksDB[0].id = motorHandle;
}

void setIntCallBacksDB(void)
{
  intCallBacksDB[motor1U].func = callback_D8;
  intCallBacksDB[motor1U].pin = SL_EMLIB_GPIO_INIT_PD8_PIN;
  intCallBacksDB[motor1U].port = SL_EMLIB_GPIO_INIT_PD8_PORT;

  intCallBacksDB[motor1V].func = callback_A6;
  intCallBacksDB[motor1V].pin = SL_EMLIB_GPIO_INIT_PA6_PIN;
  intCallBacksDB[motor1V].port = SL_EMLIB_GPIO_INIT_PA6_PORT;

  intCallBacksDB[motor1W].func = callback_A7;
  intCallBacksDB[motor1W].pin = SL_EMLIB_GPIO_INIT_PA7_PIN;
  intCallBacksDB[motor1W].port = SL_EMLIB_GPIO_INIT_PA7_PORT;

  intCallBacksDB[motor2U].func = callback_B7;
  intCallBacksDB[motor2U].pin = SL_EMLIB_GPIO_INIT_PB7_PIN;
  intCallBacksDB[motor2U].port = SL_EMLIB_GPIO_INIT_PB7_PORT;

  intCallBacksDB[motor2V].func = callback_C11;
  intCallBacksDB[motor2V].pin = SL_EMLIB_GPIO_INIT_PC11_PIN;
  intCallBacksDB[motor2V].port = SL_EMLIB_GPIO_INIT_PC11_PORT;

  intCallBacksDB[motor2W].func = callback_B8;
  intCallBacksDB[motor2W].pin = SL_EMLIB_GPIO_INIT_PB8_PIN;
  intCallBacksDB[motor2W].port = SL_EMLIB_GPIO_INIT_PB8_PORT;
}

void init_callbacks_timed(void)
{
  for(ETimedCallBacksdFunctions fun = motorHandle; fun < endOfTimedCallbacksFuncList; fun++)
  {
      // Reserve a timer.
      RTCDRV_AllocateTimer( &timedCallBacksDB[fun].id );
      RTCDRV_StartTimer( timedCallBacksDB[fun].id, rtcdrvTimerTypeOneshot, timedCallBacksDB[fun].millis, timedCallBacksDB[fun].func, NULL );
  }
}


void init_callbacks_GPIO(void)
{
  for(EIntCallBacksdFunctions place = motor1U; place < endOfIntCallbacksFuncList; place++)
  {
      GPIOINT_CallbackRegister(intCallBacksDB[place].pin, intCallBacksDB[place].func);
      GPIO_IntConfig(intCallBacksDB[place].port, intCallBacksDB[place].pin, true, true, true);
  }
}

// definition of the Callback functions
void callback_motor_handle(RTCDRV_TimerID_t id, void * user)
{
  (void) user; // unused argument in this example
  RTCDRV_StartTimer( id, rtcdrvTimerTypeOneshot, timedCallBacksDB[motorHandle].millis, timedCallBacksDB[motorHandle].func, NULL );
  handleMotors();
}


void callback_D8(uint8_t intNo)
{
  if (GPIO_PinInGet(GPIO_IRQ_INPUT_PORT, GPIO_IRQ_INPUT_PIN))
  {
    GPIO_PinOutClear(GPIO_LED0_PORT, GPIO_LED0_PIN);
  }
  else
  {
    GPIO_PinOutSet(GPIO_LED0_PORT, GPIO_LED0_PIN);
  }
}

void callback_A6(uint8_t intNo)
{

}

void callback_A7(uint8_t intNo)
{

}

void callback_B7(uint8_t intNo)
{

}

void callback_C11(uint8_t intNo)
{

}

void callback_D8(uint8_t intNo)
{

}

void callback_B8(uint8_t intNo)
{

}
