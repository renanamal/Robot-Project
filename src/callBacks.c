#include "callBacks.h"

uint8_t prevHullSequence[NUM_OF_MOTORS];

// the correct hull sequence Hall State (Hall a, Hall b, Hall c)
//4 (100) 00  10  01
//6 (110) 01  10  00
//2 (010) 01  00  10
//3 (011) 00  01  10
//1 (001) 10  01  00
//5 (101) 10  00  01
// using this sequence we create a look up table:
int8_t hallIrqCntAdevanceMatrix[6][6] = { {0,0,-1,0,1,0},\
                                          {0,0,1,0,0,-1},\
                                          {1,-1,0,0,0,0},\
                                          {0,0,0,0,-1,1},\
                                          {-1,0,0,1,0,0},\
                                          {0,1,0,-1,0,0}  };

static STimedCallBacks timedCallBacksDB[endOfTimedCallbacksFuncList];
static SIntCallBacks  intCallBacksDB[endOfIntCallbacksFuncList];

extern SMotorsData motors[NUM_OF_MOTORS];

void setTimedCallBacksDB(void)
{
  timedCallBacksDB[0].func = callback_motor_handle;
  timedCallBacksDB[0].millis = 1.0/MOTOR_SPEED_CONTROLLER_HZ*1000.0;
  timedCallBacksDB[0].id = motorHandle;
}

void setIntCallBacksDB(void)
{
  intCallBacksDB[motor1U].func = callback_motor1U;
  intCallBacksDB[motor1U].pin = SL_EMLIB_GPIO_INIT_PD8_PIN;
  intCallBacksDB[motor1U].port = SL_EMLIB_GPIO_INIT_PD8_PORT;

  intCallBacksDB[motor1V].func = callback_motor1V;
  intCallBacksDB[motor1V].pin = SL_EMLIB_GPIO_INIT_PA6_PIN;
  intCallBacksDB[motor1V].port = SL_EMLIB_GPIO_INIT_PA6_PORT;

  intCallBacksDB[motor1W].func = callback_motor1W;
  intCallBacksDB[motor1W].pin = SL_EMLIB_GPIO_INIT_PA7_PIN;
  intCallBacksDB[motor1W].port = SL_EMLIB_GPIO_INIT_PA7_PORT;

  intCallBacksDB[motor2U].func = callback_motor2U;
  intCallBacksDB[motor2U].pin = SL_EMLIB_GPIO_INIT_PB7_PIN;
  intCallBacksDB[motor2U].port = SL_EMLIB_GPIO_INIT_PB7_PORT;

  intCallBacksDB[motor2V].func = callback_motor2V;
  intCallBacksDB[motor2V].pin = SL_EMLIB_GPIO_INIT_PC11_PIN;
  intCallBacksDB[motor2V].port = SL_EMLIB_GPIO_INIT_PC11_PORT;

  intCallBacksDB[motor2W].func = callback_motor2W;
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


void callback_motor1U(uint8_t intNo)
{
  (void) intNo; // not in use
  if (GPIO_PinInGet(intCallBacksDB[motor1U].port, intCallBacksDB[motor1U].pin))
  {
      hullHandle(left);
  }
}


void callback_motor1V(uint8_t intNo)
{
  (void) intNo; // not in use
  if (GPIO_PinInGet(intCallBacksDB[motor1V].port, intCallBacksDB[motor1V].pin))
  {
      hullHandle(left);
  }
}


void callback_motor1W(uint8_t intNo)
{
  (void) intNo; // not in use
  if (GPIO_PinInGet(intCallBacksDB[motor1W].port, intCallBacksDB[motor1W].pin))
  {
      hullHandle(left);
  }
}


void callback_motor2U(uint8_t intNo)
{
  (void) intNo; // not in use
  if (GPIO_PinInGet(intCallBacksDB[motor2U].port, intCallBacksDB[motor2U].pin))
  {
      hullHandle(right);
  }
}


void callback_motor2V(uint8_t intNo)
{
  (void) intNo; // not in use
  if (GPIO_PinInGet(intCallBacksDB[motor2V].port, intCallBacksDB[motor2V].pin))
  {
      hullHandle(right);
  }
}


void callback_motor2W(uint8_t intNo)
{
  (void) intNo; // not in use
  if (GPIO_PinInGet(intCallBacksDB[motor2W].port, intCallBacksDB[motor2W].pin))
  {
      hullHandle(right);
  }
}

void hullHandle(EMotor motor)
{
  int8_t hullAdd;
  getMotorComutation(motor);
  getHallSequence(motor);
  if(motors[motor].hull.currentSequence > 5) // not a legal  sequence
  {
      ERROR_BREAK
  }

  hullAdd =  hallIrqCntAdevanceMatrix[prevHullSequence[motor]][motors[motor].hull.currentSequence];

  if(motors[motor].hull.prevHullAdded != hullAdd)
  {
    motors[motor].hull.prevHullAdded = hullAdd;
    hullAdd = 0;
  }
  else
  {
    motors[motor].hull.cnt_last_time_millis = getMillis();
    motors[motor].hull.cnt += hullAdd;
    if (hullAdd == 0 && prevHullSequence[motor] != motors[motor].hull.currentSequence)
    {
        ERROR_BREAK
    }
  }
  prevHullSequence[motor] = motors[motor].hull.currentSequence;
  sendCommandToDriver(motor);
}
