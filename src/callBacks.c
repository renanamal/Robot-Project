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
static SIntCallBacks intCallBacksDB[endOfIntCallbacksFuncList];

extern SMotorsData motors[NUM_OF_MOTORS];

void setTimedCallBacksDB(void)
{
  timedCallBacksDB[0].func = callback_motor_handle;
  timedCallBacksDB[0].millis = 1.0/MOTOR_SPEED_CONTROLLER_HZ*1000.0;
  timedCallBacksDB[0].id = motorHandle;
}

void setIntCallBacksDB(void)
{
  intCallBacksDB[motor1U].func = callback_pin8;
  intCallBacksDB[motor1U].pin = SL_EMLIB_GPIO_INIT_PD8_PIN;
  intCallBacksDB[motor1U].port = SL_EMLIB_GPIO_INIT_PD8_PORT;

  intCallBacksDB[motor1V].func = callback_pin6;
  intCallBacksDB[motor1V].pin = SL_EMLIB_GPIO_INIT_PA6_PIN;
  intCallBacksDB[motor1V].port = SL_EMLIB_GPIO_INIT_PA6_PORT;

  intCallBacksDB[motor1W].func = callback_pin7;
  intCallBacksDB[motor1W].pin = SL_EMLIB_GPIO_INIT_PA7_PIN;
  intCallBacksDB[motor1W].port = SL_EMLIB_GPIO_INIT_PA7_PORT;

  intCallBacksDB[motor2U].func = callback_pin7;
  intCallBacksDB[motor2U].pin = SL_EMLIB_GPIO_INIT_PB7_PIN;
  intCallBacksDB[motor2U].port = SL_EMLIB_GPIO_INIT_PB7_PORT;

  intCallBacksDB[motor2V].func = callback_pin11;
  intCallBacksDB[motor2V].pin = SL_EMLIB_GPIO_INIT_PC11_PIN;
  intCallBacksDB[motor2V].port = SL_EMLIB_GPIO_INIT_PC11_PORT;

  intCallBacksDB[motor2W].func = callback_pin8;
  intCallBacksDB[motor2W].pin = SL_EMLIB_GPIO_INIT_PB8_PIN;
  intCallBacksDB[motor2W].port = SL_EMLIB_GPIO_INIT_PB8_PORT;
}

void init_callbacks_timed(void)
{
  for(ETimedCallBacksdFunctions ind = 0; ind < endOfTimedCallbacksFuncList; ind++)
  {
      // Reserve a timer.
      RTCDRV_AllocateTimer( &timedCallBacksDB[ind].id );
      RTCDRV_StartTimer( timedCallBacksDB[ind].id, rtcdrvTimerTypeOneshot, timedCallBacksDB[ind].millis, timedCallBacksDB[ind].func, NULL );
  }
}


void init_callbacks_GPIO(void)
{
  uint8_t pinList[4] = {6, 7, 8, 11};
  GPIOINT_IrqCallbackPtr_t funcs[4] = {callback_pin6, callback_pin7, callback_pin8, callback_pin11};

  GPIOINT_Init();
  for(uint8_t i = 0; i < 4; i++)
  {
      GPIOINT_CallbackRegister(pinList[i], funcs[i]);
  }
  // Initialization of the GPIOINT driver.
//  for(uint8_t ind = 0; ind < endOfIntCallbacksFuncList; ind++)
//  {
//      GPIOINT_CallbackRegister(intCallBacksDB[ind].pin, intCallBacksDB[ind].func);
//      GPIO_IntConfig(intCallBacksDB[ind].port, intCallBacksDB[ind].pin, true, true, true);
//  }
//  GPIOINT_CallbackRegister(intCallBacksDB[0].pin, intCallBacksDB[0].func);

//  GPIO_ExtIntConfig(intCallBacksDB[3].port, intCallBacksDB[3].pin, intCallBacksDB[3].pin, true, true, true);
  GPIO_ExtIntConfig(intCallBacksDB[2].port, intCallBacksDB[2].pin, intCallBacksDB[2].pin, true, true, true);

  uint32_t PinMask = (1 << 6 | 1 << 7 | 1 << 8 | 1 << 11);
  GPIO_IntEnable(PinMask);
}

// definition of the Callback functions
void callback_motor_handle(RTCDRV_TimerID_t id, void * user)
{
  (void) user; // unused argument in this example
  RTCDRV_StartTimer( id, rtcdrvTimerTypeOneshot, timedCallBacksDB[motorHandle].millis, timedCallBacksDB[motorHandle].func, NULL );
  handleMotors();
}


void callback_pin8(uint8_t intNo) // pin D8 or B8
{
  (void) intNo; // not in use
  if (GPIO_PinInGet(intCallBacksDB[motor1U].port, intCallBacksDB[motor1U].pin))
  {
      hullHandle(left);
  }
  if (GPIO_PinInGet(intCallBacksDB[motor2W].port, intCallBacksDB[motor2W].pin))
  {
      hullHandle(right);
  }
}


void callback_pin6(uint8_t intNo) // pin A6
{
  (void) intNo; // not in use
  if (GPIO_PinInGet(intCallBacksDB[motor1V].port, intCallBacksDB[motor1V].pin))
  {
      hullHandle(left);
  }
}


void callback_pin7(uint8_t intNo) // pin A7 or B7
{
  (void) intNo; // not in use
  if (GPIO_PinInGet(intCallBacksDB[motor1W].port, intCallBacksDB[motor1W].pin))
  {
    hullHandle(left);
    hullHandle(right);
    uint32_t pinmask = (1<<7);
    GPIO_IntClear(pinmask);
  }
  if (GPIO_PinInGet(intCallBacksDB[motor2U].port, intCallBacksDB[motor2U].pin))
  {
  }
}


void callback_pin11(uint8_t intNo) // pin C11
{
  (void) intNo; // not in use
  if (GPIO_PinInGet(intCallBacksDB[motor2V].port, intCallBacksDB[motor2V].pin))
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
