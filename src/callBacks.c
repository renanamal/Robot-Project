#include "callBacks.h"

#ifdef DEBUG_HULLS
  #include "debugFunctions.h"
#endif

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
  // ====================== Hull motor 1 =================================
  intCallBacksDB[motor1U].func = callback_pin8;
  intCallBacksDB[motor1U].pin = SL_EMLIB_GPIO_INIT_PD8_PIN;
  intCallBacksDB[motor1U].port = SL_EMLIB_GPIO_INIT_PD8_PORT;

  intCallBacksDB[motor1V].func = callback_pin6;
  intCallBacksDB[motor1V].pin = SL_EMLIB_GPIO_INIT_PA6_PIN;
  intCallBacksDB[motor1V].port = SL_EMLIB_GPIO_INIT_PA6_PORT;

  intCallBacksDB[motor1W].func = callback_pin7;
  intCallBacksDB[motor1W].pin = SL_EMLIB_GPIO_INIT_PA7_PIN;
  intCallBacksDB[motor1W].port = SL_EMLIB_GPIO_INIT_PA7_PORT;

  // ====================== Hull motor 2 =================================
  intCallBacksDB[motor2U].func = callback_pin13;
  intCallBacksDB[motor2U].pin = SL_EMLIB_GPIO_INIT_F13_PIN;
  intCallBacksDB[motor2U].port = SL_EMLIB_GPIO_INIT_F13_PORT;

  intCallBacksDB[motor2V].func = callback_pin15;
  intCallBacksDB[motor2V].pin = SL_EMLIB_GPIO_INIT_F15_PIN;
  intCallBacksDB[motor2V].port = SL_EMLIB_GPIO_INIT_F15_PORT;

  intCallBacksDB[motor2W].func = callback_pin14;
  intCallBacksDB[motor2W].pin = SL_EMLIB_GPIO_INIT_F14_PIN;
  intCallBacksDB[motor2W].port = SL_EMLIB_GPIO_INIT_F14_PORT;

  // ====================== Encoder motor 1 =================================
  intCallBacksDB[motor1EncA].func = callback_pin1;
  intCallBacksDB[motor1EncA].pin = SL_EMLIB_GPIO_INIT_I1_PIN;
  intCallBacksDB[motor1EncA].port = SL_EMLIB_GPIO_INIT_I1_PORT;

  intCallBacksDB[motor1EncB].func = callback_pin2;
  intCallBacksDB[motor1EncB].pin = SL_EMLIB_GPIO_INIT_I2_PIN;
  intCallBacksDB[motor1EncB].port = SL_EMLIB_GPIO_INIT_I2_PORT;

  intCallBacksDB[motor1EncI].func = callback_pin3;
  intCallBacksDB[motor1EncI].pin = SL_EMLIB_GPIO_INIT_I3_PIN;
  intCallBacksDB[motor1EncI].port = SL_EMLIB_GPIO_INIT_I3_PORT;

  // ====================== Encoder motor 2 =================================
  intCallBacksDB[motor2EncA].func = callback_pin9;
  intCallBacksDB[motor2EncA].pin = SL_EMLIB_GPIO_INIT_PB9_PIN;
  intCallBacksDB[motor2EncA].port = SL_EMLIB_GPIO_INIT_PB9_PORT;

  intCallBacksDB[motor2EncB].func = callback_pin4;
  intCallBacksDB[motor2EncB].pin = SL_EMLIB_GPIO_INIT_PC4_PIN;
  intCallBacksDB[motor2EncB].port = SL_EMLIB_GPIO_INIT_PC4_PORT;

  intCallBacksDB[motor2EncI].func = callback_pin5;
  intCallBacksDB[motor2EncI].pin = SL_EMLIB_GPIO_INIT_PC5_PIN;
  intCallBacksDB[motor2EncI].port = SL_EMLIB_GPIO_INIT_PC5_PORT;
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
//  uint8_t pinList[4] = {6, 7, 8, 11};
//  GPIOINT_IrqCallbackPtr_t funcs[4] = {callback_pin6, callback_pin7, callback_pin8, callback_pin11};

  // Initialization of the GPIOINT driver.
  GPIOINT_Init();

  for(uint8_t ind = 0; ind < endOfIntCallbacksFuncList; ind++)
  {
      GPIOINT_CallbackRegister(intCallBacksDB[ind].pin, intCallBacksDB[ind].func);
//      GPIO_IntConfig(intCallBacksDB[ind].port, intCallBacksDB[ind].pin, true, true, true);
      GPIO_ExtIntConfig(intCallBacksDB[ind].port, intCallBacksDB[ind].pin, intCallBacksDB[ind].pin, true, true, true);
  }
}

// definition of the Callback functions
void callback_motor_handle(RTCDRV_TimerID_t id, void * user)
{
  (void) user; // unused argument in this example
  RTCDRV_StartTimer( id, rtcdrvTimerTypeOneshot, timedCallBacksDB[motorHandle].millis, timedCallBacksDB[motorHandle].func, NULL );
  handleMotors();
}


void callback_pin1(uint8_t intNo) // pin I1
{
  (void) intNo; // not in use
  if (GPIO_PinInGet(intCallBacksDB[motor1EncA].port, intCallBacksDB[motor1EncA].pin))
  {
      encoderHandle(left);
  }
}

void callback_pin2(uint8_t intNo) // pin I2
{
  (void) intNo; // not in use
  if (GPIO_PinInGet(intCallBacksDB[motor1EncB].port, intCallBacksDB[motor1EncB].pin))
  {
      encoderHandle(left);
  }
}

void callback_pin3(uint8_t intNo) // pin I3
{
  (void) intNo; // not in use
  if (GPIO_PinInGet(intCallBacksDB[motor1EncI].port, intCallBacksDB[motor1EncI].pin))
  {
      encoderHandle(left);
  }
}

void callback_pin4(uint8_t intNo) // pin C4
{
  (void) intNo; // not in use
  if (GPIO_PinInGet(intCallBacksDB[motor2EncB].port, intCallBacksDB[motor2EncB].pin))
  {
      encoderHandle(right);
  }
}

void callback_pin5(uint8_t intNo) // pin C5
{
  (void) intNo; // not in use
  if (GPIO_PinInGet(intCallBacksDB[motor2EncI].port, intCallBacksDB[motor2EncI].pin))
  {
      encoderHandle(right);
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

void callback_pin7(uint8_t intNo) // pin A7
{
  (void) intNo; // not in use
  if (GPIO_PinInGet(intCallBacksDB[motor1W].port, intCallBacksDB[motor1W].pin))
  {
    hullHandle(left);
  }

}

void callback_pin8(uint8_t intNo) // pin D8
{
  (void) intNo; // not in use
  if (GPIO_PinInGet(intCallBacksDB[motor1U].port, intCallBacksDB[motor1U].pin))
  {
      hullHandle(left);
  }
}

void callback_pin9(uint8_t intNo) // pin B9
{
  (void) intNo; // not in use
  if (GPIO_PinInGet(intCallBacksDB[motor2EncA].port, intCallBacksDB[motor2EncA].pin))
  {
      encoderHandle(right);
  }
}

void callback_pin13(uint8_t intNo) // pin F13
{
  (void) intNo; // not in use
  if (GPIO_PinInGet(intCallBacksDB[motor2U].port, intCallBacksDB[motor2U].pin))
  {
      hullHandle(right);
  }
}

void callback_pin14(uint8_t intNo) // pin F14
{
  (void) intNo; // not in use
  if (GPIO_PinInGet(intCallBacksDB[motor2W].port, intCallBacksDB[motor2W].pin))
  {
      hullHandle(right);
  }
}

void callback_pin15(uint8_t intNo) // pin F15
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
  motorPhaseConfigurationHandle(motor);
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
#ifdef DEBUG_HULLS
  record_hull(motor, hullAdd);
#endif
}

void encoderHandle(EMotor motor)
{
  return;
}
