#include "testChangeDir.h"
static STimedCallBacks testTimedCallBacksDB;

extern SMotorsData motors[NUM_OF_MOTORS];

static uint8_t counter = 0;
float speedTable[QUATER_SPEED_STEPS * 4] = {0};

void testInitSpeedTable()
{

  for(int i=1; i<=QUATER_SPEED_STEPS; i++)
  {
      speedTable[i] = speedTable[i-1] + 20;
  }
  for(int i=QUATER_SPEED_STEPS+1; i<=(QUATER_SPEED_STEPS*3); i++)
  {
      speedTable[i] = speedTable[i-1] - 20;
  }
  for(int i=QUATER_SPEED_STEPS*3+1; i<QUATER_SPEED_STEPS*4; i++)
  {
      speedTable[i] = speedTable[i-1] + 20;
  }
}

void setTestTimedCallBacksDB(void)
{
  testTimedCallBacksDB.func = testCallbackChangeDir;
  testTimedCallBacksDB.millis = 1.0/((float)CHANGE_DIR_HZ)*1000.0;
  testTimedCallBacksDB.id = 1;

}

void init_test_callbacks_timed(void)
{
  setTestTimedCallBacksDB();
  testInitSpeedTable();
  // Reserve a timer.
  RTCDRV_AllocateTimer( &testTimedCallBacksDB.id );
  RTCDRV_StartTimer( testTimedCallBacksDB.id, rtcdrvTimerTypeOneshot, testTimedCallBacksDB.millis, testTimedCallBacksDB.func, NULL );
}

void testCallbackChangeDir(RTCDRV_TimerID_t id, void * user)
{
 (void) user;
  motors[left].speedControler.refSpeed = speedTable[counter];
  counter = (counter+1)%(QUATER_SPEED_STEPS*4);
  RTCDRV_StartTimer( id, rtcdrvTimerTypeOneshot, testTimedCallBacksDB.millis, testTimedCallBacksDB.func, NULL );
}
