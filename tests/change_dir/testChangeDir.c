#include "testChangeDir.h"
static STimedCallBacks testTimedCallBacksDB;

extern SMotorsData motors[NUM_OF_MOTORS];

void setTestTimedCallBacksDB(void)
{
  testTimedCallBacksDB.func = testCallbackChangeDir;
  testTimedCallBacksDB.millis = 1.0/CHANGE_DIR_HZ*1000.0;
  testTimedCallBacksDB.id = changeDir;

}

void init_test_callbacks_timed(void)
{
  setTestTimedCallBacksDB();
      // Reserve a timer.
  RTCDRV_AllocateTimer( &testTimedCallBacksDB.id );
  RTCDRV_StartTimer( testTimedCallBacksDB.id, rtcdrvTimerTypeOneshot, testTimedCallBacksDB.millis, testTimedCallBacksDB.func, NULL );
}

void testCallbackChangeDir(RTCDRV_TimerID_t id, void * user)
{
 (void) user;
  motors[left].speedControler.refSpeed *= -1;
  RTCDRV_StartTimer( id, rtcdrvTimerTypeOneshot, testTimedCallBacksDB.millis, testTimedCallBacksDB.func, NULL );
}
