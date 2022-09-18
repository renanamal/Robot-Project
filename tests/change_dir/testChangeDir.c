#include "testChangeDir.h"
static STimedCallBacks testTimedCallBacksDB;

//extern SMotorsData motors[NUM_OF_MOTORS];

//static uint8_t counter = 0;
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
  testTimedCallBacksDB.func = handleMotors;
  testTimedCallBacksDB.us = CALLBACK_uS(CHANGE_DIR_HZ);
  testTimedCallBacksDB.prevTimeCall = getuSec();
}

void init_test_callbacks_timed()
{
  testInitSpeedTable();
  setTestTimedCallBacksDB();
}

uint64_t times[200];
uint64_t deltaTimes[200];
uint8_t counter = 0;
void executeTimedFunctionsTest(void)
{
    uint64_t currentTime = getuSec();
    if(currentTime - testTimedCallBacksDB.prevTimeCall >= testTimedCallBacksDB.us)
    {
        times[counter] = currentTime;
        counter++;
        testTimedCallBacksDB.prevTimeCall = currentTime;
        (*testTimedCallBacksDB.func)();
    }
    if(counter >= 200)
    {
        for(int i=1; i<200; i++)
        {
            deltaTimes[i] = times[i] - times[i-1];
        }
      DEBUG_BREAK
      while(1){}
    }
}
