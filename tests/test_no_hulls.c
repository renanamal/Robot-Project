#include "test_no_hulls.h"

extern SMotorsData motors[NUM_OF_MOTORS];
extern e_motorControlStates motorControlState[NUM_OF_MOTORS];
extern int8_t hallIrqCntAdevanceMatrix[6][6];

uint32_t millis = 10;
RTCDRV_TimerID_t id = 1;
EMotor motor = left;

void runMotorNoHulls(EMotor motor)
{
  motors[motor].speedControler.refSpeed = 1.0; // 3 [rad/sec]
  getMotorHulls(motor);
//  motors[motor].hull.HullU = 0;
//  motors[motor].hull.HullV = 1;
//  motors[motor].hull.HullW = 0;
  RTCDRV_AllocateTimer( &id );
  RTCDRV_StartTimer( id, rtcdrvTimerTypeOneshot, millis, doStep, NULL );
}

void doStep(RTCDRV_TimerID_t id, void * user)
{
  (void) user;
  motorPhaseConfigurationHandle(motor);
  getHullSequence(motor);
  motors[motor].hull.hullAdd =  hallIrqCntAdevanceMatrix[motors[motor].hull.prevSequence][motors[motor].hull.currentSequence];
  record_hull(motor);
  motors[motor].hull.prevSequence = motors[motor].hull.currentSequence;
  calcHullAdd(motor);
  motors[motor].hull.cnt_last_time_uSec = getuSec();
  motors[motor].hull.cnt += motors[motor].hull.hullAdd;
  sendCommandToDriver(motor);
//  getHullsDBG(motor);
//  calcMotorSpeedDBG(motor);
  setNextHullSequence(motor);
  RTCDRV_StartTimer( id, rtcdrvTimerTypeOneshot, millis, doStep, NULL );
}

//void calcMotorSpeedDBG(EMotor motor)
//{
//  motorControlState[motor] = MCS_START_RUNING;
//  calcSpeedFromHulls(motor);
//  continuousAverage(&motors[motor].speedControler.speedAverage);
//  setMotorDriveState(motor);
//  motorControlSatetExct(motor);
//  calcPWMpercent(motor);
//}

void setNextHullSequence(EMotor motor)
{
  // the correct hull sequence Hall State (Hall a=U, Hall b=V, Hall c=W)
  //4 (100) 00  10  01      ind 3
  //6 (110) 01  10  00      ind 5
  //2 (010) 01  00  10      ind 1
  //3 (011) 00  01  10      ind 2
  //1 (001) 10  01  00      ind 0
  //5 (101) 10  00  01      ind 4
//  static uint8_t nextSequence[6][3] = {{0,0,1}, {0,1,0}, {0,1,1}, {1,0,0}, {1,0,1}, {1,1,0}};
  static uint8_t nextSequence[6][3] = {{1,0,1}, {0,1,1}, {0,0,1}, {1,1,0}, {1,0,0}, {0,1,0}};
  volatile uint8_t sequence = ((motors[motor].hull.HullU << 2 | motors[motor].hull.HullV << 1 | motors[motor].hull.HullW) & 0x7) - 1;
  motors[motor].hull.HullU = nextSequence[sequence][0];
  motors[motor].hull.HullV = nextSequence[sequence][1];
  motors[motor].hull.HullW = nextSequence[sequence][2];
}
