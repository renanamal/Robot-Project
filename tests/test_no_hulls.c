#include "test_no_hulls.h"

extern SMotorsData motors[NUM_OF_MOTORS];

void runMotorNoHulls(EMotor motor)
{
  // the correct hull sequence Hall State (Hall a, Hall b, Hall c)
  //4 (100) 00  10  01
  //6 (110) 01  10  00
  //2 (010) 01  00  10
  //3 (011) 00  01  10
  //1 (001) 10  01  00
  //5 (101) 10  00  01
  static uint8_t programdSequence = {4, 6, 2, 3, 1, 5};
  motors[motor].speedControler.refSpeed = 1.0; // 1 [rad/sec]
  getMotorComutation(motor);

  while(1)
  {
    motorPhaseConfigurationHandle(motor);
    getHallSequence(motor);
    speedControlHandle(motor);
    setMotorDriveState(motor);
    calcMotorPWMCommand(motor);
    sendCommandToDriver(motor);
    setNextHullSequence(motor);
  }
}

void setNextHullSequence(EMotor motor)
{
  static uint8_t nextSequence[6][3] = {{0,0,1}, {0,1,0}, {0,1,1}, {1,0,0}, {1,0,1}, {1,1,0}};
  uint8_t CommotationState = (motors[motor].hull.HullU << 2 | motors[motor].hull.HullV << 1 | motors[motor].hull.HullW) & 0x7;
  motors[motor].hull.HullU = nextSequence[CommotationState][0];
  motors[motor].hull.HullV = nextSequence[CommotationState][1];
  motors[motor].hull.HullW = nextSequence[CommotationState][2];
}
