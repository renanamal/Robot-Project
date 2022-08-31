#include "motorDriverMain.h"
#include "motorControlStateMachine.h"

extern SMotorsData motors[NUM_OF_MOTORS];

void read_hulls(void)
{
  EMotor motor = left;
  volatile uint8_t hulls[3];
  getMotorComutation(motor);
  hulls[0] = motors[motor].hull.HullU;
  hulls[1] = motors[motor].hull.HullV;
  hulls[2] = motors[motor].hull.HullW;
  DEBUG_BREAK;
  while (1);
}

void test_motors_handle(void)
{
  volatile EMotor motor = left;
  getMotorComutation(motor);
  getHallSequence(motor);
  speedControlHandle(motor);
  setMotorDriveState(motor);
  calcMotorPWMCommand(motor);
//  sendCommandToDriver(motor);
}
