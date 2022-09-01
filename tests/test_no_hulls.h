#ifndef TESTS_TEST_NO_HULLS_H_
#define TESTS_TEST_NO_HULLS_H_

#include "motorDriverMain.h"
#include "motorControlStateMachine.h"
#include "rtcdriver.h"
#include <stddef.h>

void runMotorNoHulls(EMotor motor);
void setNextHullSequence(EMotor motor);
void doStep(RTCDRV_TimerID_t id, void * user);

#endif /* TESTS_TEST_NO_HULLS_H_ */
