#ifndef TESTS_CHANGE_DIR_TESTCHANGEDIR_H_
#define TESTS_CHANGE_DIR_TESTCHANGEDIR_H_

#include "callBacks.h"
#define QUATER_SPEED_STEPS 5
#define CHANGE_DIR_HZ (30 * QUATER_SPEED_STEPS * 4)

void init_test_callbacks_timed(void);
void setTestTimedCallBacksDB(void);
void testCallbackChangeDir(RTCDRV_TimerID_t id, void * user);
#endif /* TESTS_CHANGE_DIR_TESTCHANGEDIR_H_ */
