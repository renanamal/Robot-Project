#ifndef SRC_GENERALDEFINES_H_
#define SRC_GENERALDEFINES_H_


// Units conversion
#define PI (3.1415926535897932384626433832795)
#define RAD_PER_INTERAPT (PI/3.0)
#define PI_ANTIWINDUP (100.0)
#define IS_ZERO_FLOAT(dataIn) ((dataIn < 0.00001 && dataIn > -0.00001)?true:false)
#define RPS_TO_RPM (60.0)

#define MOTOR_SPEED_CNTORLLER_HZ (60.0)

#define NUM_OF_MOTORS (2)


#define ERROR_BREAK           __asm__("BKPT #0");

#ifdef DEBUG_MODE
#define DEBUG_BREAK           ERROR_BREAK;
#else
#define DEBUG_BREAK           __asm__("");
#endif

#endif /* SRC_GENERALDEFINES_H_ */
