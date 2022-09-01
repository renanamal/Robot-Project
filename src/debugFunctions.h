#ifndef SRC_DEBUGFUNCTIONS_H_
#define SRC_DEBUGFUNCTIONS_H_

#include "generalDefines.h"
#include "motorDriverMain.h"
#include "stdint.h"

#define speedControlDbgArraySize 20

typedef struct
{
  float speedError[speedControlDbgArraySize];
  float Pcorrection[speedControlDbgArraySize];
  float speed_I_correction[speedControlDbgArraySize];
  float speedFromHull[speedControlDbgArraySize];
  float speedAverage[speedControlDbgArraySize];
  float refMotorSpeedRPM[speedControlDbgArraySize];
  float commandVrms[speedControlDbgArraySize];
  uint8_t PWMpercent[speedControlDbgArraySize];
  int counter;
  float PI_correction[speedControlDbgArraySize];
  float correcteddSpeed[speedControlDbgArraySize];
}SDBGSpeedControl;

#define hullCounterDbgArraySize 21

typedef struct
{
  SGDComutation hull[hullCounterDbgArraySize];
  int counter;
}SDBGHallCounter;

typedef struct
{
  SDBGSpeedControl speedControl[NUM_OF_MOTORS];
  SDBGHallCounter  hullCounter[NUM_OF_MOTORS];
}SDBGMainDBG;


// function proto
void record_motor_data(EMotor motor);
void record_hull(EMotor motor);
void record_SpeedError(EMotor motor, float SpeedError);
void record_Pcorrection(EMotor motor, float Pcorrection);
void record_speed_I_correction(EMotor motor, float speed_I_correction);
void record_refMotorSpeedRPM(EMotor motor, float refMotorSpeedRPM);
void record_commandVrms(EMotor motor, float commandVrms);

#endif /* SRC_DEBUGFUNCTIONS_H_ */
