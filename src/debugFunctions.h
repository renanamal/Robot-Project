#ifndef SRC_DEBUGFUNCTIONS_H_
#define SRC_DEBUGFUNCTIONS_H_

#include "generalDefines.h"
#include "motorDriverMain.h"
#include "stdint.h"

#define speedControlDbgArraySize 200

typedef struct
{
  float speedFromHull[speedControlDbgArraySize];
  uint8_t PWMpercent[speedControlDbgArraySize];
  int counter;
  float PI_correction[speedControlDbgArraySize];
  float correcteddSpeed[speedControlDbgArraySize];
}SDBGSpeedControl;

#define hullCounterDbgArraySize 20

typedef struct
{
  uint8_t prevHullSequence[hullCounterDbgArraySize];
  int8_t hullIrqCntAdevanceMatrix[hullCounterDbgArraySize];
  int counter;
}SDBGHallCounter;

typedef struct
{
  SDBGSpeedControl speedControl[NUM_OF_MOTORS];
  SDBGHallCounter  hullCounter[NUM_OF_MOTORS];
}SDBGMainDBG;


// function proto
void record_speed_control(EMotor motor);
void record_hull(EMotor motor, int8_t hullAdd);

#endif /* SRC_DEBUGFUNCTIONS_H_ */
