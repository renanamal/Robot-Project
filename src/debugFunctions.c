#include "debugFunctions.h"
#include "motorDriverMain.h"

extern SMotorsData motors[NUM_OF_MOTORS];
extern uint8_t prevHullSequence[NUM_OF_MOTORS];

SDBGMainDBG mainDBG;


void record_speed_control(EMotor motor)
{
  // ========================================================== DEBUG - START ========================================================================
  mainDBG.speedControl[motor].counter++;
  mainDBG.speedControl[motor].PI_correction[mainDBG.speedControl[motor].counter] = motors[motor].speedControler.speed_I_correction;
  mainDBG.speedControl[motor].PWMpercent[mainDBG.speedControl[motor].counter] =  motors[motor].PWMCommand;
  mainDBG.speedControl[motor].speedFromHull[mainDBG.speedControl[motor].counter] = motors[motor].speedControler.speedFromHull;
  if (mainDBG.speedControl[motor].counter > (speedControlDbgArraySize - 1))
  {
    DEBUG_BREAK;
    while (1);
  }
  // ========================================================== DEBUG - END ==========================================================================
}


void record_hull(EMotor motor, int8_t hullAdd)
{
  mainDBG.hullCounter[motor].prevHullSequence[mainDBG.hullCounter[motor].counter] = prevHullSequence[motor];
  mainDBG.hullCounter[motor].hullIrqCntAdevanceMatrix[mainDBG.hullCounter[motor].counter] = hullAdd;
  mainDBG.hullCounter[motor].counter++;
  if (mainDBG.hullCounter[motor].counter > (hullCounterDbgArraySize - 1))
  {
    DEBUG_BREAK;
    while (1);
  }
}
