#include "debugFunctions.h"
#include "motorDriverMain.h"

extern SMotorsData motors[NUM_OF_MOTORS];

SDBGMainDBG mainDBG;


void record_motor_data(EMotor motor)
{
  // ========================================================== DEBUG - START ========================================================================
  mainDBG.speedControl[motor].counter++;
  uint32_t ind = mainDBG.speedControl[motor].counter;
  mainDBG.speedControl[motor].PI_correction[ind] = motors[motor].speedControler.speed_I_correction;
  mainDBG.speedControl[motor].PWMpercent[ind] =  motors[motor].PWMCommand;
  mainDBG.speedControl[motor].speedFromHull[ind] = motors[motor].speedControler.speedFromHull;
  mainDBG.speedControl[motor].speedAverage[ind] = motors[motor].speedControler.speedAverage.AverageData;
  mainDBG.speedControl[motor].correcteddSpeed[ind] = motors[motor].speedControler.speedCorrected;
  if (mainDBG.speedControl[motor].counter > (speedControlDbgArraySize - 1))
  {
    DEBUG_BREAK;
    while (1);
  }
  // ========================================================== DEBUG - END ==========================================================================
}

void record_SpeedError(EMotor motor, float SpeedError)
{
  mainDBG.speedControl[motor].speedError[mainDBG.speedControl[motor].counter] = SpeedError;
}

void record_Pcorrection(EMotor motor, float Pcorrection)
{
  mainDBG.speedControl[motor].Pcorrection[mainDBG.speedControl[motor].counter] = Pcorrection;
}

void record_speed_I_correction(EMotor motor, float speed_I_correction)
{
  mainDBG.speedControl[motor].speed_I_correction[mainDBG.speedControl[motor].counter] = speed_I_correction;
}

void record_refMotorSpeedRPM(EMotor motor, float refMotorSpeedRPM)
{
  mainDBG.speedControl[motor].refMotorSpeedRPM[mainDBG.speedControl[motor].counter] = refMotorSpeedRPM;
}

void record_commandVrms(EMotor motor, float commandVrms)
{
  mainDBG.speedControl[motor].commandVrms[mainDBG.speedControl[motor].counter] = commandVrms;
}


void record_hull(EMotor motor)
{
  uint8_t ind = mainDBG.hullCounter[motor].counter;
  mainDBG.hullCounter[motor].hull[ind] = motors[motor].hull;
  mainDBG.hullCounter[motor].counter++;
  if (mainDBG.hullCounter[motor].counter > (hullCounterDbgArraySize - 1))
  {
    DEBUG_BREAK;
    while (1);
  }
}
