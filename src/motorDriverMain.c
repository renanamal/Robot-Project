#include "motorDriverMain.h"

fctPtr pwmSetDutyCycle[] = {GPIO_motorPinPWMoutLow , GPIO_motorPinPWMoutHigh , GPIO_motorPinPWMoutDisable};
S_fullMotorPhaseConfiguration motorPhaseConfiguration;
uint8_t gCommotationState[NUM_OF_MOTORS];
SMotorsData motors[NUM_OF_MOTORS];

extern sl_pwm_instance_t sl_pwm_motor1_ch0;
extern sl_pwm_instance_t sl_pwm_motor1_ch1;
extern sl_pwm_instance_t sl_pwm_motor1_ch2;
extern sl_pwm_instance_t sl_pwm_motor2_ch0;
extern sl_pwm_instance_t sl_pwm_motor2_ch1;
extern sl_pwm_instance_t sl_pwm_motor2_ch2;
extern int8_t hallIrqCntAdevanceMatrix[6][6];

void sendCommandToDriver(EMotor motor){
  static sl_pwm_instance_t *motor_pwm_ch0;
  static sl_pwm_instance_t *motor_pwm_ch1;
  static sl_pwm_instance_t *motor_pwm_ch2;

  switch(motor){
    case left:
      motor_pwm_ch0 = &sl_pwm_motor1_ch0;
      motor_pwm_ch1 = &sl_pwm_motor1_ch1;
      motor_pwm_ch2 = &sl_pwm_motor1_ch2;
      break;

    case right:
      motor_pwm_ch0 = &sl_pwm_motor2_ch0;
      motor_pwm_ch1 = &sl_pwm_motor2_ch1;
      motor_pwm_ch2 = &sl_pwm_motor2_ch2;
      break;

    default:
      ERROR_BREAK
      return;
  }

	(*pwmSetDutyCycle[motors[motor].commutation.Upolarity])(motor_pwm_ch0, motor);
	(*pwmSetDutyCycle[motors[motor].commutation.Vpolarity])(motor_pwm_ch1, motor);
	(*pwmSetDutyCycle[motors[motor].commutation.Wpolarity])(motor_pwm_ch2, motor);
}


void getAllMotorsCommutation(void)
{
  for(EMotor motor = left; motor < endOfMotors; motor++)
  {
      getMotorHulls(motor);
  }
}


void getMotorHulls(EMotor motor){
  switch(motor)
  {
    case left:
      motors[motor].hull.HullU = GPIO_PinInGet(SL_EMLIB_GPIO_INIT_PD8_PORT, SL_EMLIB_GPIO_INIT_PD8_PIN);
      motors[motor].hull.HullV = GPIO_PinInGet(SL_EMLIB_GPIO_INIT_PA6_PORT, SL_EMLIB_GPIO_INIT_PA6_PIN);
      motors[motor].hull.HullW = GPIO_PinInGet(SL_EMLIB_GPIO_INIT_PA7_PORT, SL_EMLIB_GPIO_INIT_PA7_PIN);
      break;

    case right:
      motors[motor].hull.HullV = GPIO_PinInGet(SL_EMLIB_GPIO_INIT_F15_PORT, SL_EMLIB_GPIO_INIT_F15_PIN);
      motors[motor].hull.HullW = GPIO_PinInGet(SL_EMLIB_GPIO_INIT_F14_PORT, SL_EMLIB_GPIO_INIT_F14_PIN);
      motors[motor].hull.HullU = GPIO_PinInGet(SL_EMLIB_GPIO_INIT_F13_PORT, SL_EMLIB_GPIO_INIT_F13_PIN);
      break;

    default:
      ERROR_BREAK
  }
  return;
}

void motorPhaseConfigurationHandle(EMotor motor)
{
  gCommotationState[motor] = (motors[motor].hull.HullU << 2 | motors[motor].hull.HullV << 1 | motors[motor].hull.HullW) & 0x7;

#ifdef DEBUG_SPEED_CONTROL
  record_gCommotationState(motor, gCommotationState[motor]);
#endif


  if ((motors[motor].motorDriveState == DS_CW) || (motors[motor].motorDriveState == DS_STOP))
  {
    motors[motor].commutation = motorPhaseConfiguration.forword[gCommotationState[motor]];
  }
  else if (motors[motor].motorDriveState == DS_CCW)
  {
    motors[motor].commutation = motorPhaseConfiguration.backword[gCommotationState[motor]];
  }
  return;
}

// ==================================== PI speed control algorithm - START ===================================
float PISpeedControl(EMotor motor)
{
// TODO - enable if each motor has different parrams
//  static float ki;
//  static float kp;
//  switch(motor){
//    case left:
//      ki = LEFT_KI;
//      kp = LEFT_KP;
//      break;
//
//    case right:
//      ki = RIGHT_KI;
//      kp = RIGHT_KP;
//      break;
//  }

	float SpeedError = motors[motor].speedControler.refSpeed - motors[motor].speedControler.speedAverage.AverageData; // TODO - speedAverage or speedFromHall

	float Pcorrection = KP * SpeedError;
	float speed_I_correction = KI * SpeedError + motors[motor].speedControler.speed_I_correction;


	if (speed_I_correction > PI_ANTIWINDUP) // unti-windup
	{
		speed_I_correction = PI_ANTIWINDUP;
	}

	if (speed_I_correction < -1 * PI_ANTIWINDUP) // unti-windup
	{
		speed_I_correction = -1 * PI_ANTIWINDUP;
	}
	motors[motor].speedControler.speed_I_correction = speed_I_correction;

#ifdef DEBUG_SPEED_CONTROL
	record_SpeedError(motor, SpeedError);
	record_Pcorrection(motor, Pcorrection);
	record_speed_I_correction(motor, speed_I_correction);
#endif

	return Pcorrection + speed_I_correction;
}
// ==================================== PI linear speed control algorithm - START ===================================


// ==================================== GPIO motor Pin PWM out Disable - START ===================================
void GPIO_motorPinPWMoutDisable(sl_pwm_instance_t *motor_pwm_ch, EMotor motor)
{
  (void) motor; // unused argument
  sl_pwm_set_duty_cycle(motor_pwm_ch, 50);
  sl_pwm_start(motor_pwm_ch);
}
// ==================================== GPIO motor Pin PWM out Disable - END ===================================


// ==================================== GPIO motor Pin PWM out High - START ===================================
void GPIO_motorPinPWMoutHigh(sl_pwm_instance_t *motor_pwm_ch, EMotor motor){
  sl_pwm_set_duty_cycle(motor_pwm_ch, motors[motor].PWMCommand);
  sl_pwm_start(motor_pwm_ch);
}
// ==================================== GPIO motor Pin PWM out High - END ===================================


// ==================================== GPIO motor Pin PWM out Low - START ===================================
void GPIO_motorPinPWMoutLow(sl_pwm_instance_t *motor_pwm_ch, EMotor motor){
  (void) motor; // unused argument
//  sl_pwm_set_duty_cycle(motor_pwm_ch, 0);
//  sl_pwm_start(motor_pwm_ch);
  sl_pwm_stop(motor_pwm_ch);
}
// ==================================== GPIO motor Pin PWM out Low - END ===================================


// ==================================== set Motor Drive State - START ===================================
void setMotorDriveState(EMotor motor)
{
	if (IS_ZERO_FLOAT(motors[motor].speedControler.speedFromHull))
	{
		motors[motor].motorDriveState = DS_STOP;
	}
	else if (motors[motor].speedControler.speedFromHull > 0)
	{
		motors[motor].motorDriveState = DS_CW;
	}
	else
	{
		motors[motor].motorDriveState = DS_CCW;
	}
}
// ==================================== set Motor Drive State - END ===================================


// ==================================== set All Motors Drive State - START ===================================
void setAllMotorsDriveState(void)
{
  for(EMotor motor = left; motor < endOfMotors; motor++)
  {
      setMotorDriveState(motor);
  }
}
// ==================================== set all Motors Drive State - START ===================================


// ==================================== motor Driver Phase Configuration Initialize - START ===================================
void motorDriverPhaseConfigurationInit(void){

	// Xpolarity = 0 - LOW
	// Xpolarity = 1 - HIGH
	// Xpolarity = 2 - 50% duty cycle

	// ======================= FORWORD CONFIGURATION - START =============================
	motorPhaseConfiguration.forword[0].Upolarity = NA;
	motorPhaseConfiguration.forword[0].Vpolarity = NA;
	motorPhaseConfiguration.forword[0].Wpolarity = NA;


	motorPhaseConfiguration.forword[1].Upolarity = D;
	motorPhaseConfiguration.forword[1].Vpolarity = H;
	motorPhaseConfiguration.forword[1].Wpolarity = L;


	motorPhaseConfiguration.forword[2].Upolarity = H;
	motorPhaseConfiguration.forword[2].Vpolarity = L;
	motorPhaseConfiguration.forword[2].Wpolarity = D;


	motorPhaseConfiguration.forword[3].Upolarity = H;
	motorPhaseConfiguration.forword[3].Vpolarity = D;
	motorPhaseConfiguration.forword[3].Wpolarity = L;


	motorPhaseConfiguration.forword[4].Upolarity = L;
	motorPhaseConfiguration.forword[4].Vpolarity = D;
	motorPhaseConfiguration.forword[4].Wpolarity = H;


	motorPhaseConfiguration.forword[5].Upolarity = L;
	motorPhaseConfiguration.forword[5].Vpolarity = H;
	motorPhaseConfiguration.forword[5].Wpolarity = D;


	motorPhaseConfiguration.forword[6].Upolarity = D;
	motorPhaseConfiguration.forword[6].Vpolarity = L;
	motorPhaseConfiguration.forword[6].Wpolarity = H;


	motorPhaseConfiguration.forword[7].Upolarity = NA;
	motorPhaseConfiguration.forword[7].Vpolarity = NA;
	motorPhaseConfiguration.forword[7].Wpolarity = NA;
	// ======================= FORWORD CONFIGURATION - END =============================


	// ======================= BACKWORD CONFIGURATION - START =============================
	motorPhaseConfiguration.backword[0].Upolarity = NA;
	motorPhaseConfiguration.backword[0].Vpolarity = NA;
	motorPhaseConfiguration.backword[0].Wpolarity = NA;


	motorPhaseConfiguration.backword[1].Upolarity = L;
	motorPhaseConfiguration.backword[1].Vpolarity = H;
	motorPhaseConfiguration.backword[1].Wpolarity = D;


	motorPhaseConfiguration.backword[2].Upolarity = D;
	motorPhaseConfiguration.backword[2].Vpolarity = L;
	motorPhaseConfiguration.backword[2].Wpolarity = H;


	motorPhaseConfiguration.backword[3].Upolarity = L;
	motorPhaseConfiguration.backword[3].Vpolarity = D;
	motorPhaseConfiguration.backword[3].Wpolarity = H;


	motorPhaseConfiguration.backword[4].Upolarity = H;
	motorPhaseConfiguration.backword[4].Vpolarity = D;
	motorPhaseConfiguration.backword[4].Wpolarity = L;


	motorPhaseConfiguration.backword[5].Upolarity = D;
	motorPhaseConfiguration.backword[5].Vpolarity = H;
	motorPhaseConfiguration.backword[5].Wpolarity = L;


	motorPhaseConfiguration.backword[6].Upolarity = H;
	motorPhaseConfiguration.backword[6].Vpolarity = L;
	motorPhaseConfiguration.backword[6].Wpolarity = D;


	motorPhaseConfiguration.backword[7].Upolarity = NA;
	motorPhaseConfiguration.backword[7].Vpolarity = NA;
	motorPhaseConfiguration.backword[7].Wpolarity = NA;
	// ======================= BACKWORD CONFIGURATION - END =============================
}
// ==================================== motor Driver Phase Configuration Initialize - END ===================================


void getHullSequence(EMotor motor)
{
	uint8_t sequence = ((motors[motor].hull.HullU << 2) | (motors[motor].hull.HullV << 1) | (motors[motor].hull.HullW)) & 0x7;
	motors[motor].hull.currentSequence = sequence-1;
}


void calcHullAdd(EMotor motor)
{
  motors[motor].hull.hullAdd =  hallIrqCntAdevanceMatrix[motors[motor].hull.prevSequence][motors[motor].hull.currentSequence];
  motors[motor].hull.prevSequence = motors[motor].hull.currentSequence;
}

void calcSpeedFromHalls(EMotor motor)
{
  // to prevent changes in current time and hall counts during calculation we copy them to local variables
  uint32_t curentTimeMillis = motors[motor].hull.cnt_last_time_millis;
  uint32_t currentHallCnt = motors[motor].hull.cnt;

	if (motors[motor].motorDriveState == DS_STOP)
  {
	    motors[motor].speedControler.speedFromHull = 0.0;
	    motors[motor].speedControler.speedAverage.reset = true;
	    continuousAverage(&motors[motor].speedControler.speedAverage);
  }

	int currentDeltaCount = currentHallCnt - motors[motor].speedControler.lastHullCnt;
	uint32_t currentDt = curentTimeMillis - motors[motor].speedControler.lastCalcTimeMillis;

	if (currentDt == 0 || currentDeltaCount == 0)
	{
		//do nothing and take the last calculated "speedOut"
	  return;
	}

  float motorAngleRotated = currentDeltaCount * RAD_PER_INTERAPT;
  float dtSec = currentDt/1000.0;
  float motorSpeedBeforeGearRadSec = motorAngleRotated/dtSec;
  motors[motor].speedControler.speedFromHull = motorSpeedBeforeGearRadSec * INV_GEAR_RATIO;

	motors[motor].speedControler.lastCalcTimeMillis = curentTimeMillis;
	motors[motor].speedControler.lastHullCnt = currentHallCnt;

	motors[motor].speedControler.speedAverage.courentData = motors[motor].speedControler.speedFromHull;
  continuousAverage(&motors[motor].speedControler.speedAverage);
	return;
}


void speedControlHandle(EMotor motor)
{
  float speedCorrection = PISpeedControl(motor);
  motors[motor].speedControler.speedCorrected = motors[motor].speedControler.refSpeed + speedCorrection;
#ifdef DEBUG_SPEED_CONTROL
  record_motor_data(motor);
#endif
}


void resetMotorData(EMotor motor)
{
	motors[motor].speedControler.speed_I_correction = 0;
	motors[motor].speedControler.speedCorrected = 0;
	motors[motor].speedControler.speedFromHull = 0;
	motors[motor].speedControler.prevSpeedFromHall = 0;
	motors[motor].speedControler.refSpeed = 0;
	motors[motor].speedControler.speedCorrected = 0;
	motors[motor].speedControler.speedAverage.reset = true;
	setMotorDriveState(motor);
}


void resetAllDriveMotorsData(void)
{
  for(EMotor motor = left; motor < endOfMotors; motor++)
  {
    resetMotorData(motor);
  }
}


// ==================================== calc PWM command - START ===================================
void calcPWMpercent(EMotor motor)
{
  // calculate the motor needed voltage after PI controller
  float refMotorSpeedRPM = fabs(motors[motor].speedControler.speedCorrected) * GEAR_RATIO * RPS_TO_RPM;
  float commandVrms = (refMotorSpeedRPM / MOTOR_SPEED_CONSTANT) / MOTOR_EFFICIENCY;
  motors[motor].PWMCommand = (commandVrms/POWER_SUPPLY_VOLTAGE)*100.0;

#ifdef DEBUG_SPEED_CONTROL
  record_refMotorSpeedRPM(motor, refMotorSpeedRPM);
  record_commandVrms(motor, commandVrms);
#endif
  return;
}
// ==================================== calc PWM command - END ===================================


void sendPWMCommadToAllMotors(void)
{
  for(EMotor motor = left; motor < endOfMotors; motor++)
  {
      sendCommandToDriver(motor);
  }
}
