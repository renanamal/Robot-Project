#include <src/motorsDB.h>
#include "motorDriverMain.h"
#include "generalDefines.h"
#include "sl_pwm.h"
#include "sl_emlib_gpio_init_PA6_config.h"
#include "sl_emlib_gpio_init_PA7_config.h"
#include "sl_emlib_gpio_init_PA8_config.h"
#include "sl_emlib_gpio_init_PA9_config.h"
#include "sl_emlib_gpio_init_PB6_config.h"
#include "sl_emlib_gpio_init_PB7_config.h"
#include "sl_emlib_gpio_init_PB8_config.h"
#include "sl_emlib_gpio_init_PB9_config.h"
#include "sl_emlib_gpio_init_PC11_config.h"
#include "sl_emlib_gpio_init_PC4_config.h"
#include "sl_emlib_gpio_init_PC5_config.h"
#include "sl_emlib_gpio_init_PD8_config.h"
#include "em_gpio.h"

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
	//-------------------------------- enable the needed motor 1 driver pins ---------------------------------------
	(*pwmSetDutyCycle[motors[motor].commutation.Apolarity])(motor_pwm_ch0, motor);
	(*pwmSetDutyCycle[motors[motor].commutation.Bpolarity])(motor_pwm_ch1, motor);
	(*pwmSetDutyCycle[motors[motor].commutation.Cpolarity])(motor_pwm_ch2, motor);
}


void getAllMotorsCommutation(void)
{
  for(EMotor motor = left; motor < endOfMotors; motor++)
  {
      getMotorComutation(motor);
  }
}


void getMotorComutation(EMotor motor){
  switch(motor)
  {
    case left:
      motors[motor].hall.HallB = GPIO_PinInGet(SL_EMLIB_GPIO_INIT_PD8_PORT, SL_EMLIB_GPIO_INIT_PD8_PIN);
      motors[motor].hall.HallC = GPIO_PinInGet(SL_EMLIB_GPIO_INIT_PA6_PORT, SL_EMLIB_GPIO_INIT_PA6_PIN);
      motors[motor].hall.HallA = GPIO_PinInGet(SL_EMLIB_GPIO_INIT_PA7_PORT, SL_EMLIB_GPIO_INIT_PA7_PIN);
      break;

    case right:
      motors[motor].hall.HallB = GPIO_PinInGet(SL_EMLIB_GPIO_INIT_PB7_PORT, SL_EMLIB_GPIO_INIT_PB7_PIN);
      motors[motor].hall.HallC = GPIO_PinInGet(SL_EMLIB_GPIO_INIT_PC11_PORT, SL_EMLIB_GPIO_INIT_PC11_PIN);
      motors[motor].hall.HallA = GPIO_PinInGet(SL_EMLIB_GPIO_INIT_PB8_PORT, SL_EMLIB_GPIO_INIT_PB8_PIN);

  }

  gCommotationState[motor] = (motors[motor].hall.HallA << 2 | motors[motor].hall.HallB << 1 | motors[motor].hall.HallC) & 0x7;

  if ((motors[motor].motorDriveState == DS_CW) ||(motors[motor].motorDriveState == DS_STOP))
  {
    motors[motor].commutation = motorPhaseConfiguration.forword[gCommotationState[motor]];
  }
  else if (motors[motor].motorDriveState == DS_CCW)
  {
    motors[motor].commutation = motorPhaseConfiguration.backword[gCommotationState[motor]];
  }

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
	return Pcorrection + speed_I_correction;
}
// ==================================== PI linear speed control algorithm - START ===================================


// ==================================== GPIO motor Pin PWM out Disable - START ===================================
void GPIO_motorPinPWMoutDisable(sl_pwm_instance_t *motor_pwm_ch, EMotor motor)
{
  sl_pwm_set_duty_cycle(motor_pwm_ch, 50);
}
// ==================================== GPIO motor Pin PWM out Disable - END ===================================


// ==================================== GPIO motor Pin PWM out High - START ===================================
void GPIO_motorPinPWMoutHigh(sl_pwm_instance_t *motor_pwm_ch, EMotor motor){
  sl_pwm_set_duty_cycle(motor_pwm_ch, motors[motor].PWMCommand);
}
// ==================================== GPIO motor Pin PWM out High - END ===================================


// ==================================== GPIO motor Pin PWM out Low - START ===================================
void GPIO_motorPinPWMoutLow(sl_pwm_instance_t *motor_pwm_ch, EMotor motor){
  sl_pwm_set_duty_cycle(motor_pwm_ch, 0);
}
// ==================================== GPIO motor Pin PWM out Low - END ===================================


// ==================================== set Motor Drive State - START ===================================
void setMotorDriveState(EMotor motor)
{
	if (IS_ZERO_FLOAT(motors[motor].speedControler.speedFromHall))
	{
		motors[motor].motorDriveState = DS_STOP;
	}
	else if (motors[motor].speedControler.speedFromHall > 0)
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
	motorPhaseConfiguration.forword[0].Apolarity = 0;
	motorPhaseConfiguration.forword[0].Bpolarity = 0;
	motorPhaseConfiguration.forword[0].Cpolarity = 0;


	motorPhaseConfiguration.forword[1].Apolarity = 0;
	motorPhaseConfiguration.forword[1].Bpolarity = 1;
	motorPhaseConfiguration.forword[1].Cpolarity = 2;


	motorPhaseConfiguration.forword[2].Apolarity = 1;
	motorPhaseConfiguration.forword[2].Bpolarity = 2;
	motorPhaseConfiguration.forword[2].Cpolarity = 0;


	motorPhaseConfiguration.forword[3].Apolarity = 2;
	motorPhaseConfiguration.forword[3].Bpolarity = 1;
	motorPhaseConfiguration.forword[3].Cpolarity = 0;


	motorPhaseConfiguration.forword[4].Apolarity = 2;
	motorPhaseConfiguration.forword[4].Bpolarity = 0;
	motorPhaseConfiguration.forword[4].Cpolarity = 1;


	motorPhaseConfiguration.forword[5].Apolarity = 0;
	motorPhaseConfiguration.forword[5].Bpolarity = 2;
	motorPhaseConfiguration.forword[5].Cpolarity = 1;


	motorPhaseConfiguration.forword[6].Apolarity = 1;
	motorPhaseConfiguration.forword[6].Bpolarity = 0;
	motorPhaseConfiguration.forword[6].Cpolarity = 2;


	motorPhaseConfiguration.forword[7].Apolarity = 0;
	motorPhaseConfiguration.forword[7].Bpolarity = 0;
	motorPhaseConfiguration.forword[7].Cpolarity = 0;
	// ======================= FORWORD CONFIGURATION - END =============================


	// ======================= BACKWORD CONFIGURATION - START =============================
	motorPhaseConfiguration.backword[0].Apolarity = 0;
	motorPhaseConfiguration.backword[0].Bpolarity = 0;
	motorPhaseConfiguration.backword[0].Cpolarity = 0;


	motorPhaseConfiguration.backword[1].Apolarity = 1;
	motorPhaseConfiguration.backword[1].Bpolarity = 0;
	motorPhaseConfiguration.backword[1].Cpolarity = 2;


	motorPhaseConfiguration.backword[2].Apolarity = 0;
	motorPhaseConfiguration.backword[2].Bpolarity = 2;
	motorPhaseConfiguration.backword[2].Cpolarity = 1;


	motorPhaseConfiguration.backword[3].Apolarity = 2;
	motorPhaseConfiguration.backword[3].Bpolarity = 0;
	motorPhaseConfiguration.backword[3].Cpolarity = 1;


	motorPhaseConfiguration.backword[4].Apolarity = 2;
	motorPhaseConfiguration.backword[4].Bpolarity = 1;
	motorPhaseConfiguration.backword[4].Cpolarity = 0;


	motorPhaseConfiguration.backword[5].Apolarity = 1;
	motorPhaseConfiguration.backword[5].Bpolarity = 2;
	motorPhaseConfiguration.backword[5].Cpolarity = 0;


	motorPhaseConfiguration.backword[6].Apolarity = 0;
	motorPhaseConfiguration.backword[6].Bpolarity = 1;
	motorPhaseConfiguration.backword[6].Cpolarity = 2;


	motorPhaseConfiguration.backword[7].Apolarity = 0;
	motorPhaseConfiguration.backword[7].Bpolarity = 0;
	motorPhaseConfiguration.backword[7].Cpolarity = 0;
	// ======================= BACKWORD CONFIGURATION - END =============================
}
// ==================================== motor Driver Phase Configuration Initialize - END ===================================


void getHallSequence(EMotor motor)
{
	uint8_t sequence = ((motors[motor].hall.HallA << 2) | (motors[motor].hall.HallB << 1) | (motors[motor].hall.HallC)) & 0x7;
	motors[motor].hall.currentSequence = sequence-1;
}


float calcSpeedFromHalls(EMotor motor)
{
	static float speedOut[NUM_OF_MOTORS] = {0,0};

	if (motors[motor].motorDriveState == DS_STOP)
  {
    speedOut[motor] = 0;
    return speedOut[motor];
  }

	// to prevent changes in current time and hall counts during calculation we copy them to local variables
	uint32_t curentTimeUs = motors[motor].hall.cnt_last_time_us;
	uint32_t currentHallCnt = motors[motor].hall.cnt;

	int currentDeltaCount = currentHallCnt - motors[motor].speedControler.last_hall_cnt;
	uint32_t currentDt = curentTimeUs - motors[motor].speedControler.lastCalcTimeUs;

	if (currentDt == 0 || currentDeltaCount == 0)
	{
		//do nothing and take the last calculated "speedOut"
	    return speedOut[motor];
	}

  float motorAngleRotated = currentDeltaCount * RAD_PER_INTERAPT;
  float dtSec = currentDt/1000000.0;
  float motorSpeedBeforeGearRadSec = motorAngleRotated/dtSec;
  speedOut[motor] = motorSpeedBeforeGearRadSec * INV_GEAR_RATIO;

	motors[motor].speedControler.lastCalcTimeUs = curentTimeUs;
	motors[motor].speedControler.last_hall_cnt = currentHallCnt;
	return speedOut[motor];
}


void speedControlHandle(EMotor motor)
{
  float speedCorrection = PISpeedControl(motor);
  motors[motor].speedControler.correctedSpeed = motors[motor].speedControler.refSpeed + speedCorrection;
}


void resetMotorData(EMotor motor)
{
	motors[motor].speedControler.speed_I_correction = 0;
	motors[motor].speedControler.speedCorrected = 0;
	motors[motor].speedControler.speedFromHall = 0;
	motors[motor].speedControler.prevSpeedFromHall = 0;
	motors[motor].speedControler.refSpeed = 0;
	motors[motor].speedControler.correctedSpeed = 0;
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
