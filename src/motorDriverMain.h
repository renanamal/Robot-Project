#ifndef INC_MOTORDRIVERMAIN_H_
#define INC_MOTORDRIVERMAIN_H_

#include "generalDefines.h"
#include "stdbool.h"
#include "sl_pwm.h"
#include "stdint.h"
#include "generalPurposeFunctions.h"
#include "motorsDB.h"
#include "motorDriverMain.h"
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

#define MOTOR_PHASE_CONFIG_SIZE (8)


typedef enum{
  right = 0,
  left,
  endOfMotors
}EMotor;

typedef struct
{
  uint8_t Apolarity;
  uint8_t Bpolarity;
  uint8_t Cpolarity;
}S_motorPhaseConfiguration;

typedef struct
{
  S_motorPhaseConfiguration forword[MOTOR_PHASE_CONFIG_SIZE];
  S_motorPhaseConfiguration backword[MOTOR_PHASE_CONFIG_SIZE];
}S_fullMotorPhaseConfiguration;

typedef struct
{
  uint8_t currentSequence;
  uint8_t HallA;
  uint8_t HallB;
  uint8_t HallC;
  int32_t cnt;
  int8_t  prevHallAdded;
  uint32_t cnt_last_time_us;
}SGDComutation;

typedef enum{
  DS_STOP = 0,
  DS_CW,
  DS_CCW
}e_driveState;


typedef struct {
  float               speed_I_correction;
  float               speedCorrected;
  float               speedFromHall;
  float               prevSpeedFromHall;
  uint64_t            lastCalcTimeUs;
  int64_t             last_hall_cnt;
  float               refSpeed;
  float               correctedSpeed;
  SContinuousAverage  speedAverage;
}SPIspeedContorl;

typedef struct{
  SGDComutation                 hall;
  S_motorPhaseConfiguration     commutation;
  float                         payloadAngle;
  e_driveState                  motorDriveState;
  uint8_t                       PWMCommand;
  SPIspeedContorl               speedControler;
  bool                          isRunning;
}SMotorsData;



// ================= function prototypes ===========================
void sendCommandToDriver(EMotor motor);
void getAllMotorsCommutation(void);
void getMotorComutation(EMotor motor);
float PISpeedControl(EMotor motor);
void GPIO_motorPinPWMoutDisable(sl_pwm_instance_t *motor_ch, EMotor motor);
void GPIO_motorPinPWMoutHigh(sl_pwm_instance_t *motor_ch, EMotor motor);
void GPIO_motorPinPWMoutLow(sl_pwm_instance_t *motor_ch, EMotor motor);
void setMotorDriveState(EMotor motor);
void setAllMotorsDriveState(void);
void motorDriverPhaseConfigurationInit(void);
void getHallSequence(EMotor motor);
float calcSpeedFromHalls(EMotor motor);
void speedControlHandle(EMotor motor);
void resetMotorData(EMotor motor);
void resetAllDriveMotorsData(void);
void calcPWMpercent(EMotor motor);
void sendPWMCommadToAllMotors(void);

// ============================= Type def =======================
typedef void(*fctPtr)(sl_pwm_instance_t *, EMotor);

#endif /* INC_MOTORDRIVERMAIN_H_ */
