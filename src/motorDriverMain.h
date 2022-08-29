#ifndef INC_MOTORDRIVERMAIN_H_
#define INC_MOTORDRIVERMAIN_H_

#include "globalDefs.h"

typedef struct ScontorlData{
	float rateCorrectedFromGyro;
	float correctedRateSpeed;
	float dLinearCommandFromRate;
	float LinearSpeedCorrectedMmSec;
}ScontorlData;


typedef	struct SaverageInclinationMeasures{
	bool finished;
	float angle;
}SaverageInclinationMeasures;

// function definition
uint8_t commotationConfig(void);
void motorDriverPhaseConfigurationInit(void);
void calcMotorDriverCommand(uint8_t motorNum);
void sendCommandToDriver(uint8_t motorNum);
void GPIO_motorPinPWMoutDisable(GPIO_Port_TypeDef Port,unsigned int Pin,uint8_t Phase, uint8_t motorNum);
void GPIO_motorPinPWMoutLow(GPIO_Port_TypeDef Port,unsigned int Pin,uint8_t Phase, uint8_t motorNum);
void GPIO_motorPinPWMoutHigh(GPIO_Port_TypeDef Port,unsigned int Pin,uint8_t Phase, uint8_t motorNum);
void getMotorComutation(uint8_t motorNum);
void speedControlAlg(uint8_t motorNum,uint8_t linearSpeedRefPWM);
SPIspeedContorlOut PISpeedControl(float calcSpeed, float refSpeed, float Kp, float Ki, SPIspeedContorl *dataIn, uint8_t motorNum);
uint8_t calcPWMSpeedFromLinearSpeed(uint8_t motorNum);
uint8_t calcPWMlinearCommand(uint8_t linearPWMcommand,uint8_t motorNum);
uint16_t roundToNearestXPercents(uint16_t linearPWMcommand,float percentToRoundTo);
void setDriveState();
void getHallSequence(uint8_t motorNum);
float calcLinearCommandFromRateData(float speedIn);
float calcWeightToDevideDLinearCommandFromRate(uint8_t linearRefPWMcommand,uint8_t dSpeedPWM);
float calcLinearSpeedFromHalls(uint8_t motorNum);
float calcVlim();
void setAllMotorsDriveState(void);
void getAllMotorsCommutation(void);
void robotRateSpeedControlHandle(ScontorlData *contorlData, bool controlOn, float rateRef);
void LinearSpeedControlHandle(uint8_t motorNum, bool controlOn, float linearRef);
void getMotorComutationNoHalls(uint8_t motorNum);
void changeMotorDriveControlType(EDeviceName motorName, EDriveControlType driveControlType);
e_armMoveToAng moveArmToAngle(float angleRef, bool reset);
void disableNVICgpioIRQ(void);
void enableNVICgpioIRQ(void);
void resetMotorsData(int8_t motorNum);
void setMotorDriveState(uint8_t motorNum);
void robotLinearSpeedControlHandle(ScontorlData *contorlData, float linearRef);
void resetAllDriveMotorsData(void);
float rateFilter(float currentMeasure);
SaverageInclinationMeasures averageInclinationMeasures(bool reset);

typedef void(*fctPtr)(GPIO_Port_TypeDef,unsigned int,uint8_t,uint8_t);

//uint32_t linearCommandClock;

typedef struct {
	GPIO_Port_TypeDef Port;
	unsigned int Pin;
}S_motorDriverPortPin;

S_motorDriverPortPin motorDriverPortPinArray[3][3];

#endif /* INC_MOTORDRIVERMAIN_H_ */
