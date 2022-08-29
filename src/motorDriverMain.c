#include <globals.h>

fctPtr gpioPinSetFunCallArray[] = {GPIO_motorPinPWMoutLow , GPIO_motorPinPWMoutHigh , GPIO_motorPinPWMoutDisable};

// define external params
#ifdef SOFTWARE_DBG
SDBGMainDBG mainDBG;
#endif

// ========================================================= motor Driver Main =======================================================================
void calcMotorDriverCommand(uint8_t motorNum)
{
	float linearRef = 0;
	if (motorNum == D_NAME_DRIVE_MOTOR_LEFT || motorNum == D_NAME_DRIVE_MOTOR_RIGHT)
	{
		linearRef = gd.speed[motorNum].linearCommandSpeedToMotor;
	}
	else
	{
		linearRef = gd.speed[motorNum].linearCommandSpeedToMotorRef * 30;
	}

	if (gd.speed[motorNum].driveControlType == SPEED_CONTROL_ON)
	{
		LinearSpeedControlHandle(motorNum,true,linearRef);
	}
	else
	{
		LinearSpeedControlHandle(motorNum,false,linearRef);
	}

#ifdef SPEED_CONTROL_DBG
	// ========================================================== DEBUG - START ========================================================================
	mainDBG.speedControl[motorNum].linearFromRateSpeed[mainDBG.speedControl[motorNum].counter] = gd.speed[D_NAME_DUMMY_RATE_DATA].dLinearCommandFromRate;
	mainDBG.speedControl[motorNum].counter++;
	mainDBG.speedControl[motorNum].totalLinearSpeed[mainDBG.speedControl[motorNum].counter] = gd.speed[motorNum].totalLinearSpeedMmSecCorrected;
	mainDBG.speedControl[motorNum].PI_correction[mainDBG.speedControl[motorNum].counter] = gd.speed[motorNum].Speed_I_correction;
	mainDBG.speedControl[motorNum].PWM[mainDBG.speedControl[motorNum].counter] =  gd.motor[motorNum].PWMCommand;
	if (mainDBG.speedControl[motorNum].counter > (speedControlDBGarraySize - 1))
	{
		DEBUG_BREAK;
		while (1);
	}
	// ========================================================== DEBUG - END ==========================================================================
#endif
	// ======================================== handle Linear speed - END ====================================================
	// ======================================= Update global data for next step - START ======================================================
	//gd.speed[D_NAME_DUMMY_RATE_DATA].dLinearCommandFromRate = dLinearCommandFromRate;
}


void sendCommandToDriver(uint8_t motorNum){
	//-------------------------------- enable the needed motor 1 driver pins ---------------------------------------
	(*gpioPinSetFunCallArray[gd.motor[motorNum].commutation.Apolarity])(mcuPinConfig.motorInitConfig[motorNum].PWMPort[0],mcuPinConfig.motorInitConfig[motorNum].PWMPinNum[0],0,motorNum);
	(*gpioPinSetFunCallArray[gd.motor[motorNum].commutation.Bpolarity])(mcuPinConfig.motorInitConfig[motorNum].PWMPort[1],mcuPinConfig.motorInitConfig[motorNum].PWMPinNum[1],1,motorNum);
	(*gpioPinSetFunCallArray[gd.motor[motorNum].commutation.Cpolarity])(mcuPinConfig.motorInitConfig[motorNum].PWMPort[2],mcuPinConfig.motorInitConfig[motorNum].PWMPinNum[2],2,motorNum);
}


void getAllMotorsCommutation(void)
{
	for (uint8_t motorNum = 0; motorNum < MAX_NUM_OF_MOTORS; motorNum++)
	{
		if (gd.devicesOn.motorsOn[motorNum])
		{
			getMotorComutation(motorNum);
		}
	}
}


void getMotorComutation(uint8_t motorNum){
	gd.motor[motorNum].hall.HallB = GPIO_PinInGet(mcuPinConfig.motorInitConfig[motorNum].HallPort[0], mcuPinConfig.motorInitConfig[motorNum].HallPinNum[0]);
	gd.motor[motorNum].hall.HallC = GPIO_PinInGet(mcuPinConfig.motorInitConfig[motorNum].HallPort[1], mcuPinConfig.motorInitConfig[motorNum].HallPinNum[1]);
	gd.motor[motorNum].hall.HallA = GPIO_PinInGet(mcuPinConfig.motorInitConfig[motorNum].HallPort[2], mcuPinConfig.motorInitConfig[motorNum].HallPinNum[2]);

	gCommotationState[motorNum] = (gd.motor[motorNum].hall.HallA << 2 | gd.motor[motorNum].hall.HallB << 1 | gd.motor[motorNum].hall.HallC) & 0x7;

	if ((gd.motor[motorNum].motorDriveState == DS_FORWORD) ||(gd.motor[motorNum].motorDriveState == DS_STOP))
	{
		gd.motor[motorNum].commutation = motorPhaseConfiguration.forword[gCommotationState[motorNum]];
	}
	else if (gd.motor[motorNum].motorDriveState == DS_BACKWORD)
	{
		gd.motor[motorNum].commutation = motorPhaseConfiguration.backword[gCommotationState[motorNum]];
	}
}


void getMotorComutationNoHalls(uint8_t motorNum)
{
	// ===================================== Motor driving programmer no Halls - start =============================================
	//	static bool recordSeq = false;
	//	static uint8_t HallTemp[50];
	//	static uint8_t counter = 0;
	//	if (recordSeq)
	//	{
	//		HallTemp[counter] = 0;
	//		HallTemp[counter] |= (GPIO_PinInGet(mcuPinConfig.motorInitConfig[motorNum].HallPort[0], mcuPinConfig.motorInitConfig[motorNum].HallPinNum[0])&0x1)<<2;
	//		HallTemp[counter] |= (GPIO_PinInGet(mcuPinConfig.motorInitConfig[motorNum].HallPort[1], mcuPinConfig.motorInitConfig[motorNum].HallPinNum[1])&0x1)<<1;
	//		HallTemp[counter] |= GPIO_PinInGet(mcuPinConfig.motorInitConfig[motorNum].HallPort[2], mcuPinConfig.motorInitConfig[motorNum].HallPinNum[2])&0x1;
	//		++counter;
	//		if (counter > 49)
	//		{
	//			int a = 1;
	//		}
	//	}
	//	int8_t HallBTemp = !GPIO_PinInGet(mcuPinConfig.motorInitConfig[motorNum].HallPort[0], mcuPinConfig.motorInitConfig[motorNum].HallPinNum[0]);
	//	int8_t HallCTemp = !GPIO_PinInGet(mcuPinConfig.motorInitConfig[motorNum].HallPort[1], mcuPinConfig.motorInitConfig[motorNum].HallPinNum[1]);
	//	int8_t HallATemp = !GPIO_PinInGet(mcuPinConfig.motorInitConfig[motorNum].HallPort[2], mcuPinConfig.motorInitConfig[motorNum].HallPinNum[2]);

	const static uint8_t HallProgramerForward[6][3] = {{0,1,1},{0,0,1},{1,0,1},{1,0,0},{1,1,0},{0,1,0}};
	const static uint8_t HallProgramerBackward[6][3] = {{0,1,0},{1,1,0},{1,0,0},{1,0,1},{0,0,1},{0,1,1}};
	static int counterHallProgramer[MAX_NUM_OF_MOTORS] = {0,0,0};

	if (gd.motor[motorNum].motorDriveState == DS_FORWORD)
	{
		gd.motor[motorNum].hall.HallA = HallProgramerForward[counterHallProgramer[motorNum]][0];
		gd.motor[motorNum].hall.HallB = HallProgramerForward[counterHallProgramer[motorNum]][1];
		gd.motor[motorNum].hall.HallC = HallProgramerForward[counterHallProgramer[motorNum]][2];
	}
	if(gd.motor[motorNum].motorDriveState == DS_BACKWORD)
	{
		gd.motor[motorNum].hall.HallA = HallProgramerBackward[counterHallProgramer[motorNum]][0];
		gd.motor[motorNum].hall.HallB = HallProgramerBackward[counterHallProgramer[motorNum]][1];
		gd.motor[motorNum].hall.HallC = HallProgramerBackward[counterHallProgramer[motorNum]][2];
	}
	//	static uint8_t hallTemp[3];
	//
	//	gd.motor[motorNum].hall.HallA = hallTemp[0];
	//	gd.motor[motorNum].hall.HallB = hallTemp[1];
	//	gd.motor[motorNum].hall.HallC = hallTemp[2];

	gCommotationState[motorNum] = (gd.motor[motorNum].hall.HallA << 2 | gd.motor[motorNum].hall.HallB << 1 | gd.motor[motorNum].hall.HallC) & 0x7;

	if ((gd.motor[motorNum].motorDriveState == DS_FORWORD) ||(gd.motor[motorNum].motorDriveState == DS_STOP))
	{
		gd.motor[motorNum].commutation = motorPhaseConfiguration.forword[gCommotationState[motorNum]];
	}
	else if (gd.motor[motorNum].motorDriveState == DS_BACKWORD)
	{
		gd.motor[motorNum].commutation = motorPhaseConfiguration.backword[gCommotationState[motorNum]];
	}

	++counterHallProgramer[motorNum];
	if (counterHallProgramer[motorNum] > 5)
	{
		counterHallProgramer[motorNum] = 0;
	}
	// ===================================== Motor driving programmer no Halls - end =============================================
}


// ==================================== PI speed control algorithm - START ===================================
SPIspeedContorlOut PISpeedControl(float calcSpeed, float refSpeed, float Kp, float Ki, SPIspeedContorl *dataIn, uint8_t motorNum)
{
	SPIspeedContorlOut dataOut;
	float SpeedError = refSpeed - calcSpeed;
	uint16_t antiWindup;

	float Pcorrection = Kp * SpeedError;
	dataOut.Speed_I_correction = Ki * SpeedError + dataIn->Speed_I_correction;
	antiWindup = gd.robot.PI_drive_motor_antiWindup;

	if (dataOut.Speed_I_correction > antiWindup) // unti-windup
	{
		dataOut.Speed_I_correction = antiWindup;
	}

	if (dataOut.Speed_I_correction < -1 * antiWindup) // unti-windup
	{
		dataOut.Speed_I_correction = -1 * antiWindup;
	}
	dataOut.Speedcorrection = Pcorrection + dataOut.Speed_I_correction;
	return dataOut;
}
// ==================================== PI linear speed control algorithm - START ===================================



// ==================================== calc PWM command From Linear Ref Command - START ===================================
uint8_t calcPWMSpeedFromLinearSpeed(uint8_t motorNum)
{
	// calculate the motor needed voltage after PI controller
	float refMotorSpeedRPM = fabs(gd.speed[motorNum].totalLinearSpeedMmSecCorrected) * gd.constants.wheel_mm_sec_to_rev_sec * gd.robot.drivingMotorGearRatio * gd.constants.rps_to_rpm;
	float commandVrms = gd.robot.PWMmanualCorrection * (refMotorSpeedRPM / gd.motor[motorNum].motorCharacters.speedConst) / gd.motor[motorNum].motorCharacters.efficiency;
	uint8_t PWMcommandOut = commandVrms;
	return PWMcommandOut;
}
// ==================================== calc PWM command From Linear Ref Command - END ===================================


// ==================================== calc PWM linear Command - START ===================================
uint8_t calcPWMlinearCommand(uint8_t linearPWMcommand,uint8_t motorNum)
{
	bool doCalcLinearCommandClock = false;
	if (gd.motor[motorNum].motorDriveState != DS_STOP)
	{
		doCalcLinearCommandClock = true;
	}

	if(doCalcLinearCommandClock)
	{
		if (motorNum < MAX_NUM_OF_MOTORS - 1)
		{
			if (linearPWMcommand > gd.robot.maxLinearPWMCommandAllawedDrive)
			{
				linearPWMcommand = gd.robot.maxLinearPWMCommandAllawedDrive;
			}
			else if (linearPWMcommand < 5)//gd.robot.minLinearPWMCommandAllawedDrive[motorNum])
			{
				linearPWMcommand = 5;//gd.robot.minLinearPWMCommandAllawedDrive[motorNum];
			}
		}
		else
		{
			// arm motor - do nothing
		}
		//	linearPWMcommand = roundToNearestFivePercents(linearPWMcommand);
//		linearPWMcommand = round(linearPWMcommand);
		int maxPWMClock = TIMER_TopGet(mcuPinConfig.motorInitConfig[motorNum].timer);
		float per = (double)linearPWMcommand/100.0;
		gd.motor[motorNum].PWMCommand = linearPWMcommand;
		gd.motor[motorNum].linearCommandClock = per*maxPWMClock; //[% of maxPWMClock]
	}
	else
	{
		gd.motor[motorNum].linearCommandClock = 0;
	}
	return linearPWMcommand;
}
// ==================================== calc PWM linear Command - END ===================================




// ==================================== GPIO motor Pin PWM out Disable - START ===================================
void GPIO_motorPinPWMoutDisable(GPIO_Port_TypeDef Port,unsigned int Pin,uint8_t Phase, uint8_t motorNum)
{
	TIMER_CompareBufSet(mcuPinConfig.motorInitConfig[motorNum].timer, Phase, gd.motor[motorNum].linearCommandClock/2);
}
// ==================================== GPIO motor Pin PWM out Disable - END ===================================


// ==================================== GPIO motor Pin PWM out High - START ===================================
void GPIO_motorPinPWMoutHigh(GPIO_Port_TypeDef Port,unsigned int Pin,uint8_t Phase, uint8_t motorNum){
	TIMER_CompareBufSet(mcuPinConfig.motorInitConfig[motorNum].timer, Phase, gd.motor[motorNum].linearCommandClock);
}
// ==================================== GPIO motor Pin PWM out High - END ===================================


// ==================================== GPIO motor Pin PWM out Low - START ===================================
void GPIO_motorPinPWMoutLow(GPIO_Port_TypeDef Port,unsigned int Pin,uint8_t Phase, uint8_t motorNum){
	TIMER_CompareBufSet(mcuPinConfig.motorInitConfig[motorNum].timer, Phase, 0);
}
// ==================================== GPIO motor Pin PWM out Low - END ===================================


// ==================================== round To Nearest Five Percents - START ===================================
uint16_t roundToNearestXPercents(uint16_t dataIn,float percentToRoundTo){
	uint16_t dataOut;
	double dataInD = ((double)dataIn)/10.0;
	double dataInR = round(dataInD); // from [%] to double
	if (dataInR > dataInD)
	{
		if ((dataInR - dataInD) > (percentToRoundTo/20))
		{
			dataOut = (dataInR - percentToRoundTo/10) * 10;
		}
		else
		{
			dataOut = dataInR * 10;
		}
	}
	else
	{
		if ((dataInD - dataInR) > percentToRoundTo/20)
		{
			dataOut = (dataInR + percentToRoundTo/10) * 10;
		}
		else
		{
			dataOut = dataInR * 10;
		}
	}
	return dataOut;
}
// ==================================== round To Nearest Five Percents - END ===================================



// ==================================== set Motor Drive State - START ===================================
void setMotorDriveState(uint8_t motorNum)
{
	if (IS_ZERO_FLOAT(gd.speed[motorNum].totalLinearSpeedMmSecCorrected))
	{
		gd.motor[motorNum].motorDriveState = DS_STOP;
	}
	else if (gd.speed[motorNum].totalLinearSpeedMmSecCorrected > 0)
	{
#ifdef ROBOT_11
		gd.motor[motorNum].motorDriveState = DS_BACKWORD;
#else
		gd.motor[motorNum].motorDriveState = DS_FORWORD;
#endif
	}
	else
	{
#ifdef ROBOT_11
		gd.motor[motorNum].motorDriveState = DS_FORWORD;
#else
		gd.motor[motorNum].motorDriveState = DS_BACKWORD;
#endif
	}
}
// ==================================== set Motor Drive State - END ===================================


// ==================================== set All Motors Drive State - START ===================================
void setAllMotorsDriveState()
{
	for (uint8_t motorNum = 0; motorNum < MAX_NUM_OF_MOTORS; motorNum++)
	{
		if (gd.devicesOn.motorsOn[motorNum])
		{
			setMotorDriveState(motorNum);
		}
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


void getHallSequence(uint8_t motorNum)
{
	uint8_t sequence = ((gd.motor[motorNum].hall.HallA << 2) | (gd.motor[motorNum].hall.HallB << 1) | (gd.motor[motorNum].hall.HallC)) & 0x7;
	gd.motor[motorNum].hall.currentSequence = sequence-1;
}


float calcLinearSpeedFromHalls(uint8_t motorNum)
{
	static uint32_t last_hall_cnt[MAX_NUM_OF_MOTORS] = {0,0,0};
	static uint32_t lastCalcTimeUs[MAX_NUM_OF_MOTORS] = {0,0,0};
	static float speedOut[MAX_NUM_OF_MOTORS] = {0,0,0};
	uint32_t curentTimeUs = gd.motor[motorNum].hall.cnt_last_time_us;
	uint32_t currentHallCnt = gd.motor[motorNum].hall.cnt;
	int currentDeltaCount = currentHallCnt - last_hall_cnt[motorNum];
	uint32_t currentDt = curentTimeUs - lastCalcTimeUs[motorNum];
	float CurrentMotorSpeedMmSec = 0;
	if (gd.motor[motorNum].motorDriveState == DS_STOP)
	{
		speedOut[motorNum] = 0;
	}
	else if (currentDt == 0 || currentDeltaCount == 0)
	{
		//do nothing and take the last calculated "speedOut"
	}
	else
	{
		float motorAngleRotated = currentDeltaCount * gd.constants.rad_per_interupt;
		float dtSec = currentDt/1000000.0;
		float motorSpeedBeforeGearRadSec = motorAngleRotated/dtSec;

		CurrentMotorSpeedMmSec = (motorSpeedBeforeGearRadSec * gd.robot.invDrivingMotorGearRatio)*gd.constants.rad_to_mm_wheel;
		speedOut[motorNum] = CurrentMotorSpeedMmSec;
	}
	lastCalcTimeUs[motorNum] = curentTimeUs;
	gd.speed[motorNum].lastCalcTimeUs = curentTimeUs;
	last_hall_cnt[motorNum] = currentHallCnt;
	return speedOut[motorNum];
}


float calcVlim()
{
	float Vlim_temp[2] = {0};
	int counter = 0;
	for (int motorNum = 0; motorNum < (MAX_NUM_OF_MOTORS - 1); motorNum++) // checking only the driving motors
	{
		if (gd.devicesOn.motorsOn[motorNum])
		{
			Vlim_temp[counter] = fabs(gd.speed[motorNum].totalLinearSpeedMmSecCorrected) - 0.9*gd.robot.max_linearSpeed;
			counter++;
		}
	}

	if (counter == 0)
	{
		return 0;
	}

	if (counter == 1)
	{
		if (Vlim_temp[0] <= 0)
		{
			return 0;
		}
		else
		{
			return Vlim_temp[0];
		}
	}

	if ((Vlim_temp[0] <= 0) && (Vlim_temp[1] <= 0))
	{
		return 0;
	}

	if (Vlim_temp[0] <= 0)
	{
		return Vlim_temp[1];
	}

	if (Vlim_temp[1] <= 0)
	{
		return Vlim_temp[0];
	}

	if (Vlim_temp[1] <= Vlim_temp[0])
	{
		return Vlim_temp[0];
	}

	if (Vlim_temp[1] > Vlim_temp[0])
	{
		return Vlim_temp[1];
	}

	return 0;
}


void LinearSpeedControlHandle(uint8_t motorNum, bool controlOn, float linearRef)
{
	SPIspeedContorlOut tempLinear;
	if (controlOn)
	{
		//		float totalLinearSpeedMmSecFromHall = 0.9 * gd.speed[motorNum].prevLinearSpeedFromHall + 0.1 * gd.speed[motorNum].linearSpeedFromHall;
		//		gd.speed[motorNum].prevLinearSpeedFromHall = gd.speed[motorNum].linearSpeedFromHall;
		float totalLinearSpeedMmSecFromHall = gd.speed[motorNum].linearSpeedFromHall;
		tempLinear = PISpeedControl(totalLinearSpeedMmSecFromHall, linearRef, gd.robot.linearSpeedKP, gd.robot.linearSpeedKI, &gd.speed[motorNum],motorNum);
		gd.speed[motorNum].Speed_I_correction = tempLinear.Speed_I_correction;
		gd.speed[motorNum].totalLinearSpeedMmSecCorrected = linearRef + tempLinear.Speedcorrection;
	}
	else
	{
		gd.speed[motorNum].totalLinearSpeedMmSecCorrected = linearRef;
	}
#ifdef SPEED_CONTROL_DBG
	mainDBG.speedControl[motorNum].speedFromHall[mainDBG.speedControl[motorNum].counter] = gd.speed[motorNum].linearSpeedFromHall;
#endif
	gd.speed[motorNum].linearSpeedFromHall = 0;
}

void changeMotorDriveControlType(EDeviceName motorName, EDriveControlType driveControlType)
{
	// TODO - need to define after bit, if all sensors are operating O.K. then "FULL_SPEED_CONTROL" is to be defined
	// 		  other wise a different control type need to be defined
	if ((motorName == D_NAME_DRIVE_MOTOR_LEFT) || (motorName == D_NAME_DRIVE_MOTOR_RIGHT))
	{
		gd.speed[D_NAME_DRIVE_MOTOR_LEFT].driveControlType = driveControlType;
		gd.speed[motorName].driveControlType = gd.speed[D_NAME_DRIVE_MOTOR_LEFT].driveControlType; // the two driving motors must be at the same drive control type
	}
	else
	{
		gd.speed[motorName].driveControlType = driveControlType;
	}

	if (motorName == D_NAME_DRIVE_MOTOR_LEFT || motorName == D_NAME_DRIVE_MOTOR_RIGHT)
	{
		enableNVICgpioIRQ();
	}
}

void disableNVICgpioIRQ()
{
	if (NVIC_GetEnableIRQ(GPIO_EVEN_IRQn))
	{
		NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
		GPIO_IntClear(0xFFFF);
		NVIC_DisableIRQ(GPIO_EVEN_IRQn);
		GPIO_IntClear(0xFFFF);
		NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	}

	if (NVIC_GetEnableIRQ(GPIO_ODD_IRQn))
	{
		NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
		GPIO_IntClear(0xFFFF);
		NVIC_DisableIRQ(GPIO_ODD_IRQn);
		GPIO_IntClear(0xFFFF);
		NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	}
}

void enableNVICgpioIRQ()
{
	if (!NVIC_GetEnableIRQ(GPIO_EVEN_IRQn))
	{
		NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
		GPIO_IntClear(0xFFFF);
		NVIC_EnableIRQ(GPIO_EVEN_IRQn);
		GPIO_IntClear(0xFFFF);
		NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	}

	if (!NVIC_GetEnableIRQ(GPIO_ODD_IRQn))
	{
		NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
		GPIO_IntClear(0xFFFF);
		NVIC_EnableIRQ(GPIO_ODD_IRQn);
		GPIO_IntClear(0xFFFF);
		NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	}
}


void resetMotorsData(int8_t motorNum)
{
	gd.speed[motorNum].Speed_I_correction = 0;
	gd.speed[motorNum].dLinearCommandFromRate = 0;
	gd.speed[motorNum].totalLinearSpeedMmSecCorrected = 0;
	gd.speed[motorNum].linearCommandSpeedToMotor = 0;
	gd.motor[motorNum].linearCommandClock = 0.0;
	gd.speed[motorNum].linearSpeedFromHall = 0;
	gd.speed[motorNum].linearCommandSpeedToMotorRef = 0;
	gd.speed[motorNum].prevLinearSpeedFromHall = 0;
	setMotorDriveState(motorNum);
}


void resetAllDriveMotorsData()
{
	for (uint8_t motorNum = 0; motorNum < MAX_NUM_OF_MOTORS - 1; motorNum++) // "MAX_NUM_OF_MOTORS - 1" for drive motors only
	{
		if (gd.devicesOn.motorsOn[motorNum])
		{
			resetMotorsData(motorNum);
		}
	}
}
