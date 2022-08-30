#include "generalPurposeFunctions.h"


void delay_ms(uint32_t mDelay)
{
  USTIMER_Delay(mDelay*1000.0);
}

void delay_us(uint32_t uDelay)
{
  USTIMER_Delay(uDelay);
}

void continuousAverage(SContinuousAverage * dataIn)
{
	if (dataIn->reset)
	{
		dataIn->N = 0;
		dataIn->AverageData = 0;
		dataIn->courentData = 0;
		dataIn->reset = false;
	}
	else
	{
		++dataIn->N;
		dataIn->AverageData = (1.0/dataIn->N) * dataIn->courentData + ((dataIn->N - 1)/ dataIn->N)*dataIn->AverageData;
	}
}


