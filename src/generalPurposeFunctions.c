#include "generalPurposeFunctions.h"
#include "rtcdriver.h"
#include <stddef.h>
#include "rail.h"

void delay_ms(uint32_t mDelay)
{
  USTIMER_Delay(mDelay*1000.0);
}

void delay_us(uint32_t uDelay)
{
  USTIMER_Delay(uDelay);
}

/* The Cortex-M33 has a faster execution of the hw loop
 * with the same arm instructions. */
#if defined(__CORTEX_M) && (__CORTEX_M == 33U)
  #define HW_LOOP_CYCLE  3
#else
  #define HW_LOOP_CYCLE  4
#endif

RAIL_Time_t getuSec(void)
{
  return RAIL_GetTime();
}

uint32_t getMillis(void)
{
  return RTCDRV_TicksToMsec(RTCDRV_GetWallClockTicks32());
}


uint32_t getSec(void)
{
  return RTCDRV_TicksToSec(RTCDRV_GetWallClockTicks32());
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


