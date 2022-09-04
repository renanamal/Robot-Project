#ifndef INC_GENERALPURPOSEFUNCTIONS_H_
#define INC_GENERALPURPOSEFUNCTIONS_H_

#include "ustimer.h"
#include "stdbool.h"
#include "rtcdriver.h"
#include <stddef.h>
#include "rail.h"

typedef struct SContinuousAverage
{
  float AverageData;
  float courentData;
  float N;
  bool reset;
}SContinuousAverage;

// functions protos
void delay_ms(uint32_t mDelay);
void delay_us(uint32_t uDelay);
void continuousAverage(SContinuousAverage * dataIn);
uint32_t getMillis(void);
uint32_t getSec(void);
RAIL_Time_t getuSec(void);

#endif /* INC_GENERALPURPOSEFUNCTIONS_H_ */
