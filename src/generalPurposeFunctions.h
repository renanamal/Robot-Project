#ifndef INC_GENERALPURPOSEFUNCTIONS_H_
#define INC_GENERALPURPOSEFUNCTIONS_H_

#include "stdbool.h"
#include "ustimer.h"

typedef struct SContinuousAverage
{
  float AverageData;
  float courentData;
  float N;
  bool reset;
}SContinuousAverage;

// functions protos
//void delay_ms(uint32_t mDelay);
//void delay_us(uint32_t uDelay);
void continuousAverage(SContinuousAverage * dataIn);

#endif /* INC_GENERALPURPOSEFUNCTIONS_H_ */
