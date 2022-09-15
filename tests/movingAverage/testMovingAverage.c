#include "testMovingAverage.h"

void test_moving_average()
{
  SMovingAverage  speedAverage;
  speedAverage.reset = true;
  movingAverage(&speedAverage);
  for(int i=0; i<20; i++)
  {
      speedAverage.currentData = i;
      movingAverage(&speedAverage);
  }
}
