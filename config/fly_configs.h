#ifndef FLY_CONFIG_H
#define FLY_CONFIG_H

// Note: change this to set the desired Output frequency in Hz
#define PWM_FREQ 40000

// Buffer size
#define BUFFER_SIZE 11

// Note: change this to change the duty cycles used in this example
static const uint16_t dutyCyclePercentages[BUFFER_SIZE] =
  {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};

// Buffer of duty cycle values for DMA transfer to CCVB
// Buffer is populated after TIMER is initialized and Top value is set
static uint16_t buffer[BUFFER_SIZE];

#endif
