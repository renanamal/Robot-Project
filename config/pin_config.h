#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// $[ACMP0]
// [ACMP0]$

// $[ACMP1]
// [ACMP1]$

// $[ADC0]
// [ADC0]$

// $[CMU]
// [CMU]$

// $[DBG]
// DBG SWV on PF2
#define DBG_SWV_PORT                             gpioPortF
#define DBG_SWV_PIN                              2
#define DBG_SWV_LOC                              0

// [DBG]$

// $[ETM]
// [ETM]$

// $[PTI]
// [PTI]$

// $[GPIO]
// [GPIO]$

// $[I2C0]
// [I2C0]$

// $[I2C1]
// [I2C1]$

// $[IDAC0]
// [IDAC0]$

// $[LESENSE]
// [LESENSE]$

// $[LETIMER0]
// [LETIMER0]$

// $[LEUART0]
// [LEUART0]$

// $[LFXO]
// [LFXO]$

// $[MODEM]
// [MODEM]$

// $[PCNT0]
// [PCNT0]$

// $[PCNT1]
// [PCNT1]$

// $[PCNT2]
// [PCNT2]$

// $[PRS.CH0]
// [PRS.CH0]$

// $[PRS.CH1]
// [PRS.CH1]$

// $[PRS.CH2]
// [PRS.CH2]$

// $[PRS.CH3]
// [PRS.CH3]$

// $[PRS.CH4]
// [PRS.CH4]$

// $[PRS.CH5]
// [PRS.CH5]$

// $[PRS.CH6]
// [PRS.CH6]$

// $[PRS.CH7]
// [PRS.CH7]$

// $[PRS.CH8]
// [PRS.CH8]$

// $[PRS.CH9]
// [PRS.CH9]$

// $[PRS.CH10]
// [PRS.CH10]$

// $[PRS.CH11]
// [PRS.CH11]$

// $[TIMER0]
// TIMER0 CC0 on PD9
#define TIMER0_CC0_PORT                          gpioPortD
#define TIMER0_CC0_PIN                           9
#define TIMER0_CC0_LOC                           17

// TIMER0 CC1 on PD10
#define TIMER0_CC1_PORT                          gpioPortD
#define TIMER0_CC1_PIN                           10
#define TIMER0_CC1_LOC                           17

// TIMER0 CC2 on PD11
#define TIMER0_CC2_PORT                          gpioPortD
#define TIMER0_CC2_PIN                           11
#define TIMER0_CC2_LOC                           17

// [TIMER0]$

// $[TIMER1]
// TIMER1 CC0 on PD12
#define TIMER1_CC0_PORT                          gpioPortD
#define TIMER1_CC0_PIN                           12
#define TIMER1_CC0_LOC                           20

// [TIMER1]$

// $[USART0]
// [USART0]$

// $[USART1]
// [USART1]$

// $[USART2]
// [USART2]$

// $[USART3]
// [USART3]$

// $[VDAC0]
// [VDAC0]$

// $[WTIMER0]
// [WTIMER0]$

// $[WTIMER1]
// [WTIMER1]$

// $[CUSTOM_PIN_NAME]
#define motor 1 Hall V_PORT                      gpioPortA
#define motor 1 Hall V_PIN                       6

#define motor 1 Hall W_PORT                      gpioPortA
#define motor 1 Hall W_PIN                       7

#define motor 2 encoder A_PORT                   gpioPortB
#define motor 2 encoder A_PIN                    9

#define motor 2 encoder B_PORT                   gpioPortC
#define motor 2 encoder B_PIN                    4

#define motor 2 encoder C_PORT                   gpioPortC
#define motor 2 encoder C_PIN                    5

#define motor 1 Hall U_PORT                      gpioPortD
#define motor 1 Hall U_PIN                       8

#define motor1 U_PORT                            gpioPortD
#define motor1 U_PIN                             9

#define motor1 V_PORT                            gpioPortD
#define motor1 V_PIN                             10

#define motor1 W_PORT                            gpioPortD
#define motor1 W_PIN                             11

#define motor2 U_PORT                            gpioPortD
#define motor2 U_PIN                             12

#define motor 2 Hall U_PORT                      gpioPortF
#define motor 2 Hall U_PIN                       13

#define motor 2 Hall W_PORT                      gpioPortF
#define motor 2 Hall W_PIN                       14

#define motor 2 Hall V_PORT                      gpioPortF
#define motor 2 Hall V_PIN                       15

#define motor 1 encoder A_PORT                   gpioPortI
#define motor 1 encoder A_PIN                    1

#define motor 1 encoder B_PORT                   gpioPortI
#define motor 1 encoder B_PIN                    2

#define motor 1 encoder I_PORT                   gpioPortI
#define motor 1 encoder I_PIN                    3

// [CUSTOM_PIN_NAME]$

#endif // PIN_CONFIG_H

