#include "sl_emlib_gpio_simple_init.h"
#include "sl_emlib_gpio_init_F13_config.h"
#include "sl_emlib_gpio_init_F14_config.h"
#include "sl_emlib_gpio_init_F15_config.h"
#include "sl_emlib_gpio_init_I1_config.h"
#include "sl_emlib_gpio_init_I2_config.h"
#include "sl_emlib_gpio_init_I3_config.h"
#include "sl_emlib_gpio_init_PA6_config.h"
#include "sl_emlib_gpio_init_PA7_config.h"
#include "sl_emlib_gpio_init_PB9_config.h"
#include "sl_emlib_gpio_init_PC4_config.h"
#include "sl_emlib_gpio_init_PC5_config.h"
#include "sl_emlib_gpio_init_PD8_config.h"
#include "sl_emlib_gpio_init_PF9_config.h"
#include "em_gpio.h"
#include "em_cmu.h"

void sl_emlib_gpio_simple_init(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_F13_PORT,
                  SL_EMLIB_GPIO_INIT_F13_PIN,
                  SL_EMLIB_GPIO_INIT_F13_MODE,
                  SL_EMLIB_GPIO_INIT_F13_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_F14_PORT,
                  SL_EMLIB_GPIO_INIT_F14_PIN,
                  SL_EMLIB_GPIO_INIT_F14_MODE,
                  SL_EMLIB_GPIO_INIT_F14_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_F15_PORT,
                  SL_EMLIB_GPIO_INIT_F15_PIN,
                  SL_EMLIB_GPIO_INIT_F15_MODE,
                  SL_EMLIB_GPIO_INIT_F15_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_I1_PORT,
                  SL_EMLIB_GPIO_INIT_I1_PIN,
                  SL_EMLIB_GPIO_INIT_I1_MODE,
                  SL_EMLIB_GPIO_INIT_I1_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_I2_PORT,
                  SL_EMLIB_GPIO_INIT_I2_PIN,
                  SL_EMLIB_GPIO_INIT_I2_MODE,
                  SL_EMLIB_GPIO_INIT_I2_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_I3_PORT,
                  SL_EMLIB_GPIO_INIT_I3_PIN,
                  SL_EMLIB_GPIO_INIT_I3_MODE,
                  SL_EMLIB_GPIO_INIT_I3_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_PA6_PORT,
                  SL_EMLIB_GPIO_INIT_PA6_PIN,
                  SL_EMLIB_GPIO_INIT_PA6_MODE,
                  SL_EMLIB_GPIO_INIT_PA6_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_PA7_PORT,
                  SL_EMLIB_GPIO_INIT_PA7_PIN,
                  SL_EMLIB_GPIO_INIT_PA7_MODE,
                  SL_EMLIB_GPIO_INIT_PA7_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_PB9_PORT,
                  SL_EMLIB_GPIO_INIT_PB9_PIN,
                  SL_EMLIB_GPIO_INIT_PB9_MODE,
                  SL_EMLIB_GPIO_INIT_PB9_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_PC4_PORT,
                  SL_EMLIB_GPIO_INIT_PC4_PIN,
                  SL_EMLIB_GPIO_INIT_PC4_MODE,
                  SL_EMLIB_GPIO_INIT_PC4_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_PC5_PORT,
                  SL_EMLIB_GPIO_INIT_PC5_PIN,
                  SL_EMLIB_GPIO_INIT_PC5_MODE,
                  SL_EMLIB_GPIO_INIT_PC5_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_PD8_PORT,
                  SL_EMLIB_GPIO_INIT_PD8_PIN,
                  SL_EMLIB_GPIO_INIT_PD8_MODE,
                  SL_EMLIB_GPIO_INIT_PD8_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_PF9_PORT,
                  SL_EMLIB_GPIO_INIT_PF9_PIN,
                  SL_EMLIB_GPIO_INIT_PF9_MODE,
                  SL_EMLIB_GPIO_INIT_PF9_DOUT);
}
