#include "sl_emlib_gpio_simple_init.h"
#include "sl_emlib_gpio_init_PA6_config.h"
#include "sl_emlib_gpio_init_PA7_config.h"
#include "sl_emlib_gpio_init_PA8_config.h"
#include "sl_emlib_gpio_init_PA9_config.h"
#include "sl_emlib_gpio_init_PB6_config.h"
#include "sl_emlib_gpio_init_PB7_config.h"
#include "sl_emlib_gpio_init_PB8_config.h"
#include "sl_emlib_gpio_init_PB9_config.h"
#include "sl_emlib_gpio_init_PC11_config.h"
#include "sl_emlib_gpio_init_PC4_config.h"
#include "sl_emlib_gpio_init_PC5_config.h"
#include "sl_emlib_gpio_init_PD8_config.h"
#include "em_gpio.h"
#include "em_cmu.h"

void sl_emlib_gpio_simple_init(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_PA6_PORT,
                  SL_EMLIB_GPIO_INIT_PA6_PIN,
                  SL_EMLIB_GPIO_INIT_PA6_MODE,
                  SL_EMLIB_GPIO_INIT_PA6_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_PA7_PORT,
                  SL_EMLIB_GPIO_INIT_PA7_PIN,
                  SL_EMLIB_GPIO_INIT_PA7_MODE,
                  SL_EMLIB_GPIO_INIT_PA7_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_PA8_PORT,
                  SL_EMLIB_GPIO_INIT_PA8_PIN,
                  SL_EMLIB_GPIO_INIT_PA8_MODE,
                  SL_EMLIB_GPIO_INIT_PA8_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_PA9_PORT,
                  SL_EMLIB_GPIO_INIT_PA9_PIN,
                  SL_EMLIB_GPIO_INIT_PA9_MODE,
                  SL_EMLIB_GPIO_INIT_PA9_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_PB6_PORT,
                  SL_EMLIB_GPIO_INIT_PB6_PIN,
                  SL_EMLIB_GPIO_INIT_PB6_MODE,
                  SL_EMLIB_GPIO_INIT_PB6_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_PB7_PORT,
                  SL_EMLIB_GPIO_INIT_PB7_PIN,
                  SL_EMLIB_GPIO_INIT_PB7_MODE,
                  SL_EMLIB_GPIO_INIT_PB7_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_PB8_PORT,
                  SL_EMLIB_GPIO_INIT_PB8_PIN,
                  SL_EMLIB_GPIO_INIT_PB8_MODE,
                  SL_EMLIB_GPIO_INIT_PB8_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_PB9_PORT,
                  SL_EMLIB_GPIO_INIT_PB9_PIN,
                  SL_EMLIB_GPIO_INIT_PB9_MODE,
                  SL_EMLIB_GPIO_INIT_PB9_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_PC11_PORT,
                  SL_EMLIB_GPIO_INIT_PC11_PIN,
                  SL_EMLIB_GPIO_INIT_PC11_MODE,
                  SL_EMLIB_GPIO_INIT_PC11_DOUT);

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
}
