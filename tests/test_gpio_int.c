#include "test_gpio_int.h"

void test_gpio_callback(uint8_t pin)
{
  static int counter = 0;
  (void) pin;
  counter++;
}

void test_gpio_init(void)
{
  // Button Interrupt Config
  GPIOINT_Init();
  GPIOINT_CallbackRegister(GPIO_IRQ_INPUT_PIN, test_gpio_callback);
  GPIO_IntConfig(GPIO_IRQ_INPUT_PORT, GPIO_IRQ_INPUT_PIN, true, true, true);
}
