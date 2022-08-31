#ifndef TESTS_TEST_GPIO_INT_H_
#define TESTS_TEST_GPIO_INT_H_

#include "em_gpio.h"
#include "gpiointerrupt.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define GPIO_IRQ_INPUT_PIN                     (8U)
#define GPIO_IRQ_INPUT_PORT                    (gpioPortD)

void test_gpio_callback(uint8_t pin);
void test_gpio_init(void);

#endif /* TESTS_TEST_GPIO_INT_H_ */
