#ifndef TESTS_UART_TEST_UART_H_
#define TESTS_UART_TEST_UART_H_

#include "../../src/generalPurposeFunctions.h"
#include "uartdrv.h"
#include "sl_uartdrv_instances.h"

void send_test_data();

void callback_test(UARTDRV_Handle_t handle,
              Ecode_t transferStatus,
              uint8_t *data,
              UARTDRV_Count_t transferCount);

#endif /* TESTS_UART_TEST_UART_H_ */
