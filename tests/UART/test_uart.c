#include <tests/UART/test_uart.h>
extern UARTDRV_Handle_t sl_uartdrv_usart_inst0_handle;

#pragma pack(1)
typedef struct DummyMsg
{
  uint8_t header;
  float angle;
  uint8_t footer;
}DummyMsg;


void send_test_data()
{
  static float angle = 0.0;
//  while(1)
//  {
    angle += 0.2;
    DummyMsg msg = {0x01,angle, 0x05};
    uint8_t data[sizeof(msg)] = {};
    memcpy(data, &msg, sizeof(msg));
    UARTDRV_Count_t count = sizeof(data);
    UARTDRV_Transmit(sl_uartdrv_usart_inst0_handle, data, count, callback_test);
//    delay_ms(20);
//  }
}

void callback_test(UARTDRV_Handle_t handle,
              Ecode_t transferStatus,
              uint8_t *data,
              UARTDRV_Count_t transferCount)
{
  (void)handle;
  (void)transferStatus;
  (void)data;
  (void)transferCount;
}
