#include "metryHandle.h"
extern UARTDRV_Handle_t sl_uartdrv_usart_inst0_handle;

void sendMetry(MetryDB* msg)
{
  uint8_t data[sizeof(MetryDB)] = {};
  memcpy(data, msg, sizeof(MetryDB));
  UARTDRV_Count_t count = sizeof(data);
  UARTDRV_TransmitB(sl_uartdrv_usart_inst0_handle, data, count);
}


