#include "spidrv.h"
#include "sl_spidrv_instances.h"
#include "sl_assert.h"


#include "sl_spidrv_usart_encoder_counter_config.h"

SPIDRV_HandleData_t sl_spidrv_usart_encoder_counter_handle_data;
SPIDRV_Handle_t sl_spidrv_usart_encoder_counter_handle = &sl_spidrv_usart_encoder_counter_handle_data;

SPIDRV_Init_t sl_spidrv_usart_init_encoder_counter = {
  .port = SL_SPIDRV_USART_ENCODER_COUNTER_PERIPHERAL,
#if defined(_USART_ROUTELOC0_MASK)
  .portLocationTx = SL_SPIDRV_USART_ENCODER_COUNTER_TX_LOC,
  .portLocationRx = SL_SPIDRV_USART_ENCODER_COUNTER_RX_LOC,
  .portLocationClk = SL_SPIDRV_USART_ENCODER_COUNTER_CLK_LOC,
#if defined(SL_SPIDRV_USART_ENCODER_COUNTER_CS_LOC)
  .portLocationCs = SL_SPIDRV_USART_ENCODER_COUNTER_CS_LOC,
#endif
#elif defined(_GPIO_USART_ROUTEEN_MASK)
  .portTx = SL_SPIDRV_USART_ENCODER_COUNTER_TX_PORT,
  .portRx = SL_SPIDRV_USART_ENCODER_COUNTER_RX_PORT,
  .portClk = SL_SPIDRV_USART_ENCODER_COUNTER_CLK_PORT,
#if defined(SL_SPIDRV_USART_ENCODER_COUNTER_CS_PORT)
  .portCs = SL_SPIDRV_USART_ENCODER_COUNTER_CS_PORT,
#endif
  .pinTx = SL_SPIDRV_USART_ENCODER_COUNTER_TX_PIN,
  .pinRx = SL_SPIDRV_USART_ENCODER_COUNTER_RX_PIN,
  .pinClk = SL_SPIDRV_USART_ENCODER_COUNTER_CLK_PIN,
#if defined(SL_SPIDRV_USART_ENCODER_COUNTER_CS_PIN)
  .pinCs = SL_SPIDRV_USART_ENCODER_COUNTER_CS_PIN,
#endif
#else
  .portLocation = SL_SPIDRV_USART_ENCODER_COUNTER_ROUTE_LOC,
#endif
  .bitRate = SL_SPIDRV_USART_ENCODER_COUNTER_BITRATE,
  .frameLength = SL_SPIDRV_USART_ENCODER_COUNTER_FRAME_LENGTH,
  .dummyTxValue = 0,
  .type = SL_SPIDRV_USART_ENCODER_COUNTER_TYPE,
  .bitOrder = SL_SPIDRV_USART_ENCODER_COUNTER_BIT_ORDER,
  .clockMode = SL_SPIDRV_USART_ENCODER_COUNTER_CLOCK_MODE,
  .csControl = SL_SPIDRV_USART_ENCODER_COUNTER_CS_CONTROL,
  .slaveStartMode = SL_SPIDRV_USART_ENCODER_COUNTER_SLAVE_START_MODE,
};

void sl_spidrv_init_instances(void) {
#if !defined(SL_SPIDRV_EUSART_ENCODER_COUNTER_CS_PIN)
  EFM_ASSERT(sl_spidrv_usart_init_encoder_counter.csControl == spidrvCsControlAuto);
#endif
  SPIDRV_Init(sl_spidrv_usart_encoder_counter_handle, &sl_spidrv_usart_init_encoder_counter);
}
