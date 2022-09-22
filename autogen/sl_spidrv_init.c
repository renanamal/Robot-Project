#include "spidrv.h"
#include "sl_spidrv_instances.h"
#include "sl_assert.h"


#include "sl_spidrv_encoderSPI_config.h"

SPIDRV_HandleData_t sl_spidrv_encoderSPI_handle_data;
SPIDRV_Handle_t sl_spidrv_encoderSPI_handle = &sl_spidrv_encoderSPI_handle_data;

SPIDRV_Init_t sl_spidrv_init_encoderSPI = {
  .port = SL_SPIDRV_ENCODERSPI_PERIPHERAL,
#if defined(_USART_ROUTELOC0_MASK)
  .portLocationTx = SL_SPIDRV_ENCODERSPI_TX_LOC,
  .portLocationRx = SL_SPIDRV_ENCODERSPI_RX_LOC,
  .portLocationClk = SL_SPIDRV_ENCODERSPI_CLK_LOC,
#if defined(SL_SPIDRV_ENCODERSPI_CS_LOC)
  .portLocationCs = SL_SPIDRV_ENCODERSPI_CS_LOC,
#endif
#elif defined(_GPIO_USART_ROUTEEN_MASK)
  .portTx = SL_SPIDRV_ENCODERSPI_TX_PORT,
  .portRx = SL_SPIDRV_ENCODERSPI_RX_PORT,
  .portClk = SL_SPIDRV_ENCODERSPI_CLK_PORT,
#if defined(SL_SPIDRV_ENCODERSPI_CS_PORT)
  .portCs = SL_SPIDRV_ENCODERSPI_CS_PORT,
#endif
  .pinTx = SL_SPIDRV_ENCODERSPI_TX_PIN,
  .pinRx = SL_SPIDRV_ENCODERSPI_RX_PIN,
  .pinClk = SL_SPIDRV_ENCODERSPI_CLK_PIN,
#if defined(SL_SPIDRV_ENCODERSPI_CS_PIN)
  .pinCs = SL_SPIDRV_ENCODERSPI_CS_PIN,
#endif
#else
  .portLocation = SL_SPIDRV_ENCODERSPI_ROUTE_LOC,
#endif
  .bitRate = SL_SPIDRV_ENCODERSPI_BITRATE,
  .frameLength = SL_SPIDRV_ENCODERSPI_FRAME_LENGTH,
  .dummyTxValue = 0,
  .type = SL_SPIDRV_ENCODERSPI_TYPE,
  .bitOrder = SL_SPIDRV_ENCODERSPI_BIT_ORDER,
  .clockMode = SL_SPIDRV_ENCODERSPI_CLOCK_MODE,
  .csControl = SL_SPIDRV_ENCODERSPI_CS_CONTROL,
  .slaveStartMode = SL_SPIDRV_ENCODERSPI_SLAVE_START_MODE,
};

void sl_spidrv_init_instances(void) {
  SPIDRV_Init(sl_spidrv_encoderSPI_handle, &sl_spidrv_init_encoderSPI);
}
