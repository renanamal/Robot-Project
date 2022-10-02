#include "test_spi.h"
#include "generalPurposeFunctions.h"

//static int32_t count = 5;
//static int32_t otr = 8;
//static uint8_t mdr0;
//static uint8_t mdr1;
void test_counter_spi()
{
//  counter_default_cfg( );
//  delay_ms(300);
//
//  mdr0 = counter_read_mdr0();
//  mdr1 = counter_read_mdr1();
//  count = counter_read_cntr( );
//  otr = counter_read_otr();
  uint8_t tx_buff[ 1 ];
  uint8_t rx_buff[ 4 ];

  tx_buff[ 0 ] = 0xAA;
  while(true)
  {
      Ecode_t error = SPIDRV_MTransferB(SPI_HANDLE, (void *)tx_buff, (void *)rx_buff, 1);
      if(error != ECODE_EMDRV_SPIDRV_OK)
      {
          ERROR_BREAK
      }
      delay_ms(1000);
  }
}
