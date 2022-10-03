#include "test_spi.h"
#include "generalPurposeFunctions.h"

static int32_t count = 5;
static int32_t otr = 8;
static uint8_t mdr0;
static uint8_t mdr1;
static uint8_t str;

void test_arduino_spi()
{
  while(true)
  {
      uint8_t data_buff[ 4 ] = {0};
      uint8_t msg[3] = {0x01,0x02,0x03};
      counter_write_data(0xAA, msg, 3);
      counter_read_data( 0xAA, data_buff, 4 );
      delay_ms(1000);
  }
}

void test_counter_spi()
{
    counter_default_cfg( );
    delay_ms(300);

    str = counter_read_str();
    mdr0 = counter_read_mdr0();
    mdr1 = counter_read_mdr1();
    count = counter_read_cntr( );
    otr = counter_read_otr();
}
