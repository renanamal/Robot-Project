#include "spi_handle.h"


/*******************************************************************************
 ***************************  LOCAL VARIABLES   ********************************
 ******************************************************************************/

// Flag to signal that transfer is complete
static volatile bool transfer_complete = false;

// Data counter
static int counter = 0;

// Transmission and reception buffers
static char rx_buffer[APP_BUFFER_SIZE];
static char tx_buffer[APP_BUFFER_SIZE];


/*******************************************************************************
 *********************   LOCAL FUNCTION PROTOTYPES   ***************************
 ******************************************************************************/

// Callback fired when data is transmitted
void transfer_callback(SPIDRV_HandleData_t *handle,
                       Ecode_t transfer_status,
                       int items_transferred)
{
  (void)&handle;
  (void)items_transferred;

  // Post semaphore to signal to application
  // task that transfer is successful
  if (transfer_status == ECODE_EMDRV_SPIDRV_OK) {
    transfer_complete = true;
  }
}


void spi_master_write(uint8_t command, uint8_t *data_buff, uint8_t count)
{
  Ecode_t ecode;

  // Delay to allow slave to start
  sl_sleeptimer_delay_millisecond(1);

//  sprintf(tx_buffer, "ping %03d", counter);
//  counter++;
//  printf("Sending %s to slave...\r\n", tx_buffer);

  transfer_complete = false;

  // Non-blocking data transfer to slave. When complete, rx buffer
  // will be filled.
  ecode = SPIDRV_MTransfer(SPI_HANDLE, tx_buffer, rx_buffer, counter, transfer_callback);
  EFM_ASSERT(ecode == ECODE_OK);

  // wait for transfer to complete
  while (!transfer_complete) ;

  printf("Got message from slave: %s\r\n", rx_buffer);
}
