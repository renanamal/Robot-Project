#ifndef SRC_SPI_HANDLE_H_
#define SRC_SPI_HANDLE_H_

#include <string.h>
#include <stdio.h>
#include "spidrv.h"
#include "sl_spidrv_instances.h"

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

// use SPI handle for EXP header (configured in project settings)
#define SPI_HANDLE                  sl_spidrv_encoderSPI_handle

// size of transmission and reception buffers
#define APP_BUFFER_SIZE             16

//counter_write_data
//counter_write_command
//counter_read_register
//counter_read_data
void transfer_callback(SPIDRV_HandleData_t *handle, Ecode_t transfer_status, int items_transferred);
void spi_master_write(void);
#endif /* SRC_SPI_HANDLE_H_ */
