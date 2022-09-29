#include "counter.h"

// ------------------------------------------------------------- PRIVATE MACROS 

#define COUNTER_DUMMY 0

// ---------------------------------------------- PRIVATE FUNCTION DECLARATIONS 

//static void counter_write_command ( counter_t *ctx, uint8_t command );
//static uint8_t counter_read_register ( counter_t *ctx, uint8_t command );
//static void counter_write_data ( counter_t *ctx, uint8_t command, uint8_t *_data, uint8_t count );
//static void counter_read_data ( counter_t *ctx, uint8_t command, uint8_t *_data, uint8_t count );

// ------------------------------------------------ PUBLIC FUNCTION DEFINITIONS

//void counter_cfg_setup ( counter_cfg_t *cfg )
//{
//    // Communication gpio pins
//
//    cfg->sck = HAL_PIN_NC;
//    cfg->miso = HAL_PIN_NC;
//    cfg->mosi = HAL_PIN_NC;
//    cfg->cs = HAL_PIN_NC;
//
//    // Additional gpio pins
//
//    cfg->en = HAL_PIN_NC;
//    cfg->int_pin = HAL_PIN_NC;
//
//    cfg->spi_speed = 100000;
//    cfg->spi_mode = SPI_MASTER_MODE_0;
//    cfg->cs_polarity = SPI_MASTER_CHIP_SELECT_POLARITY_ACTIVE_LOW;
//}
//
//COUNTER_RETVAL counter_init ( counter_t *ctx, counter_cfg_t *cfg )
//{
//    spi_master_config_t spi_cfg;
//
//    spi_master_configure_default( &spi_cfg );
//    spi_cfg.speed     = cfg->spi_speed;
//    spi_cfg.sck       = cfg->sck;
//    spi_cfg.miso      = cfg->miso;
//    spi_cfg.mosi      = cfg->mosi;
//    spi_cfg.default_write_data = COUNTER_DUMMY;
//
//    digital_out_init( &ctx->cs, cfg->cs );
//    ctx->chip_select = cfg->cs;
//
//    if (  spi_master_open( &ctx->spi, &spi_cfg ) == SPI_MASTER_ERROR )
//    {
//        return COUNTER_INIT_ERROR;
//    }
//
//    spi_master_set_default_write_data( &ctx->spi, COUNTER_DUMMY );
//    spi_master_set_speed( &ctx->spi, cfg->spi_speed );
//    spi_master_set_mode( &ctx->spi, cfg->spi_mode );
//    spi_master_set_chip_select_polarity( cfg->cs_polarity );
//
//    // Output pins
//
//    digital_out_init( &ctx->en, cfg->en );
//    digital_out_high( &ctx->en );
//
//    // Input pins
//
//    digital_in_init( &ctx->int_pin, cfg->int_pin );
//
//    return COUNTER_OK;
//}

void counter_default_cfg ( void )
{
    uint8_t tmp[ 4 ] = { 0 };
    uint8_t buffer_size = counter_init_advanced( COUNTER_4X_QUAD | COUNTER_FREE_RUN | COUNTER_INDEX_DISABLED | COUNTER_FILTER_CLOCK_DIV1,
                                COUNTER_MODE_32 | COUNTER_ENABLE | COUNTER_FLAG_DISABLE );
    counter_write_dtr( buffer_size, tmp );
    counter_clear_cntr ( );
    counter_load_cntr( );
    counter_enable( );
}

//void counter_generic_transfer
//(
//    counter_t *ctx,
//    uint8_t *wr_buf,
//    uint16_t wr_len,
//    uint8_t *rd_buf,
//    uint16_t rd_len
//)
//{
//    spi_master_select_device( ctx->chip_select );
//    spi_master_write_then_read( &ctx->spi, wr_buf, wr_len, rd_buf, rd_len );
//    spi_master_deselect_device( ctx->chip_select );
//}

void counter_write_mdr0 ( uint8_t settings )
{
    counter_write_data( COUNTER_CMD_WR | COUNTER_MDR0, &settings, 1 );
}

void counter_write_mdr1 ( uint8_t settings )
{
    counter_write_data( COUNTER_CMD_WR | COUNTER_MDR1, &settings, 1 );
}

void counter_write_dtr ( uint8_t buffer_size, uint8_t* buffer )
{
    counter_write_data( COUNTER_CMD_WR | COUNTER_DTR, buffer, buffer_size );
}

void counter_load_cntr ( )
{
    counter_write_command( COUNTER_CMD_LOAD | COUNTER_CNTR );
}

void counter_load_otr ( )
{
    counter_write_command( COUNTER_CMD_LOAD | COUNTER_OTR );
}

uint8_t counter_read_mdr0 ( )
{
    uint8_t result;

    result = counter_read_register( COUNTER_CMD_RD | COUNTER_MDR0 );

    return result;
}

uint8_t counter_read_mdr1 ( )
{
    uint8_t result;

    result = counter_read_register( COUNTER_CMD_RD | COUNTER_MDR1 );

    return result;
}

int32_t counter_read_otr ( uint8_t buffer_size )
{
    uint8_t data_buf[ BUFFER_SIZE ];
    uint32_t result;

    counter_read_data( COUNTER_CMD_RD | COUNTER_OTR, data_buf );
    
    result = data_buf[ 0 ];
    
    for ( uint8_t cnt = 1; cnt < BUFFER_SIZE; cnt++ )
    {
        result <<= 8;
        result |= data_buf[ cnt ];
    }

    return result;
}

int32_t counter_read_cntr ( )
{
    uint8_t data_buf[ BUFFER_SIZE ];
    uint32_t result;

    counter_read_data( COUNTER_CMD_RD | COUNTER_CNTR, data_buf, BUFFER_SIZE );

    result = data_buf[ 0 ];
    
    for ( uint8_t cnt = 1; cnt < BUFFER_SIZE; cnt++ )
    {
        result <<= 8;
        result |= data_buf[ cnt ];
    }
    
    return result;
}

int32_t counter_read_dtr ( )
{
    uint8_t data_buf[ BUFFER_SIZE ];
    uint32_t result;

    counter_read_data( COUNTER_CMD_RD | COUNTER_DTR, data_buf );

    result = data_buf[ 0 ];
    
    for ( uint8_t cnt = 1; cnt < BUFFER_SIZE; cnt++ )
    {
        result <<= 8;
        result |= data_buf[ cnt ];
    }
    
    return result;
}

uint8_t counter_read_str ( )
{
    uint8_t result;

    result = counter_read_register( COUNTER_CMD_RD | COUNTER_STR );

    return result;
}

void counter_clear_mrd0 ( )
{
    counter_write_command( COUNTER_CMD_CLR | COUNTER_MDR0 );
}

void counter_clead_mrd1 ( )
{
    counter_write_command( COUNTER_CMD_CLR | COUNTER_MDR1 );
}

void counter_clear_cntr ( )
{
    counter_write_command( COUNTER_CMD_CLR | COUNTER_CNTR );
}

void counter_clear_str ( )
{
    counter_write_command( COUNTER_CMD_CLR | COUNTER_STR );
}

void counter_initialisation ( )
{
    uint8_t mdr0_set = COUNTER_4X_QUAD | COUNTER_FREE_RUN | COUNTER_INDEX_DISABLED
                       | COUNTER_FILTER_CLOCK_DIV1;
    uint8_t mdr1_set = COUNTER_MODE_32 | COUNTER_DISABLE | COUNTER_FLAG_DISABLE;

//    ctx->buffer_size = 4;
//    digital_out_high( &ctx->en );
    counter_write_mdr0( mdr0_set );
    counter_write_mdr1( mdr1_set );
}

uint8_t counter_init_advanced ( uint8_t mdr0_set, uint8_t mdr1_set )
{
  static uint8_t buffer_size;
//    digital_out_high( &ctx->en );
    counter_write_mdr0( mdr0_set );
    counter_write_mdr1( mdr1_set );

    switch ( mdr1_set & 0x03 )
    {
        case 0 :
        {
            buffer_size = 4;
            break;
        }
        case 1 :
        {
            buffer_size = 3;
            break;
        }
        case 2 :
        {
            buffer_size = 2;
            break;
        }
        case 3 :
        {
            buffer_size = 1;
            break;
        }
    }
    return buffer_size;
}

void counter_enable ( )
{
    uint8_t mdr1_temp = 0;

    mdr1_temp = counter_read_mdr1( ) & 0xFB;
    counter_write_mdr1( mdr1_temp );
}

void counter_disable ( )
{
    uint8_t mdr1_temp = 0;

    mdr1_temp = counter_read_mdr1( ) | 0x04;
    counter_write_mdr1( mdr1_temp );
}

//void counter_chip_enable ( counter_t *ctx )
//{
//    digital_out_high( &ctx->en );
//}

//void counter_chip_disable ( counter_t *ctx )
//{
//    digital_out_low( &ctx->en );
//}

//uint8_t get_int_state ( counter_t *ctx )
//{
//    return digital_in_read( &ctx->int_pin );
//}

// ----------------------------------------------- PRIVATE FUNCTION DEFINITIONS

static void counter_write_command ( uint8_t command )
{
    uint8_t temp[ 1 ];
    temp[ 0 ] = command;

//    spi_master_select_device( ctx->chip_select );
//    spi_master_write( &ctx->spi, &command, 1 );
//    spi_master_deselect_device( ctx->chip_select );
    SPIDRV_MTransmitB(SPI_HANDLE, &command, 1);
}

static uint8_t counter_read_register ( uint8_t command )
{   
    uint8_t tx_buff[ 1 ];
    uint8_t rx_buff[ 1 ];
    
    tx_buff[ 0 ] = command;

//    counter_generic_transfer ( ctx, &tx_buff, 1, rx_buff, 1 );
    
    SPIDRV_MTransferB(SPI_HANDLE, &tx_buff, &rx_buff, 1);
    return rx_buff[ 0 ];
}

static void counter_write_data (uint8_t command, uint8_t *data_buff, uint8_t count)
{
    uint8_t cmnd[ 1 ];
    uint8_t temp[ 4 ];
    
    cmnd[ 0 ] = command;
    memcpy( temp, data_buff, count );

//    spi_master_select_device( ctx->chip_select );
//    spi_master_write( &ctx->spi, &command, 1 );
//    spi_master_write( &ctx->spi, temp, count );
//    spi_master_deselect_device( ctx->chip_select );
    SPIDRV_MTransmitB(SPI_HANDLE, &command, 1);
    SPIDRV_MTransmitB(SPI_HANDLE, temp, count);
}

static void counter_read_data ( uint8_t command, uint8_t *data_buff )
{
    uint8_t tx_buff[ 1 ];
//    uint8_t rx_buff[ 4 ];
    
    tx_buff[ 0 ] = command;
    SPIDRV_MTransmitB(SPI_HANDLE, &tx_buff, 1);
    SPIDRV_MReceiveB(SPI_HANDLE, data_buff, BUFFER_SIZE);
//    SPIDRV_MTransferB(SPI_HANDLE, &tx_buff, data_buff, 1);
//    counter_generic_transfer ( ctx, &tx_buff, 1, rx_buff, count );
    
//    for ( uint8_t cnt = 0; cnt < count; cnt++ )
//    {
//        data_buff[ cnt ] = rx_buff[ cnt ];
//    }

}


// ------------------------------------------------------------------------- END
