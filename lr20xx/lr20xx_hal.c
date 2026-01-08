//#include <stddef.h>
#include <stdbool.h>
#include "lr20xx_hal.h"
#include "pinDefs_lr20xx.h"
#include "stm32f7xx_hal.h"
#include "pinDefs.h"
#include "main.h"	 // for delay_ticks
#include "lr20xx_system_types.h"

#define ASSERT_NSS		ASSERT_SX126X_NSS
#define UNASSERT_NSS	UNASSERT_SX126X_NSS

#define NO_CMD		0xffff

typedef enum
{
	RADIO_SLEEP,
	RADIO_AWAKE
} radio_mode_t;

static volatile radio_mode_t radio_mode = RADIO_AWAKE;
static uint8_t commands_after_reset = 2;  /* Skip status check for first 2 commands after reset */
lr20xx_stat_t stat;
uint16_t saved_command = NO_CMD;
uint16_t bad_command;

static const char *chip_mode_str()
{
	switch (stat.bits.chip_mode) {
		case LR20XX_SYSTEM_CHIP_MODE_SLEEP: return "SLEEP";
		case LR20XX_SYSTEM_CHIP_MODE_STBY_RC: return "STBY_RC";
		case LR20XX_SYSTEM_CHIP_MODE_STBY_XOSC: return "STBY_XOSC";
		case LR20XX_SYSTEM_CHIP_MODE_FS: return "FS";
		case LR20XX_SYSTEM_CHIP_MODE_RX: return "RX";
		case LR20XX_SYSTEM_CHIP_MODE_TX: return "TX";
		default: return "???";
	}
}

/* return true for error status */
static bool check_stat()
{
	bool ret = false;

	/* Skip status check for first 2 commands after reset:
	 * - Command 1 receives invalid 0x0000 status
	 * - Command 2 receives status from command 1 which may be unreliable */
	if (commands_after_reset > 0) {
		commands_after_reset--;
		return false;
	}

	if (stat.bits.command_status == LR20XX_SYSTEM_CMD_STATUS_PERR) {
		printf("CMD_STATUS_PERR cmd=%04x %s\n", saved_command, chip_mode_str());
		ret = true;
	} else if (stat.bits.command_status == LR20XX_SYSTEM_CMD_STATUS_FAIL) {
		printf("%04x CMD_STATUS_FAIL cmd=%04x, %s\n", stat.word, saved_command, chip_mode_str());
		ret = true;
	}
	if (ret)
		bad_command = saved_command;
	return ret;
}

static int lr20xx_hal_wait_on_busy( void )
{
	/* BUSY typically clears in ~1µs for normal commands, but some commands
	 * (calibration, etc.) can take up to 10ms. Tight poll for speed.
	 * At 216MHz, ~10 cycles/iteration, so 2M iterations ≈ 100ms timeout. */
	volatile uint32_t cnt = 0;
	while (BUSY) {
		if (++cnt > 2000000) {
			return -1;
		}
	}
	return 0;
}

static int lr20xx_hal_check_device_ready( void )
{
	if( radio_mode != RADIO_SLEEP )
	{
		if (lr20xx_hal_wait_on_busy( ) < 0) {
			printf("wait_on_busy fail\r\n");
			return -1;
		}
	}
	else
	{
		// Busy is HIGH in sleep mode, wake-up the device with a small glitch on NSS
		printf("wake,wait-on-busy\r\n");
		ASSERT_NSS;
		// wait for 1ms
		delay_ticks(1);
		UNASSERT_NSS;
		if (lr20xx_hal_wait_on_busy( ) < 0)
			printf("wake,wait_on_busy fail\r\n");
		radio_mode = RADIO_AWAKE;
	}
	return 0;
}

lr20xx_hal_status_t lr20xx_hal_read( const void* radio, const uint8_t* cbuffer, const uint16_t cbuffer_length,
                                     uint8_t* rbuffer, const uint16_t rbuffer_length )
{
	uint8_t dummy_bytes[2] = { 0x00, 0x00 };
	bool stat_fail = false;

	if (lr20xx_hal_check_device_ready( ) < 0) {
		printf("device not ready\r\n");
		return LR20XX_HAL_STATUS_ERROR;
	}

	// Put NSS low to start spi transaction
	ASSERT_NSS;
	for( uint16_t i = 0; i < cbuffer_length; i++ )
	{
		uint8_t in = spi_transfer( cbuffer[i] );
		if (i == 0) {
			stat.word = in;
		} else if (i == 1) {
			stat.word <<= 8;
			stat.word |= in;
			//printf("stat.word: %04x, hal_read %02x%02x\r\n", stat.word, cbuffer[0], cbuffer[1]);
			stat_fail = check_stat();
		}
	}
	saved_command = (cbuffer[0] << 8) + cbuffer[1];
	UNASSERT_NSS;

	if( rbuffer_length > 0 )
	{
		lr20xx_hal_wait_on_busy( );
		ASSERT_NSS;
		// Send dummy bytes
		for( uint16_t i = 0; i < sizeof( dummy_bytes ); i++ )
		{
			spi_transfer( dummy_bytes[i] );
		}

		for( uint16_t i = 0; i < rbuffer_length; i++ )
		{
			rbuffer[i] = spi_transfer( 0 );
		}
		// Put NSS high as the spi transaction is finished
		UNASSERT_NSS;
	}

	if (stat_fail) {
		return LR20XX_HAL_STATUS_ERROR;
	} else
		return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )
{
	bool stat_fail = false;

	if (lr20xx_hal_check_device_ready( ) < 0) {
		printf("device not ready\r\n");
		return LR20XX_HAL_STATUS_ERROR;
	}

	ASSERT_NSS;
	for( uint16_t i = 0; i < command_length; i++ )
	{
		uint8_t in = spi_transfer( command[i] );
		if (i == 0) {
			stat.word = in;
		} else if (i == 1) {
			stat.word <<= 8;
			stat.word |= in;
			//printf("stat.word: %04x, hal_write %02x%02x\r\n", stat.word, command[0], command[1]);
			stat_fail = check_stat();
		}
	}
	for( uint16_t i = 0; i < data_length; i++ )
	{
		spi_transfer( data[i] );
	}
	UNASSERT_NSS;

	saved_command = (command[0] << 8) + command[1];

	if (stat_fail)
		return LR20XX_HAL_STATUS_ERROR;
	return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_direct_read_fifo( const void* context, const uint8_t* command,
                                                 const uint16_t command_length, uint8_t* data,
                                                 const uint16_t data_length )
{
	bool stat_fail = false;

	if (lr20xx_hal_check_device_ready( ) < 0) {
		printf("device not ready\r\n");
		return LR20XX_HAL_STATUS_ERROR;
	}

	ASSERT_NSS;
	for( uint16_t i = 0; i < command_length; i++ )
	{
		uint8_t in = spi_transfer( command[i] );
		if (i == 0) {
			stat.word = in;
		} else if (i == 1) {
			stat.word <<= 8;
			stat.word |= in;
			//printf("stat.word: %04x, read_fifo %02x%02x\r\n", stat.word, command[0], command[1]);
			stat_fail = check_stat();
		}
	}
	saved_command = (command[0] << 8) + command[1];

	for( uint16_t i = 0; i < data_length; i++ )
	{
		data[i] = spi_transfer( 0 );
	}
	UNASSERT_NSS;

	if (stat_fail)
		return LR20XX_HAL_STATUS_ERROR;
	return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_direct_read( const void* context, uint8_t* data, const uint16_t data_length )
{
	if (lr20xx_hal_wait_on_busy( ) < 0) {
		return LR20XX_HAL_STATUS_ERROR;
	}

	ASSERT_NSS;
	for( uint16_t i = 0; i < data_length; i++ )
	{
		data[i] = spi_transfer( 0 );
	}
	UNASSERT_NSS;

	return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_reset( const void* radio )
{
	HAL_GPIO_WritePin(NRST_PORT, NRST_PIN, GPIO_PIN_RESET);
	// wait for 1ms
	delay_ticks(1);
	HAL_GPIO_WritePin(NRST_PORT, NRST_PIN, GPIO_PIN_SET);

	/* Skip status check for first 2 commands after reset */
	commands_after_reset = 2;

	return LR20XX_HAL_STATUS_OK;
}
