#include <stdio.h>
#include "radio.h"
#include "stm32f7xx_hal.h"
#include "lr20xx.h"
#include "pinDefs_lr20xx.h"
#include "lr20xx_system.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_radio_lora.h"
#include "lr20xx_radio_fifo.h"
#include "main.h" // for delay_ticks()

/* Uncomment to enable TCXO instead of crystal oscillator */
// #define LR20XX_USE_TCXO
#ifdef LR20XX_USE_TCXO
#define LR20XX_TCXO_SUPPLY          LR20XX_SYSTEM_TCXO_CTRL_3_0V
#define LR20XX_TCXO_STARTUP_TICKS   300  /* 300 x 30.52µs = ~9.2ms */
#endif

/* LoRa sync word: 0x12 = Private Network, 0x34 = Public Network (LoRaWAN) */
#define LR20XX_LORA_SYNCWORD        0x12

static const RadioEvents_t* RadioEvents;

/* Static packet params - set by LoRaPacketConfig_lr20xx(), used by Send_lr20xx() */
static lr20xx_radio_lora_pkt_params_t lora_pkt_params;

/* Static modulation params - set by LoRaModemConfig_lr20xx(), used by is_bw_at_*(), is_sf_at_*() */
static lr20xx_radio_lora_mod_params_t lora_mod_params;

void Radio_txDoneBottom()
{
    if (RadioEvents->TxDone_botHalf)
        RadioEvents->TxDone_botHalf();
}

void Radio_rx_done(uint8_t size, float rssi, float snr)
{
    RadioEvents->RxDone(size, rssi, snr);
}

void Radio_timeout_callback(bool tx)
{
    if (!tx) {
        if (RadioEvents->RxTimeout)
            RadioEvents->RxTimeout();
#ifdef RX_INDICATION
        RX_INDICATION = 0;
#endif
    } // else TODO tx timeout
}

void Radio_chipModeChange()
{
    /* indicate radio mode with LEDs, if desired */
}

void Radio_dio8_top_half()
{
    if (RadioEvents->DioPin_top_half) {
        RadioEvents->DioPin_top_half();
	}

	// probably use lr20xx_stat_t stat.bits.chip_mode from lr20xx_hal.c to get chip mode, LR20XX_SYSTEM_CHIP_MODE_TX
	if (LR20xx_chipMode == LR20XX_SYSTEM_CHIP_MODE_TX) {
        /* TxDone handling requires low latency, but this could be other interrupt during TX such as fifo-irq */
        if (RadioEvents->TxDone_topHalf) {
            RadioEvents->TxDone_topHalf();
        } 
	}
}

static int lr20xx_hal_wait_on_busy( unsigned cnts )
{
	unsigned cnt = 0;
	while (BUSY) {
		delay_ticks(1);
		if (++cnt > cnts) {
			return -1;
		}
	}
	return 0;
}

static void Init_lr20xx(const RadioEvents_t* e)
{
	bool okay;

	// pin initializtion require prior to using reset pin
    init_lr20xx();

	printf("Init_lr20xx\r\n");
	do {
		printf("radio reset\r\n");
		lr20xx_system_reset(NULL);
		okay = lr20xx_hal_wait_on_busy(300) == 0;
		if (!okay) {
			printf("hw_reset busy stuck hi\r\n");
		}
	} while (!okay);
	printf("radio reset ok\r\n");

    LR20xx_txDone = Radio_txDoneBottom;
    LR20xx_rxDone = Radio_rx_done;
    LR20xx_timeout = Radio_timeout_callback;
    LR20xx_chipModeChange = Radio_chipModeChange;
    LR20xx_dio8_topHalf = Radio_dio8_top_half;
    lorahal.irqTopHalf = LR20xx_dio8_topHalf;
 
    RadioEvents = e;

	/* After reset, the first command wakes the device but may not execute properly.
	 * Send GetVersion first as a wake-up command (will return v0.0), then
	 * configure regulator mode, then verify with another GetVersion. */
	{
		lr20xx_system_version_t version;
		lr20xx_system_get_version(NULL, &version);  /* wake-up command, ignore result */
	}

	/* Configure regulator mode to DC-DC for better power efficiency */
	ASSERT_LR20XX_RC( lr20xx_system_set_reg_mode(NULL, LR20XX_SYSTEM_REG_MODE_DCDC) );

	{
		lr20xx_status_t status;
		lr20xx_system_version_t version;
		delay_ticks(10);
		do {
			status = lr20xx_system_get_version(NULL, &version);
			if (status != LR20XX_STATUS_OK) {
				printf("get version fail\r\n");
				delay_ticks(50);
			} else if (version.major == 0 && version.minor == 0) {
				printf("version 0.0, unexpected\r\n");
				status = LR20XX_STATUS_ERROR;
				delay_ticks(10);
			}
		} while (status != LR20XX_STATUS_OK);
		printf("LR20xx v%u.%u\r\n", version.major, version.minor);
	}

#ifdef LR20XX_USE_TCXO
	/* Configure TCXO if enabled */
	ASSERT_LR20XX_RC( lr20xx_system_set_tcxo_mode(NULL, LR20XX_TCXO_SUPPLY, LR20XX_TCXO_STARTUP_TICKS) );
#endif

	/* Configure low frequency clock source (internal RC 32kHz) */
	ASSERT_LR20XX_RC( lr20xx_system_cfg_lfclk(NULL, LR20XX_SYSTEM_LFCLK_RC) );

	ASSERT_LR20XX_RC( lr20xx_radio_common_set_pkt_type(NULL, LR20XX_RADIO_COMMON_PKT_TYPE_LORA) );

	/* Set LoRa sync word */
	ASSERT_LR20XX_RC( lr20xx_radio_lora_set_syncword(NULL, LR20XX_LORA_SYNCWORD) );

	/* Initialize default LoRa packet params */
	lora_pkt_params.preamble_len_in_symb = 8;
	lora_pkt_params.pkt_mode = LR20XX_RADIO_LORA_PKT_EXPLICIT;
	lora_pkt_params.pld_len_in_bytes = 0;
	lora_pkt_params.crc = LR20XX_RADIO_LORA_CRC_ENABLED;
	lora_pkt_params.iq = LR20XX_RADIO_LORA_IQ_STANDARD;

	/* Configure PA for LF path (sub-GHz) */
	{
		lr20xx_radio_common_pa_cfg_t pa_cfg = {
			.pa_sel          = LR20XX_RADIO_COMMON_PA_SEL_LF,
			.pa_lf_mode      = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
			.pa_lf_duty_cycle = 6,
			.pa_lf_slices     = 7,
			.pa_hf_duty_cycle = 16,  // unused for LF, but set to default
		};
		ASSERT_LR20XX_RC( lr20xx_radio_common_set_pa_cfg(NULL, &pa_cfg) );
	}

	/* Configure RX path for LF with boost mode */
	ASSERT_LR20XX_RC( lr20xx_radio_common_set_rx_path(NULL, LR20XX_RADIO_COMMON_RX_PATH_LF,
	                                                   LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_7) );

	/* Set fallback mode after TX/RX to standby RC */
	ASSERT_LR20XX_RC( lr20xx_radio_common_set_rx_tx_fallback_mode(NULL, LR20XX_RADIO_FALLBACK_STDBY_RC) );

	/* Configure all DIO pins explicitly
	 * Options for dio_function:
	 *   LR20XX_SYSTEM_DIO_FUNC_NONE       - Not used
	 *   LR20XX_SYSTEM_DIO_FUNC_IRQ        - Interrupt output
	 *   LR20XX_SYSTEM_DIO_FUNC_RF_SWITCH  - RF switch control
	 *   LR20XX_SYSTEM_DIO_FUNC_GPIO_LOW   - GPIO output low
	 *   LR20XX_SYSTEM_DIO_FUNC_GPIO_HIGH  - GPIO output high
	 *   LR20XX_SYSTEM_DIO_FUNC_HF_CLK_OUT - HF clock output (DIO7-11 only)
	 *   LR20XX_SYSTEM_DIO_FUNC_LF_CLK_OUT - LF clock output (DIO7-11 only)
	 *   LR20XX_SYSTEM_DIO_FUNC_TX_TRIGGER - TX trigger input
	 *   LR20XX_SYSTEM_DIO_FUNC_RX_TRIGGER - RX trigger input
	 */
	ASSERT_LR20XX_RC( lr20xx_system_set_dio_function(NULL, LR20XX_SYSTEM_DIO_5,
	                                                  LR20XX_SYSTEM_DIO_FUNC_NONE,
	                                                  LR20XX_SYSTEM_DIO_DRIVE_NONE) );
	ASSERT_LR20XX_RC( lr20xx_system_set_dio_function(NULL, LR20XX_SYSTEM_DIO_6,
	                                                  LR20XX_SYSTEM_DIO_FUNC_NONE,
	                                                  LR20XX_SYSTEM_DIO_DRIVE_NONE) );
	ASSERT_LR20XX_RC( lr20xx_system_set_dio_function(NULL, LR20XX_SYSTEM_DIO_7,
	                                                  LR20XX_SYSTEM_DIO_FUNC_NONE,
	                                                  LR20XX_SYSTEM_DIO_DRIVE_NONE) );
	ASSERT_LR20XX_RC( lr20xx_system_set_dio_function(NULL, LR20XX_SYSTEM_DIO_8,
	                                                  LR20XX_SYSTEM_DIO_FUNC_IRQ,
	                                                  LR20XX_SYSTEM_DIO_DRIVE_NONE) );
	ASSERT_LR20XX_RC( lr20xx_system_set_dio_function(NULL, LR20XX_SYSTEM_DIO_9,
	                                                  LR20XX_SYSTEM_DIO_FUNC_NONE,
	                                                  LR20XX_SYSTEM_DIO_DRIVE_NONE) );
	ASSERT_LR20XX_RC( lr20xx_system_set_dio_function(NULL, LR20XX_SYSTEM_DIO_10,
	                                                  LR20XX_SYSTEM_DIO_FUNC_NONE,
	                                                  LR20XX_SYSTEM_DIO_DRIVE_NONE) );
	ASSERT_LR20XX_RC( lr20xx_system_set_dio_function(NULL, LR20XX_SYSTEM_DIO_11,
	                                                  LR20XX_SYSTEM_DIO_FUNC_NONE,
	                                                  LR20XX_SYSTEM_DIO_DRIVE_NONE) );

	/* Clear any pending IRQs and configure IRQ mask for DIO8 */
	ASSERT_LR20XX_RC( lr20xx_system_clear_irq_status(NULL, LR20XX_SYSTEM_IRQ_ALL_MASK) );

	/* Enable TX done, RX done, timeout, and error interrupts on DIO8 */
	{
		lr20xx_system_irq_mask_t irq_mask = LR20XX_SYSTEM_IRQ_TX_DONE |
		                                    LR20XX_SYSTEM_IRQ_RX_DONE |
		                                    LR20XX_SYSTEM_IRQ_TIMEOUT |
		                                    LR20XX_SYSTEM_IRQ_ERROR;
		ASSERT_LR20XX_RC( lr20xx_system_set_dio_irq_cfg(NULL, LR20XX_SYSTEM_DIO_8, irq_mask) );
	}

	{
		lr20xx_system_version_t version;
		ASSERT_LR20XX_RC(lr20xx_system_get_version(NULL, &version));
		printf("Z LR20xx v%u.%u\r\n", version.major, version.minor);
	}
}

static void Standby_lr20xx()
{
	LR20xx_setStandby(LR20XX_SYSTEM_STANDBY_MODE_RC);
}

// from start_radio() at radio.c:
// 	* bwKhz is LORA_BW_KHZ
// 	* sf is sf_at_500KHz
// 	* cr is passed 1
static void LoRaModemConfig_lr20xx(unsigned bwKHz, uint8_t sf, uint8_t cr)
{
	lora_mod_params.cr = cr;  // 1 is LR20XX_RADIO_LORA_CR_4_5
	lora_mod_params.sf = sf;  // LR20XX_RADIO_LORA_SF5 is 5, LR20XX_RADIO_LORA_SF12 is 12

	/* Map requested kHz to nearest LR20xx bandwidth */
	if (bwKHz >= 1000) {
		lora_mod_params.bw = LR20XX_RADIO_LORA_BW_1000;
	} else if (bwKHz >= 812) {
		lora_mod_params.bw = LR20XX_RADIO_LORA_BW_812;
	} else if (bwKHz >= 500) {
		lora_mod_params.bw = LR20XX_RADIO_LORA_BW_500;
	} else if (bwKHz >= 406) {
		lora_mod_params.bw = LR20XX_RADIO_LORA_BW_406;
	} else if (bwKHz >= 250) {
		lora_mod_params.bw = LR20XX_RADIO_LORA_BW_250;
	} else if (bwKHz >= 203) {
		lora_mod_params.bw = LR20XX_RADIO_LORA_BW_203;
	} else if (bwKHz >= 125) {
		lora_mod_params.bw = LR20XX_RADIO_LORA_BW_125;
	} else if (bwKHz >= 101) {
		lora_mod_params.bw = LR20XX_RADIO_LORA_BW_101;
	} else if (bwKHz >= 83) {
		lora_mod_params.bw = LR20XX_RADIO_LORA_BW_83;
	} else if (bwKHz >= 62) {
		lora_mod_params.bw = LR20XX_RADIO_LORA_BW_62;
	} else if (bwKHz >= 41) {
		lora_mod_params.bw = LR20XX_RADIO_LORA_BW_41;
	} else if (bwKHz >= 31) {
		lora_mod_params.bw = LR20XX_RADIO_LORA_BW_31;
	} else if (bwKHz >= 20) {
		lora_mod_params.bw = LR20XX_RADIO_LORA_BW_20;
	} else if (bwKHz >= 15) {
		lora_mod_params.bw = LR20XX_RADIO_LORA_BW_15;
	} else if (bwKHz >= 10) {
		lora_mod_params.bw = LR20XX_RADIO_LORA_BW_10;
	} else {
		lora_mod_params.bw = LR20XX_RADIO_LORA_BW_7;
	}

	lora_mod_params.ppm = lr20xx_radio_lora_get_recommended_ppm_offset(lora_mod_params.sf, lora_mod_params.bw);

	ASSERT_LR20XX_RC( lr20xx_radio_lora_set_modulation_params(NULL, &lora_mod_params) );
}

/* Calibration delta threshold in Hz (50 MHz) */
#define LR20XX_CALIBRATION_DELTA_HZ    50000000

static uint32_t last_calibrated_freq_hz = 0;
static uint32_t current_rf_freq_hz = 0;

void SetChannel_lr20xx(unsigned hz)
{
	current_rf_freq_hz = hz;
	ASSERT_LR20XX_RC( lr20xx_radio_common_set_rf_freq(NULL, hz) );

	/* Calibrate front end if frequency changed significantly */
	uint32_t freq_diff;
	if (hz > last_calibrated_freq_hz)
		freq_diff = hz - last_calibrated_freq_hz;
	else
		freq_diff = last_calibrated_freq_hz - hz;

	if (freq_diff >= LR20XX_CALIBRATION_DELTA_HZ || last_calibrated_freq_hz == 0) {
		lr20xx_radio_common_front_end_calibration_value_t calibration = {
			.frequency_in_hertz = hz,
			.rx_path = LR20XX_RADIO_COMMON_RX_PATH_LF,
		};
		ASSERT_LR20XX_RC( lr20xx_radio_common_calibrate_front_end_helper(NULL, &calibration, 1) );
		last_calibrated_freq_hz = hz;
	}
}

static void Rx_lr20xx(unsigned timeout_us)
{
	uint32_t timeout_rtc;

	if (timeout_us == 0) {
		/* Continuous RX mode */
		timeout_rtc = 0xFFFFFF;
	} else {
		/* Convert microseconds to RTC ticks (32.768 kHz = 30.52µs per tick) */
		timeout_rtc = (uint32_t)(((uint64_t)timeout_us * 32768) / 1000000);
		if (timeout_rtc > 0xFFFFFE) {
			timeout_rtc = 0xFFFFFE;  /* Max timeout ~511 seconds */
		}
	}

	LR20xx_chipMode = LR20XX_SYSTEM_CHIP_MODE_RX;
	ASSERT_LR20XX_RC( lr20xx_radio_common_set_rx_with_timeout_in_rtc_step(NULL, timeout_rtc) );
}

static void set_tx_dbm_lr20xx(int8_t dbm)
{
	/* For LR20XX_RADIO_COMMON_PA_SEL_LF: power range is -9.5dBm to +22dBm
	 * power_half_dbm = dbm * 2 (since power is in 0.5dBm steps)
	 * Clamp to valid range: 0xED (-9.5dBm) to 0x2C (+22dBm) */
	int8_t power_half_dbm = dbm * 2;
	if (power_half_dbm > 44)      // +22dBm max
		power_half_dbm = 44;
	if (power_half_dbm < -19)     // -9.5dBm min
		power_half_dbm = -19;

	ASSERT_LR20XX_RC( lr20xx_radio_common_set_tx_params(NULL, power_half_dbm, LR20XX_RADIO_COMMON_RAMP_48_US) );
}

int Send_lr20xx(uint8_t size/*, timestamp_t maxListenTime, timestamp_t channelFreeTime, int rssiThresh*/)
{
	/* Update payload length in stored packet params */
	lora_pkt_params.pld_len_in_bytes = size;

	if (lr20xx_radio_lora_set_packet_params( NULL, &lora_pkt_params ) != LR20XX_STATUS_OK)
		return -1;
	lr20xx_radio_fifo_clear_tx( NULL );
	if (lr20xx_radio_fifo_write_tx( NULL, LR20xx_tx_buf, size) != LR20XX_STATUS_OK)
		return -1;
	if (lr20xx_radio_common_set_tx(NULL, 0) != LR20XX_STATUS_OK)
		return -1;

	LR20xx_chipMode = LR20XX_SYSTEM_CHIP_MODE_TX;
	return 0;
}

static bool service_lr20xx()
{
    return LR20xx_service();
}

static void LoRaPacketConfig_lr20xx(unsigned preambleLen, bool fixLen, bool crcOn, bool invIQ)
{
	lora_pkt_params.preamble_len_in_symb = preambleLen;
	lora_pkt_params.pkt_mode = fixLen ? LR20XX_RADIO_LORA_PKT_IMPLICIT : LR20XX_RADIO_LORA_PKT_EXPLICIT;
	lora_pkt_params.crc = crcOn ? LR20XX_RADIO_LORA_CRC_ENABLED : LR20XX_RADIO_LORA_CRC_DISABLED;
	lora_pkt_params.iq = invIQ ? LR20XX_RADIO_LORA_IQ_INVERTED : LR20XX_RADIO_LORA_IQ_STANDARD;
	/* pld_len_in_bytes will be set by Send_lr20xx() */

	ASSERT_LR20XX_RC( lr20xx_radio_lora_set_packet_params(NULL, &lora_pkt_params) );
}

static void printOpMode_lr20xx()
{
	lr20xx_system_stat1_t stat1;
	lr20xx_system_stat2_t stat2;
	lr20xx_system_irq_mask_t irq_status;

	if (lr20xx_system_get_status(NULL, &stat1, &stat2, &irq_status) == LR20XX_STATUS_OK) {
		switch (stat2.chip_mode) {
			case LR20XX_SYSTEM_CHIP_MODE_SLEEP: printf("SLEEP "); break;
			case LR20XX_SYSTEM_CHIP_MODE_STBY_RC: printf("STBY_RC "); break;
			case LR20XX_SYSTEM_CHIP_MODE_STBY_XOSC: printf("STBY_XOSC "); break;
			case LR20XX_SYSTEM_CHIP_MODE_FS: printf("FS "); break;
			case LR20XX_SYSTEM_CHIP_MODE_RX: printf("RX "); break;
			case LR20XX_SYSTEM_CHIP_MODE_TX: printf("TX "); break;
			default: printf("?%d? ", stat2.chip_mode); break;
		}
	}
}

/* Bandwidth values in order from lowest to highest, with kHz values */
static const struct {
	lr20xx_radio_lora_bw_t bw;
	uint16_t khz;
} bw_table[] = {
	{ LR20XX_RADIO_LORA_BW_7,    8 },    /* 7.81 kHz */
	{ LR20XX_RADIO_LORA_BW_10,   10 },   /* 10.42 kHz */
	{ LR20XX_RADIO_LORA_BW_15,   16 },   /* 15.63 kHz */
	{ LR20XX_RADIO_LORA_BW_20,   21 },   /* 20.83 kHz */
	{ LR20XX_RADIO_LORA_BW_31,   31 },   /* 31.25 kHz */
	{ LR20XX_RADIO_LORA_BW_41,   42 },   /* 41.67 kHz */
	{ LR20XX_RADIO_LORA_BW_62,   63 },   /* 62.50 kHz */
	{ LR20XX_RADIO_LORA_BW_83,   83 },   /* 83.34 kHz */
	{ LR20XX_RADIO_LORA_BW_101,  102 },  /* 101.56 kHz */
	{ LR20XX_RADIO_LORA_BW_125,  125 },  /* 125 kHz */
	{ LR20XX_RADIO_LORA_BW_203,  203 },  /* 203 kHz */
	{ LR20XX_RADIO_LORA_BW_250,  250 },  /* 250 kHz */
	{ LR20XX_RADIO_LORA_BW_406,  406 },  /* 406 kHz */
	{ LR20XX_RADIO_LORA_BW_500,  500 },  /* 500 kHz */
	{ LR20XX_RADIO_LORA_BW_812,  812 },  /* 812 kHz */
	{ LR20XX_RADIO_LORA_BW_1000, 1000 }, /* 1000 kHz */
};
#define BW_TABLE_SIZE (sizeof(bw_table) / sizeof(bw_table[0]))

static bool is_bw_at_lowest(void)
{
	return lora_mod_params.bw == LR20XX_RADIO_LORA_BW_7;
}

static bool is_bw_at_highest(void)
{
	return lora_mod_params.bw == LR20XX_RADIO_LORA_BW_1000;
}

void step_bw_lr20xx(bool up)
{
	unsigned i;

	/* Find current position in table */
	for (i = 0; i < BW_TABLE_SIZE; i++) {
		if (bw_table[i].bw == lora_mod_params.bw)
			break;
	}

	if (i >= BW_TABLE_SIZE)
		return;  /* current bw not found */

	if (up) {
		if (i < BW_TABLE_SIZE - 1)
			i++;
	} else {
		if (i > 0)
			i--;
	}

	lora_mod_params.bw = bw_table[i].bw;
	lora_mod_params.ppm = lr20xx_radio_lora_get_recommended_ppm_offset(lora_mod_params.sf, lora_mod_params.bw);
	ASSERT_LR20XX_RC( lr20xx_radio_lora_set_modulation_params(NULL, &lora_mod_params) );
}

uint16_t get_bw_khz_lr20xx(void)
{
	for (unsigned i = 0; i < BW_TABLE_SIZE; i++) {
		if (bw_table[i].bw == lora_mod_params.bw)
			return bw_table[i].khz;
	}
	return 0;
}

uint8_t get_sf_lr20xx(void)
{
	return lora_mod_params.sf;
}

uint32_t get_freq_hz_lr20xx(void)
{
	return current_rf_freq_hz;
}

uint8_t get_chip_mode_lr20xx(void)
{
	lr20xx_system_stat1_t stat1;
	lr20xx_system_stat2_t stat2;
	lr20xx_system_irq_mask_t irq_status;

	if (lr20xx_system_get_status(NULL, &stat1, &stat2, &irq_status) == LR20XX_STATUS_OK) {
		return stat2.chip_mode;
	}
	return 0xff;  /* error */
}

static bool is_sf_at_slowest(void)
{
	return lora_mod_params.sf == LR20XX_RADIO_LORA_SF12;
}

static bool is_sf_at_fastest(void)
{
	return lora_mod_params.sf == LR20XX_RADIO_LORA_SF5;
}

void step_sf_lr20xx(bool up)
{
	if (up) {
		/* Higher SF = slower, more range */
		if (lora_mod_params.sf < LR20XX_RADIO_LORA_SF12)
			lora_mod_params.sf++;
	} else {
		/* Lower SF = faster */
		if (lora_mod_params.sf > LR20XX_RADIO_LORA_SF5)
			lora_mod_params.sf--;
	}

	lora_mod_params.ppm = lr20xx_radio_lora_get_recommended_ppm_offset(lora_mod_params.sf, lora_mod_params.bw);
	ASSERT_LR20XX_RC( lr20xx_radio_lora_set_modulation_params(NULL, &lora_mod_params) );
}

static void PostConfigReadBack()
{
	/* do nothing here, or save initial values */
}

void sethal_lr20xx()
{
    lorahal.init = Init_lr20xx;
    lorahal.standby = Standby_lr20xx;
    lorahal.loRaModemConfig = LoRaModemConfig_lr20xx;
    lorahal.setChannel = SetChannel_lr20xx;
    lorahal.set_tx_dbm = set_tx_dbm_lr20xx;
    lorahal.loRaPacketConfig = LoRaPacketConfig_lr20xx;
    lorahal.send = Send_lr20xx;
    lorahal.printOpMode = printOpMode_lr20xx;
    lorahal.service = service_lr20xx;
    lorahal.rx = Rx_lr20xx;
    lorahal.irqTopHalf = LR20xx_dio8_topHalf;
    lorahal.irq_pin = DIO8_PIN;
    lorahal.postcfgreadback = PostConfigReadBack;
    lorahal.bw_at_highest = is_bw_at_highest;
    lorahal.bw_at_lowest = is_bw_at_lowest;
    lorahal.sf_at_slowest = is_sf_at_slowest;
    lorahal.sf_at_fastest = is_sf_at_fastest;

    lorahal.rx_buf = LR20xx_rx_buf;
    lorahal.tx_buf = LR20xx_tx_buf;
}
