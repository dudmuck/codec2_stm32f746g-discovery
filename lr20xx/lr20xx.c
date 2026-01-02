#include <stdio.h>
#include "lr20xx.h"
#include "lr20xx_hal.h"
#include "lr20xx_system.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_radio_fifo.h"
#include "lr20xx_radio_lora.h"
#include "pinDefs_lr20xx.h"
#include "stm32f7xx_hal.h"

lr20xx_system_chip_modes_t LR20xx_chipMode;

void (*LR20xx_dio8_topHalf)(void);
void (*LR20xx_timeout)(bool tx);
void (*LR20xx_txDone)(void);
void (*LR20xx_rxDone)(uint8_t size, float rssi, float snr);
void (*LR20xx_chipModeChange)(void);
void (*LR20xx_cadDone)(bool detected);
void (*LR20xx_preambleDetected)(void);
void (*LR20xx_fifoTx)(lr20xx_radio_fifo_flag_t tx_fifo_flags);
void (*LR20xx_fifoRx)(lr20xx_radio_fifo_flag_t rx_fifo_flags);

uint8_t LR20xx_tx_buf[LR20XX_BUF_SIZE];
uint8_t LR20xx_rx_buf[LR20XX_BUF_SIZE];

/*yyy void test_reset_pin()
{
	printf("testResetPin\r\n");
	for (;;) {
    	HAL_GPIO_WritePin(NRST_PORT, NRST_PIN, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(NRST_PORT, NRST_PIN, GPIO_PIN_SET);
	}
}*/

void init_lr20xx()
{
    GPIO_InitTypeDef   GPIO_InitStructure;

    /************** NRST output **********************/
    NRST_GPIO_CLK_ENABLE();
    GPIO_InitStructure.Pin = NRST_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(NRST_PORT, &GPIO_InitStructure);
    HAL_GPIO_WritePin(NRST_PORT, NRST_PIN, GPIO_PIN_SET);  // Start with reset released
	//yyy test_reset_pin();

    /************** BUSY input **********************/
    BUSY_GPIO_CLK_ENABLE();
    GPIO_InitStructure.Pin = BUSY_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BUSY_PORT, &GPIO_InitStructure);

    /************** dio8 interrupt in **********************/
    DIO8_GPIO_CLK_ENABLE();
    GPIO_InitStructure.Pin = DIO8_PIN;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
    HAL_GPIO_Init(DIO8_PORT, &GPIO_InitStructure);

    HAL_NVIC_SetPriority(DIO8_IRQn, 0x0F, 0x00);
    HAL_NVIC_EnableIRQ(DIO8_IRQn);
}

void LR20xx_setStandby(lr20xx_system_standby_mode_t sm)
{
	ASSERT_LR20XX_RC( lr20xx_system_set_standby_mode(NULL, sm) );
	LR20xx_chipMode = LR20XX_SYSTEM_CHIP_MODE_STBY_RC;
}

bool LR20xx_service()
{

    if (BUSY) {
        return true;
    }

    while (DIO8) {
        lr20xx_system_irq_mask_t irqFlags;
        ASSERT_LR20XX_RC( lr20xx_system_get_and_clear_irq_status(NULL, &irqFlags) );

        /* Update chip mode from radio status (updated after every SPI transaction) */
        extern lr20xx_stat_t stat;
        LR20xx_chipMode = stat.bits.chip_mode;

        if (irqFlags & LR20XX_SYSTEM_IRQ_TX_DONE) {
            printf("TX_DONE IRQ flags=0x%08lX\r\n", (unsigned long)irqFlags);
            if (LR20xx_txDone)
                LR20xx_txDone();
        }

        /* Handle LORA_HEADER_ERROR - clear FIFO to prevent stale data.
         * Stats are tracked by radio and reported via fhss_print_and_reset_rx_stats(). */
        if (irqFlags & LR20XX_SYSTEM_IRQ_LORA_HEADER_ERROR) {
            lr20xx_radio_fifo_clear_rx(NULL);
        }

        if (irqFlags & LR20XX_SYSTEM_IRQ_RX_DONE) {
            if (!(irqFlags & LR20XX_SYSTEM_IRQ_CRC_ERROR) &&
                !(irqFlags & LR20XX_SYSTEM_IRQ_LEN_ERROR) &&
                !(irqFlags & LR20XX_SYSTEM_IRQ_LORA_HEADER_ERROR)) {
                uint16_t size;
                ASSERT_LR20XX_RC( lr20xx_radio_common_get_rx_packet_length(NULL, &size) );
                if (size <= LR20XX_BUF_SIZE) {
                    /* Check if streaming RX already read some data */
                    extern volatile uint16_t rx_fifo_read_idx;
                    if (rx_fifo_read_idx > 0) {
                        /* Streaming RX active - only read remaining bytes not yet read */
                        if (size > rx_fifo_read_idx) {
                            uint16_t remaining = size - rx_fifo_read_idx;
                            ASSERT_LR20XX_RC( lr20xx_radio_fifo_read_rx(NULL, &LR20xx_rx_buf[rx_fifo_read_idx], remaining) );
                            rx_fifo_read_idx = size;  /* Update index so streaming_rx_decode() sees all data */
                        }
                        /* Data already in buffer from streaming reads */
                    } else {
                        /* Normal RX - read entire packet */
                        ASSERT_LR20XX_RC( lr20xx_radio_fifo_read_rx(NULL, LR20xx_rx_buf, size) );
                        rx_fifo_read_idx = size;  /* Update index so streaming_rx_decode() sees data */
                    }
                    if (LR20xx_rxDone) {
                        lr20xx_radio_lora_packet_status_t pkt_status = { 0 };
                        ASSERT_LR20XX_RC( lr20xx_radio_lora_get_packet_status(NULL, &pkt_status) );
                        float rssi = pkt_status.rssi_pkt_in_dbm - (pkt_status.rssi_pkt_half_dbm_count * 0.5f);
                        float snr = pkt_status.snr_pkt_raw / 4.0f;
                        LR20xx_rxDone(size, rssi, snr);
                    }
                }
            }
        }
        if (irqFlags & LR20XX_SYSTEM_IRQ_TIMEOUT) {
			LR20xx_timeout(LR20xx_chipMode == LR20XX_SYSTEM_CHIP_MODE_TX);
		}

        if (irqFlags & LR20XX_SYSTEM_IRQ_FIFO_TX) {
            lr20xx_radio_fifo_flag_t rx_fifo_flags, tx_fifo_flags;
            ASSERT_LR20XX_RC( lr20xx_radio_fifo_get_and_clear_irq_flags(NULL, &rx_fifo_flags, &tx_fifo_flags) );
            if (LR20xx_fifoTx)
                LR20xx_fifoTx(tx_fifo_flags);
        }

        if (irqFlags & LR20XX_SYSTEM_IRQ_FIFO_RX) {
            lr20xx_radio_fifo_flag_t rx_fifo_flags, tx_fifo_flags;
            ASSERT_LR20XX_RC( lr20xx_radio_fifo_get_and_clear_irq_flags(NULL, &rx_fifo_flags, &tx_fifo_flags) );
            if (LR20xx_fifoRx)
                LR20xx_fifoRx(rx_fifo_flags);
        }

        if (irqFlags & LR20XX_SYSTEM_IRQ_CAD_DONE) {
            bool detected = (irqFlags & LR20XX_SYSTEM_IRQ_CAD_DETECTED) != 0;
            if (LR20xx_cadDone)
                LR20xx_cadDone(detected);
        }

        if (irqFlags & LR20XX_SYSTEM_IRQ_PREAMBLE_DETECTED) {
            if (LR20xx_preambleDetected)
                LR20xx_preambleDetected();
        }

        if (irqFlags & LR20XX_SYSTEM_IRQ_ERROR) {
            lr20xx_system_errors_t errors;
            if (lr20xx_system_get_errors(NULL, &errors) == LR20XX_STATUS_OK) {
                printf("LR20xx ERROR: 0x%04X\r\n", errors);
                if (errors & LR20XX_SYSTEM_ERRORS_HF_XOSC_START_MASK)
                    printf("  HF_XOSC_START_ERR\r\n");
                if (errors & LR20XX_SYSTEM_ERRORS_LF_XOSC_START_MASK)
                    printf("  LF_XOSC_START_ERR\r\n");
                if (errors & LR20XX_SYSTEM_ERRORS_PLL_LOCK_MASK)
                    printf("  PLL_LOCK_ERR\r\n");
                if (errors & LR20XX_SYSTEM_ERRORS_LF_RC_CALIB_MASK)
                    printf("  LF_RC_CALIB_ERR\r\n");
                if (errors & LR20XX_SYSTEM_ERRORS_HF_RC_CALIB_MASK)
                    printf("  HF_RC_CALIB_ERR\r\n");
                if (errors & LR20XX_SYSTEM_ERRORS_PLL_CALIB_MASK)
                    printf("  PLL_CALIB_ERR\r\n");
                if (errors & LR20XX_SYSTEM_ERRORS_AAF_CALIB_MASK)
                    printf("  AAF_CALIB_ERR\r\n");
                if (errors & LR20XX_SYSTEM_ERRORS_IMG_CALIB_MASK)
                    printf("  IMG_CALIB_ERR\r\n");
                if (errors & LR20XX_SYSTEM_ERRORS_MEAS_UNIT_ADC_CALIB_MASK)
                    printf("  MEAS_UNIT_ADC_CALIB_ERR\r\n");
                if (errors & LR20XX_SYSTEM_ERRORS_RXFREQ_NO_FRONT_END_CALIB_MASK)
                    printf("  RXFREQ_NO_FRONT_END_CALIB_ERR\r\n");
                lr20xx_system_clear_errors(NULL);
            }
        }
    }

    return false;
}
