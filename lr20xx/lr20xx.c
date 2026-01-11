#include <stdio.h>
#include "lr20xx.h"
#include "lr20xx_system.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_radio_fifo.h"
#ifdef MODEM_FSK
#include "lr20xx_radio_fsk.h"
#else
#include "lr20xx_radio_lora.h"
#endif
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

/* Deferred FIFO warning flag (to avoid slow printf in IRQ) */
volatile uint16_t rx_fifo_residual_bytes;  /* >0 means warning needs to be printed */

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

    /* DIO8 interrupt priority: higher priority for faster TX_DONE response.
     * Must be >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5) for FreeRTOS. */
    HAL_NVIC_SetPriority(DIO8_IRQn, 0x06, 0x00);
    HAL_NVIC_EnableIRQ(DIO8_IRQn);
}

void LR20xx_setStandby(lr20xx_system_standby_mode_t sm)
{
	ASSERT_LR20XX_RC( lr20xx_system_set_standby_mode(NULL, sm) );
	LR20xx_chipMode = LR20XX_SYSTEM_CHIP_MODE_STBY_RC;
}

/* External flag set by DIO8 interrupt in stm32f7xx_it.c */
extern volatile uint8_t dio8_irq_pending;

bool LR20xx_service()
{
    if (BUSY) {
        return true;
    }

    /* Check interrupt flag for faster response */
    while (DIO8 || dio8_irq_pending) {
        dio8_irq_pending = 0;  /* Clear flag */
        lr20xx_system_irq_mask_t irqFlags;
        ASSERT_LR20XX_RC( lr20xx_system_get_and_clear_irq_status(NULL, &irqFlags) );

        if (irqFlags & LR20XX_SYSTEM_IRQ_TX_DONE) {
			LR20xx_chipMode = LR20XX_SYSTEM_CHIP_MODE_STBY_RC;
            if (LR20xx_txDone)
                LR20xx_txDone();
        }

        if (irqFlags & LR20XX_SYSTEM_IRQ_RX_DONE) {
            if (!(irqFlags & LR20XX_SYSTEM_IRQ_CRC_ERROR) &&
                !(irqFlags & LR20XX_SYSTEM_IRQ_LEN_ERROR)) {
                uint16_t size;
                ASSERT_LR20XX_RC( lr20xx_radio_common_get_rx_packet_length(NULL, &size) );
                if (size <= LR20XX_BUF_SIZE) {
                    extern volatile uint16_t rx_fifo_read_idx;

                    if (LR20xx_fifoRx) {
                        /* Streaming mode: delegate FIFO reads to the FIFO callback.
                         * This ensures a single consistent code path for all FIFO reads,
                         * preventing race conditions in multi-packet mode. */
                        lr20xx_radio_fifo_flag_t rx_flags = LR20XX_RADIO_FIFO_FLAG_THRESHOLD_HIGH;
                        LR20xx_fifoRx(rx_flags);  /* Read any remaining data */
                    } else {
                        /* Non-streaming mode: read entire packet directly */
                        uint16_t fifo_remaining;
                        if (lr20xx_radio_fifo_get_rx_level(NULL, &fifo_remaining) == LR20XX_STATUS_OK) {
                            if (fifo_remaining > 0 && (rx_fifo_read_idx + fifo_remaining) <= LR20XX_BUF_SIZE) {
                                ASSERT_LR20XX_RC( lr20xx_radio_fifo_read_rx(NULL, &LR20xx_rx_buf[rx_fifo_read_idx], fifo_remaining) );
                                rx_fifo_read_idx += fifo_remaining;
                            }
                        } else if (rx_fifo_read_idx == 0) {
                            /* Fallback: no FIFO level available, read full packet to start */
                            ASSERT_LR20XX_RC( lr20xx_radio_fifo_read_rx(NULL, LR20xx_rx_buf, size) );
                            rx_fifo_read_idx = size;
                        }
                    }
                    if (LR20xx_rxDone) {
#ifdef MODEM_FSK
                        lr20xx_radio_fsk_packet_status_t pkt_status = { 0 };
                        ASSERT_LR20XX_RC( lr20xx_radio_fsk_get_packet_status(NULL, &pkt_status) );
                        float rssi = pkt_status.rssi_avg_in_dbm - (pkt_status.rssi_avg_half_dbm_count * 0.5f);
                        float snr = 0.0f;  /* FSK doesn't provide SNR */
#else
                        lr20xx_radio_lora_packet_status_t pkt_status = { 0 };
                        ASSERT_LR20XX_RC( lr20xx_radio_lora_get_packet_status(NULL, &pkt_status) );
                        float rssi = pkt_status.rssi_pkt_in_dbm - (pkt_status.rssi_pkt_half_dbm_count * 0.5f);
                        float snr = pkt_status.snr_pkt_raw / 4.0f;
#endif
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
