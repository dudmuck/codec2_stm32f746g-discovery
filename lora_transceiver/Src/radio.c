/* abstracted radio interface: works with any radio chip */
#include "main.h"
#include "radio.h"
#ifndef ENABLE_LR20XX
#include "sx126x.h"
#include "sx127x.h"
#include "pinDefs_sx126x.h"
#include "pinDefs_sx127x.h"
#else
#include "lr20xx.h"
#include "lr20xx_radio_fifo.h"
#endif /* !ENABLE_LR20XX */
#include "pinDefs.h"
#ifdef USE_FREERTOS
#include "freertos_tasks.h"
#endif

lorahal_t lorahal;

volatile unsigned tickAtIrq;
volatile uint8_t mlatirq;

void radio_irq_callback()
{
    tickAtIrq = uwTick;
}

void txDoneCB()
{
    txing = 0;
    appHal.lcd_printOpMode(false);
    lcd_print_tx_duration((int)(tickAtIrq - txStartAt), (unsigned)cycleDur);
#ifdef USE_FREERTOS
    /* Signal TX task that transmission is complete.
     * This is called from task context (lorahal.service() in main loop),
     * so use regular semaphore API, not FromISR variant. */
    if (xTxDoneSemaphore != NULL) {
        xSemaphoreGive(xTxDoneSemaphore);
    }
#endif
    if (rx_start_at_tx_done) {
        lora_rx_begin();
        rx_start_at_tx_done = 0;
    }
}

void rxTimeoutCB()
{
}

#ifdef ENABLE_LR20XX
volatile uint8_t fifo_tx_room;      /* flag: TX FIFO has room for more data */
volatile uint8_t fifo_tx_underflow; /* flag: TX FIFO underflow occurred */
volatile uint8_t fifo_rx_overflow;  /* flag: RX FIFO overflow occurred */
volatile uint16_t rx_fifo_read_idx; /* how much data read from RX FIFO so far */
volatile uint8_t fifo_rx_disabled;  /* flag: disable RX FIFO reads during packet transition */

void fifoTxCB(lr20xx_radio_fifo_flag_t tx_fifo_flags)
{
    if (tx_fifo_flags & LR20XX_RADIO_FIFO_FLAG_UNDERFLOW) {
        fifo_tx_underflow = 1;
    }
    if (tx_fifo_flags & LR20XX_RADIO_FIFO_FLAG_THRESHOLD_LOW) {
        fifo_tx_room = 1;
        /* If streaming TX active, send more data to FIFO */
        if (streaming_tx_active) {
            Send_lr20xx_fifo_continue();
        }
    }
}

static uint32_t fifo_rx_irq_count;
static uint32_t fifo_rx_total_bytes;  /* Debug: total bytes read from RX FIFO */
static uint16_t fifo_rx_max_idx;      /* Debug: max rx_fifo_read_idx reached */

/* Counter for software buffer full events (debug) */
static uint32_t fifo_sw_full_count;
uint32_t get_fifo_sw_full_count(void) { return fifo_sw_full_count; }

void fifoRxCB(lr20xx_radio_fifo_flag_t rx_fifo_flags)
{
    if (rx_fifo_flags & LR20XX_RADIO_FIFO_FLAG_OVERFLOW) {
        fifo_rx_overflow = 1;
        /* Clear FIFO to stop interrupt storm - data is corrupted anyway */
        lr20xx_radio_fifo_clear_rx(NULL);
        return;  /* Don't try to read corrupted data */
    }
    if (rx_fifo_flags & LR20XX_RADIO_FIFO_FLAG_THRESHOLD_HIGH) {
        fifo_rx_irq_count++;
        /* RX FIFO threshold reached - read available data in ONE operation.
         * At 95 kbps, looping to drain causes data to arrive faster than we read,
         * leading to overflow. Single read per callback is more efficient. */
        uint16_t fifo_level;
        uint16_t max_read = (streaming_cfg.packets_per_frame > 1) ? LR20XX_BUF_SIZE : (lora_payload_length * 2);

        /* Check if software buffer is full - if so, clear hardware FIFO to prevent overflow.
         * This loses data but maintains buffer consistency. The decode loop will handle
         * resync via sequence number tracking. */
        if (rx_fifo_read_idx >= max_read) {
            lr20xx_radio_fifo_clear_rx(NULL);
            fifo_sw_full_count++;
            return;  /* Buffer full - decoder severely behind */
        }

        if (lr20xx_radio_fifo_get_rx_level(NULL, &fifo_level) != LR20XX_STATUS_OK) {
            return;  /* FIFO query failed */
        }

        if (fifo_level == 0) {
            return;  /* FIFO empty */
        }

        uint16_t space_left = max_read - rx_fifo_read_idx;
        if (fifo_level > space_left) {
            fifo_level = space_left;  /* Limit to available space */
        }

        if (fifo_level > 0 && (rx_fifo_read_idx + fifo_level) <= LR20XX_BUF_SIZE) {
            lr20xx_radio_fifo_read_rx(NULL, &LR20xx_rx_buf[rx_fifo_read_idx], fifo_level);
            rx_fifo_read_idx += fifo_level;
            fifo_rx_total_bytes += fifo_level;
            if (rx_fifo_read_idx > fifo_rx_max_idx) {
                fifo_rx_max_idx = rx_fifo_read_idx;
            }
        }
    }
}

uint32_t get_fifo_rx_irq_count(void) { return fifo_rx_irq_count; }
uint32_t get_fifo_rx_total_bytes(void) { return fifo_rx_total_bytes; }
uint16_t get_fifo_rx_max_idx(void) { return fifo_rx_max_idx; }
void reset_fifo_rx_debug(void) { fifo_rx_total_bytes = 0; fifo_rx_max_idx = 0; }
#endif /* ENABLE_LR20XX */

const RadioEvents_t rev = {
    /* DioPin_top_half */     radio_irq_callback,
    /* TxDone_topHalf */    NULL,
    /* TxDone_botHalf */    txDoneCB,
    /* TxTimeout  */        NULL,
    /* RxDone  */           rxDoneCB,
    /* RxTimeout  */        rxTimeoutCB,
    /* RxError  */          NULL,
    /* FhssChangeChannel  */NULL,
    /* CadDone  */          NULL
};

extern uint16_t lora_bw_khz;
#define TX_DBM                  /*20*/ -5		/* TODO low power used for bench testing */
#define CF_HZ               917600000

/************************************************************/

SPI_HandleTypeDef SpiHandle;

uint8_t spi_transfer(uint8_t out_data)
{
    HAL_StatusTypeDef status;
    uint8_t in_data;
    status = HAL_SPI_TransmitReceive(&SpiHandle, &out_data, &in_data, 1, 100);
    switch (status) {
        case HAL_BUSY: printf("SPI_BUSY "); break;
        case HAL_ERROR: printf("SPI_ERROR "); break;
        case HAL_TIMEOUT: printf("SPI_TIMEOUT "); break;
        default: break;
    }
    return in_data;
}

static void spi_begin()
{
    GPIO_InitTypeDef   GPIO_InitStructure;

    NSS_SX126X_GPIO_CLK_ENABLE();
    GPIO_InitStructure.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Pin = NSS_SX126X_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(NSS_SX126X_PORT, &GPIO_InitStructure);

    NSS_SX127X_GPIO_CLK_ENABLE();
    GPIO_InitStructure.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Pin = NSS_SX127X_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(NSS_SX127X_PORT, &GPIO_InitStructure);

    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    SPIx_SCK_GPIO_CLK_ENABLE();
    SPIx_MISO_GPIO_CLK_ENABLE();
    SPIx_MOSI_GPIO_CLK_ENABLE();
    /* Enable SPI clock */
    SPIx_CLK_ENABLE();

    /*##-2- Configure peripheral GPIO ##########################################*/
    /* SPI SCK GPIO pin configuration  */
    GPIO_InitStructure.Pin       = SPIx_SCK_PIN;
    GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull      = GPIO_PULLUP;
    GPIO_InitStructure.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Alternate = SPIx_SCK_AF;
    HAL_GPIO_Init(SPIx_SCK_GPIO_PORT, &GPIO_InitStructure);

    /* SPI MISO GPIO pin configuration  */
    GPIO_InitStructure.Pin = SPIx_MISO_PIN;
    GPIO_InitStructure.Alternate = SPIx_MISO_AF;
    HAL_GPIO_Init(SPIx_MISO_GPIO_PORT, &GPIO_InitStructure);

    /* SPI MOSI GPIO pin configuration  */
    GPIO_InitStructure.Pin = SPIx_MOSI_PIN;
    GPIO_InitStructure.Alternate = SPIx_MOSI_AF;
    HAL_GPIO_Init(SPIx_MOSI_GPIO_PORT, &GPIO_InitStructure);
    
    SpiHandle.Instance               = SPIx;
    SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;  /* 54MHz/4 = 13.5MHz for faster radio access */
    SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
    SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
    SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
    SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
    SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
    SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    SpiHandle.Init.CRCPolynomial     = 7;
    SpiHandle.Init.NSS               = SPI_NSS_SOFT;
    SpiHandle.Init.Mode = SPI_MODE_MASTER;
    if (HAL_SPI_Init(&SpiHandle) != HAL_OK)
    {
        printf("spiInitFail\r\n");
        for (;;) asm("nop");
    }

} // ..spi_begin()

void start_radio()
{
#ifndef ENABLE_LR20XX
    status_t status;
#endif

    spi_begin();

#ifdef ENABLE_LR20XX
    sethal_lr20xx();
    setAppHal_lr20xx();
    LR20xx_fifoTx = fifoTxCB;
    LR20xx_fifoRx = fifoRxCB;
#else
    SX126x_xfer(OPCODE_GET_STATUS, 0, 1, &status.octet);

    RegOpMode.octet = read_reg(REG_OPMODE);
    if (RegOpMode.octet != 0xff) {
        /* radio is sx127x: status.octet==00, sx127xopmode==1 */
        printf("is sx127x\r\n");
        sethal_sx127x();
        setAppHal_sx127x();
    } else {
        /* radio is sx126x: status.octet==45, sx127xopmode==0xff */
        printf("IS sx126x\r\n");
        sethal_sx126x();
        setAppHal_sx126x();
    }
#endif

    lorahal.init(&rev);

    lorahal.standby();

#ifdef MODEM_FSK
    /* Calculate FSK bitrate based on Opus frame size and rate.
     * FSK_bitrate = (payload_bits + overhead_bits) * frames_per_sec * margin
     * Overhead: ~96 bits (40 preamble + 32 sync + 8 length + 16 CRC)
     * Add 50% margin for timing tolerance (multi-frame modes need more headroom) */
    {
        extern uint16_t _bytes_per_frame;
        extern uint8_t frames_per_sec;
        uint32_t payload_bits = _bytes_per_frame * 8;
        uint32_t overhead_bits = 96;
        uint32_t bits_per_frame = payload_bits + overhead_bits;
        /* bitrate_bps = bits_per_frame * frames_per_sec * 1.5 (50% margin) */
        uint32_t fsk_bitrate_bps = (bits_per_frame * frames_per_sec * 15) / 10;
        uint32_t fsk_bitrate_kbps = (fsk_bitrate_bps + 999) / 1000;  /* round up */
        printf("FSK: %u bytes/frame * %u fps -> %lu kbps\r\n",
               _bytes_per_frame, frames_per_sec, fsk_bitrate_kbps);
        lorahal.loRaModemConfig(fsk_bitrate_kbps, 0, 0);
    }
#else
    lorahal.loRaModemConfig(lora_bw_khz, sf_at_500KHz, 1);
#endif

    lorahal.setChannel(CF_HZ);

    lorahal.set_tx_dbm(TX_DBM);

#ifdef MODEM_FSK
    /* FSK packet config: preamble in bits, variable length, CRC on */
    lorahal.loRaPacketConfig(40, false, true, false);
#else
                      // preambleLen, fixLen, crcOn, invIQ
    lorahal.loRaPacketConfig(8, false, false, false);      // crcOff
#endif

    lorahal.postcfgreadback();
}

