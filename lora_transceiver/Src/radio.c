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

#define LORA_BW_KHZ             500
#define TX_DBM                  20
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
    SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
    lorahal.loRaModemConfig(LORA_BW_KHZ, sf_at_500KHz, 1);
    lorahal.setChannel(CF_HZ);

    lorahal.set_tx_dbm(TX_DBM);

                      // preambleLen, fixLen, crcOn, invIQ
    lorahal.loRaPacketConfig(8, false, false, false);      // crcOff

    lorahal.postcfgreadback();
}

