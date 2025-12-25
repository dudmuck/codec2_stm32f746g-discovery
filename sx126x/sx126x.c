#include "sx126x.h"
#include "pinDefs_sx126x.h"
#include "pinDefs.h"
#include <stdbool.h>
#include <stdio.h>
#include "stm32f7xx_hal.h"

#define BUSY_PIN                        GPIO_PIN_4
#define BUSY_PORT                       GPIOB
#define BUSY_GPIO_CLK_ENABLE            __HAL_RCC_GPIOB_CLK_ENABLE
#define BUSY                            (HAL_GPIO_ReadPin(BUSY_PORT, BUSY_PIN) == GPIO_PIN_SET)

chipMote_e SX126x_chipMode;

void (*SX126x_dio1_topHalf)(void);
void (*SX126x_timeout)(bool tx);
void (*SX126x_txDone)(void);
void (*SX126x_rxDone)(uint8_t size, float rssi, float snr);
void (*SX126x_chipModeChange)(void);
void (*SX126x_cadDone)(bool detected);
void (*SX126x_preambleDetected)(void);

uint8_t SX126x_tx_buf[256];    // lora fifo size
uint8_t SX126x_rx_buf[256];    // lora fifo size

void SX126x_start_tx(uint8_t pktLen)
{
    uint8_t buf[8];
 
    {
        uint8_t i;
 
        while (BUSY)
            ;
 
        ASSERT_SX126X_NSS;

        spi_transfer(OPCODE_WRITE_BUFFER);
        spi_transfer(0);   // offset
        i = 0;
        for (i = 0; i < pktLen; i++) {
            spi_transfer(SX126x_tx_buf[i]);
        }
        UNASSERT_SX126X_NSS;
    }
 
    buf[0] = 0x40;
    buf[1] = 0x00;
    buf[2] = 0x00;
    SX126x_xfer(OPCODE_SET_TX, 3, 0, buf);
 
    SX126x_chipMode = CHIPMODE_TX;
    if (SX126x_chipModeChange)
        SX126x_chipModeChange();
}


void SX126x_xfer(uint8_t opcode, uint8_t wlen, uint8_t rlen, uint8_t* ptr)
{
    static bool sleeping = false;
    const uint8_t* stopPtr;
    const uint8_t* wstop;
    const uint8_t* rstop;
    uint8_t nop = 0;
 
    if (sleeping) {
        ASSERT_SX126X_NSS;
        while (BUSY)
            ;
        sleeping = false;
    } else {
        while (BUSY)
            ;
        ASSERT_SX126X_NSS;
    }
 

    spi_transfer(opcode);
 
    wstop = ptr + wlen;
    rstop = ptr + rlen;
    if (rlen > wlen)
        stopPtr = rstop;
    else
        stopPtr = wstop;
 
    for (; ptr < stopPtr; ptr++) {
        if (ptr < wstop && ptr < rstop)
            *ptr = spi_transfer(*ptr);
        else if (ptr < wstop)
            spi_transfer(*ptr);
        else
            *ptr = spi_transfer(nop);    // n >= write length: send NOP
    }
 
    UNASSERT_SX126X_NSS;
 
    if (opcode == OPCODE_SET_SLEEP)
        sleeping = true;
}

void SX126x_setPacketType(uint8_t type)
{
    SX126x_xfer(OPCODE_SET_PACKET_TYPE, 1, 0, &type);
}

uint8_t SX126x_getPacketType()
{
    uint8_t buf[2];
    SX126x_xfer(OPCODE_GET_PACKET_TYPE, 0, 2, buf);
    return buf[1];
}

uint32_t SX126x_readReg(uint16_t addr, uint8_t len)
{
    uint32_t ret = 0;
    unsigned i;
 
    uint8_t buf[7];
    buf[0] = addr >> 8;
    buf[1] = (uint8_t)addr;
    SX126x_xfer(OPCODE_READ_REGISTER, 2, 3+len, buf);
    for (i = 0; i < len; i++) {
        ret <<= 8;
        ret |= buf[i+3];
    }
    return ret;
}

void SX126x_writeReg(uint16_t addr, uint32_t data, uint8_t len)
{
    uint8_t buf[6];
    uint8_t n;
    buf[0] = addr >> 8;
    buf[1] = (uint8_t)addr;
    for (n = len; n > 0; n--) {
        buf[n+1] = (uint8_t)data;
        data >>= 8;
    }
    SX126x_xfer(OPCODE_WRITE_REGISTER, 2+len, 2+len, buf);
}

void SX126x_set_tx_dbm(bool is1262, int8_t dbm)
{
    uint8_t buf[4];
    // use OCP default
 
    buf[3] = 1;
    if (is1262) {
        buf[0] = 4;
        buf[1] = 7;
        buf[2] = 0;
 
        if (dbm > 22)
            dbm = 22;
        else if (dbm < -3)
            dbm = -3;
    } else {
        if (dbm == 15)
            buf[0] = 6;
        else
            buf[0] = 4;
        buf[1] = 0;
        buf[2] = 1;
 
        if (dbm > 14)
            dbm = 14;
        else if (dbm < -3)
            dbm = -3;
    }
    SX126x_xfer(OPCODE_SET_PA_CONFIG, 4, 0, buf);
 
    if (is1262 && dbm > 18) {
        /* OCP is set by chip whenever SetPaConfig() is called */
        SX126x_writeReg(REG_ADDR_OCP, 0x38, 1);
    }
 
    // SetTxParams
    buf[0] = dbm;
    //if (opt == 0) txco
    buf[1] = SET_RAMP_200U;
    SX126x_xfer(OPCODE_SET_TX_PARAMS, 2, 0, buf);
}

float SX126x_getMHz()
{
    uint32_t frf = SX126x_readReg(REG_ADDR_RFFREQ, 4);
    return frf / (float)MHZ_TO_FRF;
}

uint8_t SX126x_setMHz(float MHz)
{
    unsigned frf = MHz * MHZ_TO_FRF;
    uint8_t buf[4];
 
    buf[0] = frf >> 24;
    buf[1] = frf >> 16;
    buf[2] = frf >> 8;
    buf[3] = frf;
    SX126x_xfer(OPCODE_SET_RF_FREQUENCY, 4, 0, buf);
    return buf[3];
}

void SX126x_setStandby(stby_t stby)
{
    uint8_t octet = stby;
    SX126x_xfer(OPCODE_SET_STANDBY, 1, 0, &octet);
 
    SX126x_chipMode = CHIPMODE_NONE;
    if (SX126x_chipModeChange)
        SX126x_chipModeChange();
}

void SX126x_SetDIO2AsRfSwitchCtrl(uint8_t en)
{
    SX126x_xfer(OPCODE_SET_DIO2_AS_RFSWITCH, 1, 0, &en);
}


/*void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == DIO1_PIN) {
        if (SX126x_dio1_topHalf)
            SX126x_dio1_topHalf();
    }
}*/

/* PI0 is DIO1 */
void DIO1_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(DIO1_PIN);
}

void init_sx126x()
{
    GPIO_InitTypeDef   GPIO_InitStructure;

    BUSY_GPIO_CLK_ENABLE();
    GPIO_InitStructure.Pin = BUSY_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BUSY_PORT, &GPIO_InitStructure);

    /************** dio1 interrupt in **********************/
    GPIO_InitStructure.Pin = DIO1_PIN;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
    HAL_GPIO_Init(DIO1_PORT, &GPIO_InitStructure);
    
    HAL_NVIC_SetPriority(DIO1_IRQn, 0x0F, 0x00);
    HAL_NVIC_EnableIRQ(DIO1_IRQn);
}


bool SX126x_service()
{
    IrqFlags_t irqFlags, clearIrqFlags;
    uint8_t buf[4];
 
    if (BUSY) {
        return true;
    }
 
    while (DIO1) {
        SX126x_xfer(OPCODE_GET_IRQ_STATUS, 0, 3, buf);
        irqFlags.word = buf[1] << 8;
        irqFlags.word |= buf[2];
        clearIrqFlags.word = 0;
        if (irqFlags.bits.TxDone) {
            SX126x_chipMode = CHIPMODE_NONE;
            if (SX126x_chipModeChange)
                SX126x_chipModeChange();  // might change to Rx
            if (SX126x_txDone)
                SX126x_txDone();
            clearIrqFlags.bits.TxDone = 1;
        }
        if (irqFlags.bits.RxDone) {
            if (SX126x_rxDone) {
                uint8_t len;
                float snr, rssi;
                int8_t s;
                SX126x_xfer(OPCODE_GET_RX_BUFFER_STATUS, 0, 3, buf);
                len = buf[1];
                SX126x_ReadBuffer(len, buf[2]);
                SX126x_xfer(OPCODE_GET_PACKET_STATUS, 0, 4, buf);
                rssi = -buf[1] / 2.0;   // TODO FSK
                s = buf[2];
                snr = s / 4.0;
                SX126x_rxDone(len, rssi, snr);
            }
            clearIrqFlags.bits.RxDone = 1;
        }
        if (irqFlags.bits.Timeout) {
            if (SX126x_chipMode != CHIPMODE_NONE) {
                if (SX126x_timeout)
                    SX126x_timeout(SX126x_chipMode == CHIPMODE_TX);
            }
            SX126x_chipMode = CHIPMODE_NONE;
            if (SX126x_chipModeChange)
                SX126x_chipModeChange();
            clearIrqFlags.bits.Timeout = 1;
        }
        if (irqFlags.bits.CadDone) {
            if (SX126x_cadDone)
                SX126x_cadDone(irqFlags.bits.CadDetected);
 
            clearIrqFlags.bits.CadDone = 1;
            clearIrqFlags.bits.CadDetected = irqFlags.bits.CadDetected;
        }
        if (irqFlags.bits.PreambleDetected) {
            clearIrqFlags.bits.PreambleDetected = 1;
            if (SX126x_preambleDetected)
                SX126x_preambleDetected();
        }
 
        if (clearIrqFlags.word != 0) {
            buf[0] = clearIrqFlags.word >> 8;
            buf[1] = (uint8_t)clearIrqFlags.word;
            SX126x_xfer(OPCODE_CLEAR_IRQ_STATUS, 2, 0, buf);
        }
 
    } // ...while (dio1)
 
    return false;
} // ..service()

void SX126x_ReadBuffer(uint8_t size, uint8_t offset)
{
    unsigned i;
    while (BUSY)
        ;
 
    ASSERT_SX126X_NSS;
 
    spi_transfer(OPCODE_READ_BUFFER);
    spi_transfer(offset);
    spi_transfer(0);   // NOP
    i = 0;
    for (i = 0; i < size; i++) {
        SX126x_rx_buf[i] = spi_transfer(0);
    }
 
    UNASSERT_SX126X_NSS;
}


void SX126x_start_rx(unsigned timeout)
{
    uint8_t buf[8];
 
    buf[0] = timeout >> 16;
    buf[1] = timeout >> 8;
    buf[2] = timeout;
    SX126x_xfer(OPCODE_SET_RX, 3, 0, buf);
 
    SX126x_chipMode = CHIPMODE_RX;
    if (SX126x_chipModeChange)
        SX126x_chipModeChange();
}

