#include "main.h"
#include "radio.h"
#include "sx126x.h"

extern ModulationParams_t mpLORA;   // from radio.c

const uint8_t loraBWs[] = {
    LORA_BW_7, LORA_BW_10, LORA_BW_15,
    LORA_BW_20, LORA_BW_31, LORA_BW_41,
    LORA_BW_62, LORA_BW_125, LORA_BW_250,
    LORA_BW_500
};

static const char* const lora_bwstrs[] = {
    " 7.81KHz", "10.42KHz", "15.63KHz",
    "20.83KHz", "31.25KHz", "41.67KHz",
    "62.5KHz", "125KHz", "250KHz",
    "500KHz",
    NULL
};

void sx126x_step_bw(bool up)
{
    unsigned n, prevbw = mpLORA.lora.bandwidth;
    for (n = 0; n < sizeof(loraBWs); n++) {
        if (mpLORA.lora.bandwidth == loraBWs[n]) {
            printf("prevBwIdx:%u,%02x ", n, loraBWs[n]);
            break;
        }
    }

    if (up) {
        if (n < 10)
            n++;
    } else {
        if (n > 0)
            n--;
    }
    printf("after%u,%02x\r\n", n, loraBWs[n]);
    mpLORA.lora.bandwidth = loraBWs[n];
    if (prevbw != mpLORA.lora.bandwidth) {
        SX126x_xfer(OPCODE_SET_MODULATION_PARAMS, 4, 0, mpLORA.buf);
    }

}


void sx126x_step_sf(bool up)
{
    unsigned prevsf = mpLORA.lora.spreadingFactor;
    if (up) {
        if (mpLORA.lora.spreadingFactor > 5)
            mpLORA.lora.spreadingFactor--;
    } else {
        if (mpLORA.lora.spreadingFactor < 12)
            mpLORA.lora.spreadingFactor++;
    }
    if (prevsf != mpLORA.lora.spreadingFactor) {
        SX126x_xfer(OPCODE_SET_MODULATION_PARAMS, 4, 0, mpLORA.buf);
    }
}

static void lcd_print_sx126x_sf(uint8_t x, uint8_t y)
{
    char str[16];
    loraConfig0_t conf0;
    conf0.octet = SX126x_readReg(REG_ADDR_LORA_CONFIG0, 1);
    mpLORA.lora.spreadingFactor = conf0.bits.modem_sf;
    sprintf(str, "sf%u ", conf0.bits.modem_sf);
    BSP_LCD_DisplayStringAt(x, y, (uint8_t *)str, LEFT_MODE);
}

static void lcd_print_sx126x_bw(uint8_t x, uint8_t y)
{
    char str[16];
    int n;
    loraConfig0_t conf0;
    conf0.octet = SX126x_readReg(REG_ADDR_LORA_CONFIG0, 1);
    mpLORA.lora.bandwidth = conf0.bits.modem_bw;

    for (n = 0; n < sizeof(loraBWs); n++) {
        if (conf0.bits.modem_bw == loraBWs[n]) {
            sprintf(str, "%s   ", lora_bwstrs[n]);
            BSP_LCD_DisplayStringAt(x, y, (uint8_t *)str, LEFT_MODE);
            break;
        }
    }
}

static void lcd_print_sx126x_opmode(bool tx_wait)
{
    char str[16];
    const char* modeStr;
    status_t status;
    SX126x_xfer(OPCODE_GET_STATUS, 0, 1, &status.octet);

    switch (status.bits.chipMode) {
        case 2: // STBY_RC
        case 3: // STBY_XOSC
        case 4: // FS
            modeStr = "   ";
            BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
            BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
            break;
        case 5:
            BSP_LCD_SetBackColor(LCD_COLOR_GREEN);
            BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
            modeStr = "RX";
            break; // RX
        case 6:
            BSP_LCD_SetBackColor(LCD_COLOR_RED);
            BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
            modeStr = "TX";
            break; // TX
        default:
            BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
            BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
            sprintf(str, "%d", status.bits.chipMode);
            modeStr = str;
            break;
    }

    BSP_LCD_SetFont(&Font24);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()-50, (uint8_t*)modeStr, RIGHT_MODE);
}

static void sx126x_print_status()
{
    float MHz;
    int n, bw_idx = -1;
    
    lorahal.printOpMode();

    {
        loraConfig0_t conf0;
        conf0.octet = SX126x_readReg(REG_ADDR_LORA_CONFIG0, 1);
        for (n = 0; n < sizeof(loraBWs); n++) {
            if (conf0.bits.modem_bw == loraBWs[n]) {
                bw_idx = n;
                break;
            }
        }

        printf("%s sf%u ", lora_bwstrs[bw_idx], conf0.bits.modem_sf);
    }
    MHz = SX126x_getMHz();
    printf("%uKHz\r\n", (unsigned)(MHz*1000));
}

void setAppHal_sx126x()
{
    appHal.lcd_printOpMode = lcd_print_sx126x_opmode;
    appHal.radioPrintStatus = sx126x_print_status;
    appHal.lcd_print_bw = lcd_print_sx126x_bw;
    appHal.lcd_print_sf = lcd_print_sx126x_sf;
    appHal.step_bw = sx126x_step_bw;
    appHal.step_sf = sx126x_step_sf;
}

