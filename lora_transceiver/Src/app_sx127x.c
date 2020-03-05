#include <inttypes.h>
#include "main.h"
#include "radio.h"
#include "sx127x.h"

static void lcd_print_sx127x_opmode(bool tx_wait)
{
    char str[16];
    const char* modeStr = NULL;

    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
try:
    RegOpMode.octet = read_reg(REG_OPMODE);

    switch (RegOpMode.bits.Mode) {
        case 0:
            break;
            modeStr = "SLEEP"; // 0
            break;
        case 1:
            modeStr = "       "; // 1  standby
            break;
        case 2:
            modeStr = "FS_TX"; // 2
            break;
        case 3:
            BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
            BSP_LCD_SetTextColor(LCD_COLOR_RED);
            modeStr = "   TX"; // 3
            break;
        case 4:
            modeStr = "FS_RX"; // 4
            break;
        case 5:
            BSP_LCD_SetBackColor(LCD_COLOR_GREEN);
            BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
            modeStr = "RX"; // 5
            break;
        case 6:
            BSP_LCD_SetBackColor(LCD_COLOR_GREEN);
            BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
            modeStr = "RX_SINGLE"; // 6
            break;
        case 7:
            modeStr = "CAD"; // 7
            break;
        default:
            sprintf(str, "%d", RegOpMode.bits.Mode);
            break;
        
    }
    
    BSP_LCD_SetFont(&Font24);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()-50, (uint8_t*)modeStr, RIGHT_MODE);

    if (tx_wait && RegOpMode.bits.Mode == 2)    // FS_TX is step to TX
        goto try;
}

static void lora_print_dio()
{
    RegDioMapping2.octet = read_reg(REG_DIOMAPPING2);
    printf("DIO5:");
    switch (RegDioMapping2.bits.Dio5Mapping) {
        case 0: printf("ModeReady"); break;
        case 1: printf("ClkOut"); break;
        case 2: printf("ClkOut"); break;
    }
    printf(" DIO4:");
    switch (RegDioMapping2.bits.Dio4Mapping) {
        case 0: printf("CadDetected"); break;
        case 1: printf("PllLock"); break;
        case 2: printf("PllLock"); break;
    }    
    RegDioMapping1.octet = read_reg(REG_DIOMAPPING1);
    printf(" DIO3:");
    switch (RegDioMapping1.bits.Dio3Mapping) {
        case 0: printf("CadDone"); break;
        case 1: printf("ValidHeader"); break;
        case 2: printf("PayloadCrcError"); break;
    }    
    printf(" DIO2:");
    switch (RegDioMapping1.bits.Dio2Mapping) {
        case 0:
        case 1:
        case 2:
            printf("FhssChangeChannel");
            break;
    }    
    printf(" DIO1:");
    switch (RegDioMapping1.bits.Dio1Mapping) {
        case 0: printf("RxTimeout"); break;
        case 1: printf("FhssChangeChannel"); break;
        case 2: printf("CadDetected"); break;
    }    
    printf(" DIO0:");
    switch (RegDioMapping1.bits.Dio0Mapping) {
        case 0: printf("RxDone"); break;
        case 1: printf("TxDone"); break;
        case 2: printf("CadDone"); break;
    }    
    
    printf("\r\n"); 
}

void lora_printCodingRate(bool from_rx)
{
    uint8_t d = getCodingRate(from_rx);
    printf("CodingRate:");
    switch (d) {
        case 1: printf("4/5 "); break;
        case 2: printf("4/6 "); break;
        case 3: printf("4/7 "); break;
        case 4: printf("4/8 "); break;
        default:
            printf("%d ", d);
            break;
    }
}

void lora_printHeaderMode()
{
    if (getHeaderMode())
        printf("implicit ");
    else
        printf("explicit ");
}

void lora_getBwStr(char **out)
{
    (void)sx127x_getBw();
    
    if (xcvr_type == SX1276) {
        switch (RegModemConfig.sx1276bits.Bw) {
            case 0: *out = "7.8KHz "; break;
            case 1: *out = "10.4KHz "; break;
            case 2: *out = "15.6KHz "; break;
            case 3: *out = "20.8KHz "; break;
            case 4: *out = "31.25KHz "; break;
            case 5: *out = "41.7KHz "; break;
            case 6: *out = "62.5KHz "; break;
            case 7: *out = "125KHz "; break;
            case 8: *out = "250KHz "; break;
            case 9: *out = "500KHz "; break;
            default: *out = NULL; break;
        }
    } else if (xcvr_type == SX1272) {
        switch (RegModemConfig.sx1272bits.Bw) {
            case 0: *out = "125KHz "; break;
            case 1: *out = "250KHz "; break;
            case 2: *out = "500KHz "; break;
            case 3: *out = "11b "; break;
        }
    }
}

static void lora_printRxPayloadCrcOn()
{
    bool on = getRxPayloadCrcOn();
    printf("RxPayloadCrcOn:%d = ", on);
    if (getHeaderMode())
        printf("Rx/");  // implicit mode
        
    if (on)
        printf("Tx CRC Enabled\r\n");
    else
        printf("Tx CRC disabled\r\n");
}

void printLoraIrqs_(bool clear)
{
    //already read RegIrqFlags.octet = read_reg(REG_LR_IRQFLAGS);
    printf("\r\nIrqFlags:");
    if (RegIrqFlags.bits.CadDetected)
        printf("CadDetected ");
    if (RegIrqFlags.bits.FhssChangeChannel) {
        //RegHopChannel.octet = read_reg(REG_LR_HOPCHANNEL);
        printf("FhssChangeChannel:%d ", RegHopChannel.bits.FhssPresentChannel);
    }
    if (RegIrqFlags.bits.CadDone)
        printf("CadDone ");
    if (RegIrqFlags.bits.TxDone)
        printf("TxDone ");
    if (RegIrqFlags.bits.ValidHeader)
        printf("[42mValidHeader[0m ");
    if (RegIrqFlags.bits.PayloadCrcError)
        printf("[41mPayloadCrcError[0m ");
    if (RegIrqFlags.bits.RxDone)
        printf("[42mRxDone[0m ");  
    if (RegIrqFlags.bits.RxTimeout)
        printf("RxTimeout ");
 
    printf("\r\n");
 
    if (clear)
        write_reg(REG_LR_IRQFLAGS, RegIrqFlags.octet);
 
}

static void lora_print_status()
{    
    char *bwstr;
    RegOpMode.octet = read_reg(REG_OPMODE);
    if (!RegOpMode.bits.LongRangeMode) {
        printf("FSK\r\n");
        return;
    }
    
    lora_print_dio();
    printf("LoRa ");
    
    // printing LoRa registers at 0x0d -> 0x3f
 
    RegModemConfig.octet = read_reg(REG_LR_MODEMCONFIG);
    RegModemConfig2.octet = read_reg(REG_LR_MODEMCONFIG2);
 
    lora_printCodingRate(false); // false: transmitted coding rate
    lora_printHeaderMode();

    lora_getBwStr(&bwstr);
    printf(" %s sf:%d ", bwstr, sx127x_getSf());
    lora_printRxPayloadCrcOn();
    // RegModemStat
    printf("ModemStat:0x%02x\r\n", read_reg(REG_LR_MODEMSTAT));
 
    // fifo ptrs:
    RegPayloadLength = read_reg(REG_LR_PAYLOADLENGTH);
    RegRxMaxPayloadLength = read_reg(REG_LR_RX_MAX_PAYLOADLENGTH);
    printf("fifoptr=0x%02x txbase=0x%02x rxbase=0x%02x payloadLength=0x%02x maxlen=0x%02x",
        read_reg(REG_LR_FIFOADDRPTR),
        read_reg(REG_LR_FIFOTXBASEADDR),
        read_reg(REG_LR_FIFORXBASEADDR),
        RegPayloadLength,
        RegRxMaxPayloadLength
    );
 
    RegIrqFlags.octet = read_reg(REG_LR_IRQFLAGS);
    printLoraIrqs_(false);
 
    RegHopPeriod = read_reg(REG_LR_HOPPERIOD);
    if (RegHopPeriod != 0) {
        printf("\r\nHopPeriod:0x%02x\r\n", RegHopPeriod);
    }
 
    printf("SymbTimeout:%d ", read_u16(REG_LR_MODEMCONFIG2) & 0x3ff);
 
    RegPreamble = read_u16(REG_LR_PREAMBLEMSB);
    printf("PreambleLength:%d ", RegPreamble);
 
    if (RegOpMode.bits.Mode == RF_OPMODE_RECEIVER || RegOpMode.bits.Mode == RF_OPMODE_RECEIVER_SINGLE) {
        printf("rssi:%ddBm ", get_current_rssi());
    }
 
    printf("TxContinuousMode:%d ", RegModemConfig2.sx1276bits.TxContinuousMode);    // same for sx1272 and sx1276
 
    printf("\r\n");
    printf("AgcAutoOn:%d", getAgcAutoOn());
    if (xcvr_type == SX1272) {
        printf(" LowDataRateOptimize:%d\r\n", RegModemConfig.sx1272bits.LowDataRateOptimize);
    }
 
    printf("\r\nHeaderCount:%d PacketCount:%d, ",
        read_u16(REG_LR_RXHEADERCNTVALUE_MSB), read_u16(REG_LR_RXPACKETCNTVALUE_MSB));
 
    printf("Lora detection threshold:%02x\r\n", read_reg(REG_LR_DETECTION_THRESHOLD));
    RegTest31.octet = read_reg(REG_LR_TEST31);
    printf("detect_trig_same_peaks_nb:%d\r\n", RegTest31.bits.detect_trig_same_peaks_nb);
 
    if (xcvr_type == SX1272) {
        RegModemConfig.octet = read_reg(REG_LR_MODEMCONFIG);
        printf("LowDataRateOptimize:%d ", RegModemConfig.sx1272bits.LowDataRateOptimize);
    } else if (xcvr_type == SX1276) {
        RegModemConfig3.octet = read_reg(REG_LR_MODEMCONFIG3);
        printf("LowDataRateOptimize:%d ", RegModemConfig3.sx1276bits.LowDataRateOptimize);        
    }
    
    printf(" invert: rx=%d tx=%d\r\n", RegTest33.bits.invert_i_q, !RegTest33.bits.chirp_invert_tx);
    
    printf("\r\n");
}

static void
printPa()
{
    RegPaConfig.octet = read_reg(REG_PACONFIG);
    if (RegPaConfig.bits.PaSelect) {
        double output_dBm = 17 - (15-RegPaConfig.bits.OutputPower);
        printf(" PABOOST OutputPower=%.1fdBm", output_dBm);
    } else {
        double pmax = (0.6*RegPaConfig.bits.MaxPower) + 10.8;
        double output_dBm = pmax - (15-RegPaConfig.bits.OutputPower);
#ifdef TARGET_MTS_MDOT_F411RE
        printf(" \x1b[31mRFO pmax=%.1fdBm OutputPower=%.1fdBm\x1b[0m", pmax, output_dBm);  // not connected
#else
        printf(" RFO pmax=%.1fdBm OutputPower=%.1fdBm", pmax, output_dBm);
#endif
    }
}

static void printOpMode()
{
    RegOpMode.octet = read_reg(REG_OPMODE);
    switch (RegOpMode.bits.Mode) {
        case RF_OPMODE_SLEEP: printf("[7msleep[0m"); break;
        case RF_OPMODE_STANDBY: printf("[7mstby[0m"); break;
        case RF_OPMODE_SYNTHESIZER_TX: printf("[33mfstx[0m"); break;
        case RF_OPMODE_TRANSMITTER: printf("[31mtx[0m"); break;
        case RF_OPMODE_SYNTHESIZER_RX: printf("[33mfsrx[0m"); break;
        case RF_OPMODE_RECEIVER: printf("[32mrx[0m"); break;
        case 6:
            if (RegOpMode.bits.LongRangeMode)
                printf("[42mrxs[0m");
            else
                printf("-6-");
            break;  // todo: different lora/fsk
        case 7:
            if (RegOpMode.bits.LongRangeMode)
                printf("[45mcad[0m");
            else
                printf("-7-");
            break;  // todo: different lora/fsk
    }
}

static void /* things always present, whether lora or fsk */
common_print_status()
{
    printf("version:0x%02x %.3fMHz ", read_reg(REG_VERSION), (double)get_frf_MHz());
    printOpMode();
 
    printPa();
 
    RegOcp.octet = read_reg(REG_OCP);
    if (RegOcp.bits.OcpOn) {
        int imax = 0;
        if (RegOcp.bits.OcpTrim < 16)
            imax = 45 + (5 * RegOcp.bits.OcpTrim);
        else if (RegOcp.bits.OcpTrim < 28)
            imax = -30 + (10 * RegOcp.bits.OcpTrim);
        else
            imax = 240;
        printf(" OcpOn %dmA ", imax);
    } else
        printf(" OcpOFF ");
 
    printf("\r\n");
    
}

static void sx127x_print_status()
{
    if (xcvr_type == SX1276) {
#if defined(TARGET_MTS_MDOT_F411RE)
        printf("\r\nSX1276 ");
#else
        if (shield_type == SHIELD_TYPE_LAS)
            printf("\r\nSX1276LAS ");
        if (shield_type == SHIELD_TYPE_MAS)
            printf("\r\nSX1276MAS ");
#endif /* !TARGET_MTS_MDOT_F411RE */                       
    } else if (xcvr_type == SX1272)
        printf("\r\nSX1272 ");
        
    RegOpMode.octet = read_reg(REG_OPMODE);
    if (RegOpMode.bits.LongRangeMode)
        lora_print_status();
    else {
        /*fsk_print_status();*/
        printf("TODOFSK ");
    }
    common_print_status();
}


static void lcd_print_sx127x_bw(uint8_t x, uint8_t y)
{
    char *bwstr;
    lora_getBwStr(&bwstr);
    BSP_LCD_DisplayStringAt(x, y, (uint8_t *)bwstr, LEFT_MODE);
    
}

static void lcd_print_sx127x_sf(uint8_t x, uint8_t y)
{
    char str[16];
    sprintf(str, "%u", sx127x_getSf());
    BSP_LCD_DisplayStringAt(x, y, (uint8_t *)str, LEFT_MODE);
}

void sx127x_step_bw(bool up)
{
    uint8_t bw = sx127x_getBw();

    if (xcvr_type == SX1276) {
        if (up) {
            if (bw < 9)
                bw++;
        }
    } else if (xcvr_type == SX1272) {
        if (up) {
            if (bw < 2)
                bw++;
        }
    }

    if (!up) {
        if (bw > 0)
            bw--;
    }

    setBw(bw);
}

void sx127x_step_sf(bool up)
{
    uint8_t sf = sx127x_getSf();
    if (up) {
        if (sf > 7)
            sf--;
    } else {
        if (sf < 12)
            sf++;
    }
    setSf(sf);
}

void setAppHal_sx127x()
{
    appHal.lcd_printOpMode = lcd_print_sx127x_opmode;
    appHal.radioPrintStatus = sx127x_print_status;
    appHal.lcd_print_bw = lcd_print_sx127x_bw;
    appHal.lcd_print_sf = lcd_print_sx127x_sf;
    appHal.step_bw = sx127x_step_bw;
    appHal.step_sf = sx127x_step_sf;
}

