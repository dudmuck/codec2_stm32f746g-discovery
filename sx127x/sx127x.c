#include "sx127x.h"
#include "pinDefs_sx127x.h"
#include "pinDefs.h"
#include "stm32f7xx_hal.h"

RegOpMode_t RegOpMode;
RegPaConfig_t RegPaConfig;
RegOcp_t RegOcp;            // 0x0b

RegIrqFlags_t       RegIrqFlags;            // 0x12
uint8_t             RegRxNbBytes;           // 0x13
RegModemStatus_t    RegModemStatus;         // 0x18
int8_t              RegPktSnrValue;         // 0x19  signed, s/n can be negative
uint8_t             RegPktRssiValue;        // 0x1a
RegHopChannel_t     RegHopChannel;          // 0x1c
RegModemConfig_t    RegModemConfig;         // 0x1d
RegModemConfig2_t   RegModemConfig2;        // 0x1e
RegModemConfig3_t   RegModemConfig3;        // 0x26
uint16_t            RegPreamble;            // 0x20->0x21
uint8_t             RegPayloadLength;       // 0x22
uint8_t             RegRxMaxPayloadLength;  // 0x23
uint8_t             RegHopPeriod;           // 0x24
RegTest31_t         RegTest31;              // 0x31
RegTest33_t         RegTest33;              // 0x33
RegAutoDrift_t      RegAutoDrift;           // 0x36  sx1276 only
RegGainDrift_t      RegGainDrift;           // 0x3a
RegDriftInvert_t    RegDriftInvert;         // 0x3b

RegDioMapping1_t RegDioMapping1;
RegDioMapping2_t RegDioMapping2;

uint8_t SX127x_tx_buf[256];    // lora fifo size
uint8_t SX127x_rx_buf[256];    // lora fifo size

type_e xcvr_type;

uint8_t read_reg(uint8_t addr)
{
    uint8_t ret;
    // Select the device by seting chip select low
    ASSERT_SX127X_NSS;

    spi_transfer(addr); // bit7 is low for reading from radio
 
    // Send a dummy byte to receive the contents of register
    ret = spi_transfer(0x00);
 
    // Deselect the device
    UNASSERT_SX127X_NSS;
    
    return ret;
}

void write_u16(uint8_t addr, uint16_t data)
{
    ASSERT_SX127X_NSS;   // Select the device by seting chip select low

    spi_transfer(addr | 0x80); // bit7 is high for writing to radio
    spi_transfer((data >> 8) & 0xff);
    spi_transfer(data & 0xff);
 
    UNASSERT_SX127X_NSS;   // Deselect the device
}

void write_reg(uint8_t addr, uint8_t data)
{
    ASSERT_SX127X_NSS;   // Select the device by seting chip select low
 
    spi_transfer(addr | 0x80); // bit7 is high for writing to radio
    spi_transfer(data);
 
    UNASSERT_SX127X_NSS;   // Deselect the device
}

void invert_rx(bool inv)
{
    RegTest33.bits.invert_i_q = inv;
    write_reg(REG_LR_TEST33, RegTest33.octet);
    /**/
    RegDriftInvert.bits.invert_timing_error_per_symbol = !RegTest33.bits.invert_i_q;    
    write_reg(REG_LR_DRIFT_INVERT, RegDriftInvert.octet);
}

void invert_tx(bool inv)
{
    RegTest33.bits.chirp_invert_tx = !inv;
    write_reg(REG_LR_TEST33, RegTest33.octet);    
}

void lora_enable()
{
    sx127x_set_opmode(RF_OPMODE_SLEEP);
    
    RegOpMode.bits.LongRangeMode = 1;
    write_reg(REG_OPMODE, RegOpMode.octet);
    
    RegDioMapping1.bits.Dio0Mapping = 0;    // DIO0 to RxDone
    RegDioMapping1.bits.Dio1Mapping = 0;
    write_reg(REG_DIOMAPPING1, RegDioMapping1.octet);
    
    RegTest31.octet = read_reg(REG_LR_TEST31);    
    RegTest31.bits.if_freq_auto = 0;    // improved RX spurious rejection
    write_reg(REG_LR_TEST31, RegTest31.octet);    
        
    sx127x_set_opmode(RF_OPMODE_STANDBY);            
}

static void rfsw_callback()
{
    if (RegOpMode.bits.Mode == RF_OPMODE_TRANSMITTER)
        SET_RFSW;
    else
        CLEAR_RFSW;
}
 

void sx127x_set_opmode(chip_mode_e mode)
{
    RegOpMode.bits.Mode = mode;
    
    // callback to control antenna switch and PaSelect (PABOOST/RFO) for TX
    rfsw_callback();
    
    write_reg(REG_OPMODE, RegOpMode.octet);
}

void DIO0_IRQHandler()
{
   HAL_GPIO_EXTI_IRQHandler(DIO0_PIN);
}

static void sx127x_get_type()
{
    RegOpMode.octet = read_reg(REG_OPMODE);
    
    /* SX1272 starts in FSK mode on powerup, RegOpMode bit3 will be set for BT1.0 in FSK */
    if (!RegOpMode.bits.LongRangeMode) {
        sx127x_set_opmode(RF_OPMODE_SLEEP);
        HAL_Delay(10);
        RegOpMode.bits.LongRangeMode = 1;
        write_reg(REG_OPMODE, RegOpMode.octet);
        HAL_Delay(10);
        RegOpMode.octet = read_reg(REG_OPMODE);     
    }
 
    if (RegOpMode.sx1276LORAbits.LowFrequencyModeOn)
        xcvr_type = SX1276;
    else {
        RegOpMode.sx1276LORAbits.LowFrequencyModeOn = 1;
        write_reg(REG_OPMODE, RegOpMode.octet);
        RegOpMode.octet = read_reg(REG_OPMODE);
        if (RegOpMode.sx1276LORAbits.LowFrequencyModeOn)
            xcvr_type = SX1276;
        else
            xcvr_type = SX1272;
    }
}

shield_type_e shield_type;

void init_sx127x()
{
    GPIO_InitTypeDef   GPIO_InitStructure;

    sx127x_get_type();

    DIO0_GPIO_CLK_ENABLE();
    GPIO_InitStructure.Pin = DIO0_PIN;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
    HAL_GPIO_Init(DIO0_PORT, &GPIO_InitStructure);

    RFSW_GPIO_CLK_ENABLE();
    GPIO_InitStructure.Pin = RFSW_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(RFSW_PORT, &GPIO_InitStructure);
    if (HAL_GPIO_ReadPin(RFSW_PORT, RFSW_PIN) == GPIO_PIN_SET) {
        shield_type = SHIELD_TYPE_LAS;
    } else {
        shield_type = SHIELD_TYPE_MAS;
    }
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(RFSW_PORT, &GPIO_InitStructure);
    
    HAL_NVIC_SetPriority(DIO0_IRQn, 0x0F, 0x00);
    HAL_NVIC_EnableIRQ(DIO0_IRQn);

    if (!RegOpMode.bits.LongRangeMode)
        lora_enable();
        
    RegModemConfig.octet = read_reg(REG_LR_MODEMCONFIG);
    RegModemConfig2.octet = read_reg(REG_LR_MODEMCONFIG2);
    RegTest33.octet = read_reg(REG_LR_TEST33);     // invert_i_q
    RegDriftInvert.octet = read_reg(REG_LR_DRIFT_INVERT);
    RegGainDrift.octet = read_reg(REG_LR_GAIN_DRIFT);
    
    if (xcvr_type == SX1276) {
        RegAutoDrift.octet = read_reg(REG_LR_SX1276_AUTO_DRIFT);
    }
}

uint8_t sx127x_getBw()
{
    if (xcvr_type == SX1276)
        return RegModemConfig.sx1276bits.Bw;
    else if (xcvr_type == SX1272)
        return RegModemConfig.sx1272bits.Bw;
    else
        return 0;
}

volatile bool HF;

void write_u24(uint8_t addr, uint32_t data)
{
    ASSERT_SX127X_NSS;

    spi_transfer(addr | 0x80); // bit7 is high for writing to radio
    spi_transfer((data >> 16) & 0xff);
    spi_transfer((data >> 8) & 0xff);
    spi_transfer(data & 0xff);
 
    UNASSERT_SX127X_NSS;
    
    if (addr == REG_FRFMSB) {
        if (data < 0x8340000)   // < 525MHz
            HF = false;
        else
            HF = true;
    }
}

void set_frf_MHz( float MHz )
{
    uint32_t frf;
    
    frf = MHz / (float)FREQ_STEP_MHZ;
    write_u24(REG_FRFMSB, frf);
    
    if (MHz < 525)
        HF = false;
    else
        HF = true;
}

uint8_t sx127x_getSf()
{
    // spreading factor same between sx127[26]
    return RegModemConfig2.sx1276bits.SpreadingFactor;
}

void setSf(uint8_t sf)
{
    if (!RegOpMode.bits.LongRangeMode)
        return; 

    // write register at 0x37 with value 0xc if at SF6
    if (sf < 7)
        write_reg(REG_LR_DETECTION_THRESHOLD, 0x0c);
    else
        write_reg(REG_LR_DETECTION_THRESHOLD, 0x0a);
    
    RegModemConfig2.sx1276bits.SpreadingFactor = sf; // spreading factor same between sx127[26]
    write_reg(REG_LR_MODEMCONFIG2, RegModemConfig2.octet);
    
    if (xcvr_type == SX1272) {
        if (get_symbol_period() > 16)
            RegModemConfig.sx1272bits.LowDataRateOptimize = 1;
        else
            RegModemConfig.sx1272bits.LowDataRateOptimize = 0;
        write_reg(REG_LR_MODEMCONFIG, RegModemConfig.octet);
    } else if (xcvr_type == SX1276) {
        if (get_symbol_period() > 16)
            RegModemConfig3.sx1276bits.LowDataRateOptimize = 1;
        else
            RegModemConfig3.sx1276bits.LowDataRateOptimize = 0;
        write_reg(REG_LR_MODEMCONFIG3, RegModemConfig3.octet);
    }
}

void write_fifo(uint8_t len)
{
    int i;
    
    ASSERT_SX127X_NSS;
    spi_transfer(REG_FIFO | 0x80); // bit7 is high for writing to radio
    
    for (i = 0; i < len; i++) {
        spi_transfer(SX127x_tx_buf[i]);
    }
    UNASSERT_SX127X_NSS;
}

void start_tx(uint8_t len)
{                   
    // DIO0 to TxDone
    if (RegDioMapping1.bits.Dio0Mapping != 1) {
        RegDioMapping1.bits.Dio0Mapping = 1;
        write_reg(REG_DIOMAPPING1, RegDioMapping1.octet);
    }
    
    // set FifoPtrAddr to FifoTxPtrBase
    write_reg(REG_LR_FIFOADDRPTR, read_reg(REG_LR_FIFOTXBASEADDR));
    
    // write PayloadLength bytes to fifo
    write_fifo(len);
       
    sx127x_set_opmode(RF_OPMODE_TRANSMITTER);
}

void set_nb_trig_peaks(int n)
{
    /* TODO: different requirements for RX_CONTINUOUS vs RX_SINGLE */
    RegTest31.bits.detect_trig_same_peaks_nb = n;
    write_reg(REG_LR_TEST31, RegTest31.octet);
}

void start_rx(chip_mode_e mode)
{
    if (!RegOpMode.bits.LongRangeMode)
        return; // fsk mode
    if (RegOpMode.sx1276LORAbits.AccessSharedReg)
        return; // fsk page
        
    if (xcvr_type == SX1276) {
        if (RegModemConfig.sx1276bits.Bw == 9) {  // if 500KHz bw: improved tolerance of reference frequency error
            if (RegAutoDrift.bits.freq_to_time_drift_auto) {
                RegAutoDrift.bits.freq_to_time_drift_auto = 0;
                write_reg(REG_LR_SX1276_AUTO_DRIFT, RegAutoDrift.octet);
            }
            if (HF) {
                // > 525MHz
                if (RegGainDrift.bits.freq_to_time_drift != 0x24) {
                    RegGainDrift.bits.freq_to_time_drift = 0x24;
                    write_reg(REG_LR_GAIN_DRIFT, RegGainDrift.octet);                    
                }
            } else {
                // < 525MHz
                if (RegGainDrift.bits.freq_to_time_drift != 0x3f) {
                    RegGainDrift.bits.freq_to_time_drift = 0x3f;
                    write_reg(REG_LR_GAIN_DRIFT, RegGainDrift.octet); 
                }
            }

        } else {
            if (!RegAutoDrift.bits.freq_to_time_drift_auto) {
                RegAutoDrift.bits.freq_to_time_drift_auto = 1;
                write_reg(REG_LR_SX1276_AUTO_DRIFT, RegAutoDrift.octet);
            }
        }
    } // ... if (xcvr_type == SX1276)  
    
    // RX_CONTINUOUS: false detections vs missed detections tradeoff
    switch (RegModemConfig2.sx1276bits.SpreadingFactor) {
        case 6:
            set_nb_trig_peaks(3);
            break;
        case 7:
            set_nb_trig_peaks(4);
            break;
        default:
            set_nb_trig_peaks(5);
            break;
    }   
        
    sx127x_set_opmode(mode);

    if (RegDioMapping1.bits.Dio0Mapping != 0) {
        RegDioMapping1.bits.Dio0Mapping = 0;    // DIO0 to RxDone
        write_reg(REG_DIOMAPPING1, RegDioMapping1.octet);
    }
    
    write_reg(REG_LR_FIFOADDRPTR, read_reg(REG_LR_FIFORXBASEADDR));
}

void setBw(uint8_t bw)
{
    if (!RegOpMode.bits.LongRangeMode)
        return;
        
    if (xcvr_type == SX1276) {        
        RegModemConfig.sx1276bits.Bw = bw;
        if (get_symbol_period() > 16)
            RegModemConfig3.sx1276bits.LowDataRateOptimize = 1;
        else
            RegModemConfig3.sx1276bits.LowDataRateOptimize = 0;
        write_reg(REG_LR_MODEMCONFIG3, RegModemConfig3.octet);        
    } else if (xcvr_type == SX1272) {
        RegModemConfig.sx1272bits.Bw = bw;
        if (get_symbol_period() > 16)
            RegModemConfig.sx1272bits.LowDataRateOptimize = 1;
        else
            RegModemConfig.sx1272bits.LowDataRateOptimize = 0;
    } else
        return;
        
    write_reg(REG_LR_MODEMCONFIG, RegModemConfig.octet);
}

void setBw_KHz(int khz)
{
    uint8_t bw = 0;
    
    if (xcvr_type == SX1276) {
        if (khz <= 8) bw = 0;
        else if (khz <= 11) bw = 1;
        else if (khz <= 16) bw = 2;
        else if (khz <= 21) bw = 3;
        else if (khz <= 32) bw = 4;
        else if (khz <= 42) bw = 5;
        else if (khz <= 63) bw = 6;
        else if (khz <= 125) bw = 7;
        else if (khz <= 250) bw = 8;
        else if (khz <= 500) bw = 9;
    } else if (xcvr_type == SX1272) {
        if (khz <= 125) bw = 0;
        else if (khz <= 250) bw = 1;
        else if (khz <= 500) bw = 2;
    }
    
    setBw(bw);
}

float get_symbol_period()
{
    float khz = 0;
    
    if (xcvr_type == SX1276) {
        switch (RegModemConfig.sx1276bits.Bw) {
            case 0: khz = 7.8; break;
            case 1: khz = 10.4; break;
            case 2: khz = 15.6; break;
            case 3: khz = 20.8; break;
            case 4: khz = 31.25; break;
            case 5: khz = 41.7; break;
            case 6: khz = 62.5; break;
            case 7: khz = 125; break;
            case 8: khz = 250; break;
            case 9: khz = 500; break;
        }
    } else if (xcvr_type == SX1272) {
        switch (RegModemConfig.sx1272bits.Bw) {
            case 0: khz = 125; break;
            case 1: khz = 250; break;
            case 2: khz = 500; break;            
        }
    }
    
    // return symbol duration in milliseconds
    return (1 << RegModemConfig2.sx1276bits.SpreadingFactor) / khz; 
}

void read_fifo(uint8_t len)
{
    int i;
     
    ASSERT_SX127X_NSS;
    spi_transfer(REG_FIFO); // bit7 is low for reading from radio
    for (i = 0; i < len; i++) {
        SX127x_rx_buf[i] = spi_transfer(0);
    }
    UNASSERT_SX127X_NSS;
}

service_action_e service()
{
    /*if (RegOpMode.bits.Mode == RF_OPMODE_RECEIVER) {
        if (poll_vh) {
            RegIrqFlags.octet = read_reg(REG_LR_IRQFLAGS);
            if (RegIrqFlags.bits.ValidHeader) {
                RegIrqFlags.octet = 0;
                RegIrqFlags.bits.ValidHeader = 1;
                write_reg(REG_LR_IRQFLAGS, RegIrqFlags.octet);
                printf("VH\r\n");
            }
        }
    }*/
       
    if (IS_DIO0_CLR)
        return SERVICE_NONE;
        
    switch (RegDioMapping1.bits.Dio0Mapping) {
        case 0: // RxDone
            /* user checks for CRC error in IrqFlags */
            RegIrqFlags.octet = read_reg(REG_LR_IRQFLAGS);  // save flags
            RegHopChannel.octet = read_reg(REG_LR_HOPCHANNEL);
            //printf("[%02x]", RegIrqFlags.octet);
            write_reg(REG_LR_IRQFLAGS, RegIrqFlags.octet); // clear flags in radio
            
            /* any register of interest on received packet is read(saved) here */        
            RegModemStatus.octet = read_reg(REG_LR_MODEMSTAT);          
            RegPktSnrValue = read_reg(REG_LR_PKTSNRVALUE);
            RegPktRssiValue = read_reg(REG_LR_PKTRSSIVALUE);
            RegRxNbBytes = read_reg(REG_LR_RXNBBYTES);
    
            write_reg(REG_LR_FIFOADDRPTR, read_reg(REG_LR_FIFORXCURRENTADDR));
            read_fifo(RegRxNbBytes);
            return SERVICE_READ_FIFO;
        case 1: // TxDone
            RegIrqFlags.octet = 0;
            RegIrqFlags.bits.TxDone = 1;
            write_reg(REG_LR_IRQFLAGS, RegIrqFlags.octet);                  
            return SERVICE_TX_DONE;        
    } // ...switch (RegDioMapping1.bits.Dio0Mapping)
    
    return SERVICE_ERROR;    
}

int get_pkt_rssi()
{
    if (xcvr_type == SX1276) {
        if (HF)
            return RegPktRssiValue - 157;
        else
            return RegPktRssiValue - 164;
    } else
        return RegPktRssiValue - 139;
}

uint8_t getCodingRate(bool from_rx)
{
    if (from_rx) {
        // expected RegModemStatus was read on RxDone interrupt
        return RegModemStatus.bits.RxCodingRate;    
    } else {    // transmitted coding rate...
        if (xcvr_type == SX1276)
            return RegModemConfig.sx1276bits.CodingRate;
        else if (xcvr_type == SX1272)
            return RegModemConfig.sx1272bits.CodingRate;
        else
            return 0;
    }
}

bool getHeaderMode(void)
{
    if (xcvr_type == SX1276) {
        RegModemConfig.octet = read_reg(REG_LR_MODEMCONFIG);
        return RegModemConfig.sx1276bits.ImplicitHeaderModeOn;
    } else if (xcvr_type == SX1272) {
        RegModemConfig.octet = read_reg(REG_LR_MODEMCONFIG);
        return RegModemConfig.sx1272bits.ImplicitHeaderModeOn;
    } else
        return false;
}

float get_frf_MHz(void)
{
    uint32_t frf;
    uint8_t lsb, mid, msb;
    float MHz;
    
    msb = read_reg(REG_FRFMSB);
    mid = read_reg(REG_FRFMID);
    lsb = read_reg(REG_FRFLSB);
    frf = msb;
    frf <<= 8;
    frf += mid;
    frf <<= 8;
    frf += lsb;
    
    MHz = frf * FREQ_STEP_MHZ;
    
    if (MHz < 525)
        HF = false;
    else
        HF = true;
        
    return MHz;
}

bool getRxPayloadCrcOn(void)
{
    /* RxPayloadCrcOn enables CRC generation in transmitter */
    /* in implicit mode, this bit also enables CRC in receiver */
    if (xcvr_type == SX1276) {
        RegModemConfig2.octet = read_reg(REG_LR_MODEMCONFIG2);
        return RegModemConfig2.sx1276bits.RxPayloadCrcOn;
    } else if (xcvr_type == SX1272) {
        RegModemConfig.octet = read_reg(REG_LR_MODEMCONFIG);
        return RegModemConfig.sx1272bits.RxPayloadCrcOn;
    } else
        return 0;
}

uint16_t read_u16(uint8_t addr)
{
    uint16_t ret;
    // Select the device by seting chip select low
    ASSERT_SX127X_NSS;

    spi_transfer(addr); // bit7 is low for reading from radio
 
    // Send a dummy byte to receive the contents of register
    ret = spi_transfer(0x00);
    ret <<= 8;
    ret += spi_transfer(0x00);
 
    // Deselect the device
    UNASSERT_SX127X_NSS;
    
    return ret;
}

int get_current_rssi()
{
    uint8_t v = read_reg(REG_LR_RSSIVALUE);
    if (xcvr_type == SX1276) {
        if (HF)
            return v - 157;
        else
            return v - 164;
    } else
        return v - 139;
}

bool getAgcAutoOn(void)
{
    if (xcvr_type == SX1276) {
        RegModemConfig3.octet = read_reg(REG_LR_MODEMCONFIG3);
        return RegModemConfig3.sx1276bits.AgcAutoOn;
    } else if (xcvr_type == SX1272) {
        RegModemConfig2.octet = read_reg(REG_LR_MODEMCONFIG2);
        return RegModemConfig2.sx1272bits.AgcAutoOn;
    } else
        return 0;
}
