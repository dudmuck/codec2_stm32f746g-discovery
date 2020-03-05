#include "radio.h"
#include <stdio.h>
#include "stm32f7xx_hal.h"
#include "sx127x.h"
#include "pinDefs_sx127x.h"

volatile struct pe {
    uint8_t dio0 : 1;
    uint8_t dio1 : 1;
    uint8_t txing : 1;
} pinEvent;

const RadioEvents_t* RadioEvents;

static void PostConfigReadBack()
{
}

static bool is_bw_at_highest(void)
{
    if (xcvr_type == SX1272)
        return sx127x_getBw() == 2;
    else if (xcvr_type == SX1276)
        return sx127x_getBw() == 9;
    else {
        for (;;) asm("nop");
    }
}

static bool is_bw_at_lowest(void)
{
    return sx127x_getBw() == 0;
}

static bool is_sf_at_slowest()
{
    return sx127x_getSf() == 12;
}

static bool is_sf_at_fastest()
{
    return sx127x_getSf() == 7;
}

static void Init_sx127x(const RadioEvents_t* e)
{
    RadioEvents = e;

    init_sx127x();
}

static void Standby_sx127x()
{
   sx127x_set_opmode(RF_OPMODE_STANDBY);
}

static void LoRaModemConfig_sx127x(unsigned bwKHz, uint8_t sf, uint8_t coderate)
{
    float sp;
    if (!RegOpMode.bits.LongRangeMode)
        lora_enable();
 
    RegModemConfig2.sx1276bits.SpreadingFactor = sf;
    write_reg(REG_LR_MODEMCONFIG2, RegModemConfig2.octet);
 
    setBw_KHz(bwKHz);
 
    if (xcvr_type == SX1276) {
        RegModemConfig.sx1276bits.CodingRate = coderate;
 
        sp = get_symbol_period();
        if (sp > 16)
            RegModemConfig3.sx1276bits.LowDataRateOptimize = 1;
        else
            RegModemConfig3.sx1276bits.LowDataRateOptimize = 0;
 
        write_reg(REG_LR_MODEMCONFIG3, RegModemConfig3.octet);
    } else if (xcvr_type == SX1272) {
        RegModemConfig.sx1272bits.CodingRate = coderate;
 
        if (get_symbol_period() > 16)
            RegModemConfig.sx1272bits.LowDataRateOptimize = 1;
        else
            RegModemConfig.sx1272bits.LowDataRateOptimize = 0;
    }
    write_reg(REG_LR_MODEMCONFIG, RegModemConfig.octet);
}

void SetChannel_sx127x(unsigned hz)
{
    set_frf_MHz(hz / 1000000.0);
}

void LoRaPacketConfig_sx127x(unsigned preambleLen, bool fixLen, bool crcOn, bool invIQ)
{
    RegPreamble = preambleLen;
    write_u16(REG_LR_PREAMBLEMSB, RegPreamble);
 
    if (xcvr_type == SX1276) {
        RegModemConfig.sx1276bits.ImplicitHeaderModeOn = fixLen;
        RegModemConfig2.sx1276bits.RxPayloadCrcOn = crcOn;
        write_reg(REG_LR_MODEMCONFIG2, RegModemConfig2.octet);
    } else if (xcvr_type == SX1272) {
        RegModemConfig.sx1272bits.ImplicitHeaderModeOn = fixLen;
        RegModemConfig.sx1272bits.RxPayloadCrcOn = crcOn;
    }
 
    write_reg(REG_LR_MODEMCONFIG, RegModemConfig.octet);
 
    invert_tx(invIQ);
    invert_rx(invIQ);
}

int Send_sx127x(uint8_t size)
{
    if (RegOpMode.bits.Mode == RF_OPMODE_SLEEP) {
        sx127x_set_opmode(RF_OPMODE_STANDBY);
        //delay_ticks(2);
        HAL_Delay(2);
    }
    write_reg(REG_LR_IRQFLAGS, 0x08); // ensure TxDone is cleared
    RegPayloadLength = size;
    write_reg(REG_LR_PAYLOADLENGTH, RegPayloadLength);
 
#if 0
    if (maxListenTime > 0) {
        int rssi;
        us_timestamp_t startAt, chFreeAt, now;
        lora.start_rx(RF_OPMODE_RECEIVER);
        startAt = lpt.read_us();
Lstart:
        do {
            now = lpt.read_us();
            if ((now - startAt) > maxListenTime) {
                return -1;
            }
            rssi = lora.get_current_rssi();
        } while (rssi > rssiThresh);
        chFreeAt = lpt.read_us();
        do {
            now = lpt.read_us();
            rssi = lora.get_current_rssi();
            if (rssi > rssiThresh) {
                goto Lstart;
            }
        } while ((now - chFreeAt) < channelFreeTime);
    }
#endif /* if 0 */
 
    start_tx(size);
    pinEvent.txing = 1;

    return 0;
}

const char* const opmode_status_strs[] = {
    "SLEEP    ", // 0
    "STANDBY  ", // 1
    "FS_TX    ", // 2
    "TX       ", // 3
    "FS_RX    ", // 4
    "RX       ", // 5
    "RX_SINGLE", // 6
    "CAD      ", // 7
    NULL
};

static void printOpMode_sx127x()
{
    RegOpMode.octet = read_reg(REG_OPMODE);
    printf("%s ", opmode_status_strs[RegOpMode.bits.Mode]);
}

static void ocp(uint8_t ma)
{
    if (ma < 130)
        RegOcp.bits.OcpTrim = (ma - 45) / 5;
    else
        RegOcp.bits.OcpTrim = (ma + 30) / 10;
    write_reg(REG_OCP, RegOcp.octet);
   
    RegOcp.octet = read_reg(REG_OCP);
    if (RegOcp.bits.OcpTrim < 16)
        ma = 45 + (5 * RegOcp.bits.OcpTrim);
    else if (RegOcp.bits.OcpTrim < 28)
        ma = (10 * RegOcp.bits.OcpTrim) - 30;
    else
        ma = 240;
}

static void set_tx_dbm_sx127x(int8_t dbm)
{
    RegPdsTrim1_t pds_trim;
    uint8_t v, adr, pa_test_adr;
 
    if (xcvr_type == SX1276) {
        adr = REG_PDSTRIM1_SX1276;
        pa_test_adr = REG_PATEST_SX1276;
    } else {
        adr = REG_PDSTRIM1_SX1272;
        pa_test_adr = REG_PATEST_SX1272;
    }
       
    v = read_reg(pa_test_adr);
    if (dbm == PA_OFF_DBM) {
        /* for bench testing: prevent overloading receiving station (very low TX power) */
        v &= ~0x20; // turn off pu_regpa_n: disable PA
        write_reg(pa_test_adr, v);
        return;
    } else if ((v & 0x20) == 0) {
        v |= 0x20; // turn on pu_regpa_n: enable PA
        write_reg(pa_test_adr, v);
    }
 
    pds_trim.octet = read_reg(adr);   
 
    if (shield_type == SHIELD_TYPE_LAS)
        RegPaConfig.bits.PaSelect = 1;
    else
        RegPaConfig.bits.PaSelect = 0;
                
    if (RegPaConfig.bits.PaSelect) {
        /* PABOOST used: +2dbm to +17, or +20 */
        if (dbm > 17) {
            if (dbm > 20)
                dbm = 20;
            dbm -= 3;
            pds_trim.bits.prog_txdac = 7;
            write_reg(adr, pds_trim.octet);
            ocp(150);
        } else
            ocp(120);
 
        if (dbm > 1)
                RegPaConfig.bits.OutputPower = dbm - 2;
    } else {
        /* RFO used: -1 to +14dbm */
        ocp(80);
        if (dbm < 15)
            RegPaConfig.bits.OutputPower = dbm + 1;
    }
    write_reg(REG_PACONFIG, RegPaConfig.octet);
 
    RegPaConfig.octet = read_reg(REG_PACONFIG);
    if (RegPaConfig.bits.PaSelect) {
        dbm = RegPaConfig.bits.OutputPower + pds_trim.bits.prog_txdac - 2;
    } else {
        dbm = RegPaConfig.bits.OutputPower - 1;
    }
}

void SX127x_dio0_topHalf()
{
    if (RadioEvents->DioPin_top_half)
        RadioEvents->DioPin_top_half();
 
    if (pinEvent.txing) {
        /* TxDone handling requires low latency */
        if (RadioEvents->TxDone_topHalf)
            RadioEvents->TxDone_topHalf();    // TODO in callback read irqAt for timestamp of interrupt
 
    }
 
    pinEvent.dio0 = 1;
}

static void Rx_sx127x(unsigned timeout)
{
    if (timeout == 0) {
        start_rx(RF_OPMODE_RECEIVER);
    } else {
        start_rx(RF_OPMODE_RECEIVER_SINGLE);
    }
 
    pinEvent.txing = 0;
}

static void
dio0UserContext()
{
    service_action_e act = service();
 
    if (!pinEvent.txing) {
        if (act == SERVICE_READ_FIFO && RadioEvents->RxDone) {
            int8_t rssi;
            float snr = RegPktSnrValue / 4.0;
            
            rssi = get_pkt_rssi();
            if (snr < 0)
                rssi += snr;
            RadioEvents->RxDone(RegRxNbBytes, rssi, snr);
        } 
    } else if (act == SERVICE_TX_DONE) {
        if (RadioEvents->TxDone_botHalf)
            RadioEvents->TxDone_botHalf();
    }
}

bool service_sx127x()
{
    if (pinEvent.dio0) {
        if (HAL_GPIO_ReadPin(DIO0_PORT, DIO0_PIN) == GPIO_PIN_SET) {
            dio0UserContext();
            pinEvent.txing = 0;
        }
        pinEvent.dio0 = 0;
    } else if (HAL_GPIO_ReadPin(DIO0_PORT, DIO0_PIN) == GPIO_PIN_SET) {
        /* fail: missed interrupt */
        SX127x_dio0_topHalf();
    }
 
    /*if (pinEvent.dio1) {
        dio1UserContext();
        pinEvent.dio1 = 0;
    }*/
    return false;
}

void SetLoRaSymbolTimeout_sx127x(uint16_t symbs)
{
    if (!RegOpMode.bits.LongRangeMode)
        lora_enable();
 
    write_reg(REG_LR_SYMBTIMEOUTLSB, symbs & 0xff);
    symbs >>= 8;
    RegModemConfig2.sx1272bits.SymbTimeoutMsb = symbs;
    write_reg(REG_LR_MODEMCONFIG2, RegModemConfig2.octet);
}

void sethal_sx127x()
{
    lorahal.init = Init_sx127x;
    lorahal.standby = Standby_sx127x;
    lorahal.loRaModemConfig = LoRaModemConfig_sx127x;
    lorahal.setChannel = SetChannel_sx127x;
    lorahal.set_tx_dbm = set_tx_dbm_sx127x;
    lorahal.loRaPacketConfig = LoRaPacketConfig_sx127x;
    lorahal.send = Send_sx127x;
    lorahal.printOpMode = printOpMode_sx127x;
    lorahal.service = service_sx127x;
    lorahal.rx = Rx_sx127x;
    lorahal.setLoRaSymbolTimeout = SetLoRaSymbolTimeout_sx127x;
    lorahal.irqTopHalf = SX127x_dio0_topHalf;
    lorahal.irq_pin = DIO0_PIN;
    lorahal.postcfgreadback = PostConfigReadBack;
    lorahal.bw_at_highest = is_bw_at_highest;
    lorahal.bw_at_lowest = is_bw_at_lowest;
    lorahal.sf_at_slowest = is_sf_at_slowest;
    lorahal.sf_at_fastest = is_sf_at_fastest;

    lorahal.rx_buf = SX127x_rx_buf;
    lorahal.tx_buf = SX127x_tx_buf;
}
