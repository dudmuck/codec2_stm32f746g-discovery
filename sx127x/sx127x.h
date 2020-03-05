#include <stdint.h>
#include <stdbool.h>

#define XTAL_FREQ   32000000

#define FREQ_STEP_MHZ     61.03515625e-6    // 32 / (2^19)
#define FREQ_STEP_KHZ     61.03515625e-3    // 32e3 / (2^19)
#define FREQ_STEP_HZ      61.03515625       // 32e6 / (2^19)

//#define MHZ_TO_FRF(m)   (m / FREQ_STEP_MHZ)

/*!
 * SX127x Internal registers Address
 */
#define REG_FIFO                                    0x00
#define REG_OPMODE                                  0x01
#define REG_FRFMSB                                  0x06
#define REG_FRFMID                                  0x07
#define REG_FRFLSB                                  0x08
// Tx settings
#define REG_PACONFIG                                0x09
#define REG_PARAMP                                  0x0A
#define REG_OCP                                     0x0B 
// Rx settings
#define REG_LNA                                     0x0C

// LoRa registers
#define REG_LR_FIFOADDRPTR                          0x0d
#define REG_LR_FIFOTXBASEADDR                       0x0e
#define REG_LR_FIFORXBASEADDR                       0x0f
#define REG_LR_FIFORXCURRENTADDR /*REG_LR_RXDATAADDR*/  0x10
#define REG_LR_IRQFLAGSMASK                         0x11
#define REG_LR_IRQFLAGS                             0x12
#define REG_LR_RXNBBYTES                            0x13
#define REG_LR_RXHEADERCNTVALUE_MSB                 0x14
#define REG_LR_RXHEADERCNTVALUE_LSB                 0x15
#define REG_LR_RXPACKETCNTVALUE_MSB                 0x16
#define REG_LR_RXPACKETCNTVALUE_LSB                 0x17
#define REG_LR_MODEMSTAT                            0x18
#define REG_LR_PKTSNRVALUE                          0x19
#define REG_LR_PKTRSSIVALUE                         0x1a
#define REG_LR_RSSIVALUE                            0x1b
#define REG_LR_HOPCHANNEL                           0x1c
#define REG_LR_MODEMCONFIG                          0x1d
#define REG_LR_MODEMCONFIG2                         0x1e
#define REG_LR_SYMBTIMEOUTLSB                       0x1f
#define REG_LR_PREAMBLEMSB                          0x20
#define REG_LR_PREAMBLELSB                          0x21
#define REG_LR_PAYLOADLENGTH                        0x22 // and RX length for implicit
#define REG_LR_RX_MAX_PAYLOADLENGTH                 0x23 // length limit for explicit mode
#define REG_LR_HOPPERIOD                            0x24
#define REG_LR_RXBYTEADDR /*REG_LR_RXDATAADDR*/     0x25
#define REG_LR_MODEMCONFIG3                         0x26    // sx1272 REG_LR_PPM_CORRECTION_MSB
#define REG_LR_PPM_CORRECTION_LSB                   0x27
#define REG_LR_TEST28                               0x28  // est_freq_error
#define REG_LR_TEST29                               0x29    // est_freq_error
#define REG_LR_TEST2A                               0x2a    // est_freq_error
#define REG_LR_TEST2B                               0x2b    // 
#define REG_LR_WIDEBAND_RSSI                        0x2c 
#define REG_LR_AGCH_TH                              0x2d    // agc_upper_th
#define REG_LR_AGCL_TH                              0x2e    // agc_lower_th
#define REG_LR_IFFRQH                               0x2f    // if_freq(12:8)
#define REG_LR_IFFRQL                               0x30    // if_freq(7:0)
#define REG_LR_TEST31                               0x31    // if_freq_auto, ...
#define REG_LR_TEST32                               0x32    // 
#define REG_LR_TEST33                               0x33    // invert IQ
#define REG_LR_CAD_PEAK_TO_NOISE_RATIO              0x34
#define REG_LR_CAD_MIN_PEAK                         0x35
#define REG_LR_SX1276_AUTO_DRIFT                    0x36
#define REG_LR_DETECTION_THRESHOLD                  0x37
#define REG_LR_SYNC_BYTE                            0x39    // default 0x12 (value of 0x21 will isolate network)
#define REG_LR_GAIN_DRIFT                           0x3a
#define REG_LR_DRIFT_INVERT                         0x3b  

/*******************************************************/

#define REG_DIOMAPPING1                             0x40
#define REG_DIOMAPPING2                             0x41
#define REG_VERSION                                 0x42

#define REG_PATEST_SX1276                           0x44
#define REG_PATEST_SX1272                           0x4b
#define REG_PDSTRIM1_SX1276                         0x4d
#define REG_PDSTRIM1_SX1272                         0x5a
#define REG_PLL_SX1272                              0x5c    // RX PLL bandwidth
#define REG_PLL_LOWPN_SX1272                        0x5e
#define REG_PLL_SX1276                              0x70 
#define REG_BSYNCTST2                               0x67

typedef enum {
    SERVICE_NONE = 0,
    SERVICE_ERROR,
    //! request to call read_fifo()
    SERVICE_READ_FIFO,
    //! notification to application of transmit complete
    SERVICE_TX_DONE
} service_action_e;

/******************************************************************************/
 
typedef union {
    struct {    // sx1272 register 0x01
        uint8_t Mode                : 3;    // 0,1,2
        uint8_t ModulationShaping   : 2;    // 3,4  FSK/OOK
        uint8_t ModulationType      : 2;    // 5,6  FSK/OOK
        uint8_t LongRangeMode       : 1;    // 7    change this bit only in sleep mode
    } bits;
    struct {    // sx1276 register 0x01
        uint8_t Mode                : 3;    // 0,1,2
        uint8_t LowFrequencyModeOn  : 1;    // 3    1=access to LF test registers (0=HF regs)
        uint8_t reserved            : 1;    // 4
        uint8_t ModulationType      : 2;    // 5,6  FSK/OOK
        uint8_t LongRangeMode       : 1;    // 7    change this bit only in sleep mode
    } sx1276FSKbits;
    struct {    // sx1276 register 0x01
        uint8_t Mode                : 3;    // 0,1,2
        uint8_t LowFrequencyModeOn  : 1;    // 3    1=access to LF test registers (0=HF regs)
        uint8_t reserved            : 2;    // 4,5
        uint8_t AccessSharedReg     : 1;    // 6    1=FSK registers while in LoRa mode
        uint8_t LongRangeMode       : 1;    // 7    change this bit only in sleep mode
    } sx1276LORAbits;
    uint8_t octet;
} RegOpMode_t;

typedef union {
    struct {    // sx12xx register 0x09
        uint8_t OutputPower : 4;    // 0,1,2,3
        uint8_t MaxPower    : 3;    // 4,5,6
        uint8_t PaSelect    : 1;    // 7        1=PA_BOOST
    } bits;
    uint8_t octet;
} RegPaConfig_t;

typedef union {
    struct {    // sx12xx register 0x0b
        uint8_t OcpTrim : 5;    // 0,1,2,3,4
        uint8_t OcpOn   : 1;    // 5
        uint8_t unused  : 2;    // 6,7
    } bits;
    uint8_t octet;
} RegOcp_t;

typedef union {
    struct {    // sx127x register 0x12
        uint8_t CadDetected         : 1;    // 0
        uint8_t FhssChangeChannel   : 1;    // 1
        uint8_t CadDone             : 1;    // 2
        uint8_t TxDone              : 1;    // 3
        uint8_t ValidHeader         : 1;    // 4
        uint8_t PayloadCrcError     : 1;    // 5
        uint8_t RxDone              : 1;    // 6
        uint8_t RxTimeout           : 1;    // 7
    } bits;
    uint8_t octet;
} RegIrqFlags_t;

typedef union {
    struct {    // sx127x register 0x18
        uint8_t detect          : 1;    // 0
        uint8_t sync            : 1;    // 1
        uint8_t rx_ongoing      : 1;    // 2
        uint8_t header_valid    : 1;    // 3
        uint8_t clear           : 1;    // 4
        uint8_t RxCodingRate    : 3;    // 5,6,7
    } bits;
    uint8_t octet;
} RegModemStatus_t;

typedef union {
    struct {    // sx127x register 0x1c
        uint8_t FhssPresentChannel  : 6;    // 0,1,2,3,4,5
        uint8_t RxPayloadCrcOn      : 1;    // 6
        uint8_t PllTimeout          : 1;    // 7
    } bits;
    uint8_t octet;
} RegHopChannel_t;

typedef union {
    struct {    // sx1276 register 0x1d
        uint8_t ImplicitHeaderModeOn    : 1;    // 0
        uint8_t CodingRate              : 3;    // 1,2,3
        uint8_t Bw                      : 4;    // 4,5,6,7
    } sx1276bits;
    struct {    // sx1272 register 0x1d
        uint8_t LowDataRateOptimize     : 1;    // 0  ppm_offset: number of cyclic shifts possible to encode to symbol
        uint8_t RxPayloadCrcOn          : 1;    // 1
        uint8_t ImplicitHeaderModeOn    : 1;    // 2
        uint8_t CodingRate              : 3;    // 3,4,5
        uint8_t Bw                      : 2;    // 6,7
    } sx1272bits;
    uint8_t octet;
} RegModemConfig_t;

typedef union {
    struct {    // sx1276 register 0x1e
        uint8_t SymbTimeoutMsb          : 2;    // 0,1
        uint8_t RxPayloadCrcOn          : 1;    // 2
        uint8_t TxContinuousMode        : 1;    // 3
        uint8_t SpreadingFactor         : 4;    // 4,5,6,7
    } sx1276bits;
    struct {    // sx1272 register 0x1e
        uint8_t SymbTimeoutMsb          : 2;    // 0,1
        uint8_t AgcAutoOn               : 1;    // 2
        uint8_t TxContinuousMode        : 1;    // 3
        uint8_t SpreadingFactor         : 4;    // 4,5,6,7
    } sx1272bits;
    uint8_t octet;
} RegModemConfig2_t;

typedef union {
    struct {    // sx127x register 0x26
        uint8_t reserved    : 2;    // 0,1
        uint8_t AgcAutoOn   : 1;    // 2
        uint8_t LowDataRateOptimize  : 1;    // 3   ppm_offset, use when symbol duration exceeds 16ms
        uint8_t unused      : 4;    // 4,5,6,7 
    } sx1276bits;
    uint8_t octet;
    uint8_t sx1272_ppm_correction_msb;
} RegModemConfig3_t;

typedef union {
    struct {    // sx127x register 0x31
        uint8_t detect_trig_same_peaks_nb  : 3;    // 0,1,2
        uint8_t disable_pll_timeout        : 1;    // 3
        uint8_t tracking_intergral         : 2;    // 4,5
        uint8_t frame_sync_gain            : 1;    // 6
        uint8_t if_freq_auto               : 1;    // 7
    } bits;
    uint8_t octet;
} RegTest31_t;

typedef union {
    struct {    // sx127x register 0x33
        uint8_t chirp_invert_tx    : 1;    // 0  invert TX spreading sequence  (default=1)
        uint8_t chirp_invert_rx    : 1;    // 1  invert chip direction in RX mode  (default=1)
        uint8_t sync_detect_th     : 1;    // 2  require 6dB despread SNR during preamble
        uint8_t invert_coef_phase  : 1;    // 3  
        uint8_t invert_coef_amp    : 1;    // 4
        uint8_t quad_correction_en : 1;    // 5  enable IQ compensation
        uint8_t invert_i_q         : 1;    // 6  RX invert (default=0)
        uint8_t start_rambist      : 1;    // 7
    } bits;
    uint8_t octet;
} RegTest33_t;

typedef union {
    struct {    // sx1276 register 0x36
        uint8_t freq_to_time_drift_auto         : 1;    // 0  manual control of 0x3a register (1=auto)
        uint8_t sd_max_freq_deviation_auto      : 1;    // 1  manual control of 0x3b[3:1] (1=auto)
        uint8_t reserved                        : 6;    // 
    } bits;
    uint8_t octet;
} RegAutoDrift_t;

typedef union {
    struct {    // sx127x register 0x3a
        uint8_t freq_to_time_drift         : 6;    // 0,1,2,3,4,5  manual control of 0x3a register
        uint8_t preamble_timing_gain       : 1;    // 6,7
    } bits;
    uint8_t octet;
} RegGainDrift_t;

typedef union {
    struct {    // sx127x register 0x3b
        uint8_t coarse_sync                     : 1;    // 0  must be set to 1
        uint8_t fine_sync                       : 1;    // 1  must be clr to 0
        uint8_t invert_timing_error_per_symbol  : 1;    // 2  set to !invert_i_q
        uint8_t invert_freq_error               : 1;    // 3  
        uint8_t invert_delta_sampling           : 1;    // 4    must be set to 1
        uint8_t reserved                        : 1;    // 5    must be clr to 0
        uint8_t invert_fast_timing              : 1;    // 6 
        uint8_t invert_carry_in                 : 1;    // 7
    } bits;
    uint8_t octet;
} RegDriftInvert_t;

/***************************************************************/

typedef union {
    struct {    // sx12xx register 0x40
        uint8_t Dio3Mapping     : 2;    // 0,1
        uint8_t Dio2Mapping     : 2;    // 2,3
        uint8_t Dio1Mapping     : 2;    // 4,5
        uint8_t Dio0Mapping     : 2;    // 6,7 
    } bits;
    uint8_t octet;
} RegDioMapping1_t;

typedef union {
    struct {    // sx12xx register 0x41
        uint8_t MapPreambleDetect : 1;    // 0      //DIO4 assign: 1b=preambleDet 0b=rssiThresh
        uint8_t io_mode           : 3;    // 1,2,3  //0=normal,1=debug,2=fpga,3=pll_tx,4=pll_rx,5=analog
        uint8_t Dio5Mapping       : 2;    // 4,5
        uint8_t Dio4Mapping       : 2;    // 6,7 
    } bits;
    uint8_t octet;
} RegDioMapping2_t;

/***************************************************/
 
typedef union {
    struct {    // sx1272 register 0x5a (sx1276 0x4d)
        uint8_t prog_txdac             : 3;    // 0,1,2     BGR ref current to PA DAC
        uint8_t pds_analog_test        : 1;    // 3      
        uint8_t pds_pa_test            : 2;    // 4,5
        uint8_t pds_ptat               : 2;    // 6,7     leave at 2 (5uA)
    } bits;
    uint8_t octet;
} RegPdsTrim1_t;
 
/**************************************************************************/

typedef enum {
    SX_NONE = 0,
    SX1272,
    SX1276
} type_e;

typedef enum {
    RF_OPMODE_SLEEP = 0,
    RF_OPMODE_STANDBY,          // 1
    RF_OPMODE_SYNTHESIZER_TX,   // 2
    RF_OPMODE_TRANSMITTER,      // 3
    RF_OPMODE_SYNTHESIZER_RX,   // 4
    RF_OPMODE_RECEIVER,         // 5
    RF_OPMODE_RECEIVER_SINGLE,  // 6
    RF_OPMODE_CAD               // 7
} chip_mode_e;

typedef enum {
    SHIELD_TYPE_NONE = 0,
    SHIELD_TYPE_LAS,
    SHIELD_TYPE_MAS,
} shield_type_e;

extern type_e xcvr_type;

void init_sx127x(void);
uint8_t sx127x_getBw(void);
uint8_t sx127x_getSf(void);
void setSf(uint8_t sf);
void sx127x_set_opmode(chip_mode_e mode);
void lora_enable(void);
uint8_t read_reg(uint8_t addr);
uint16_t read_u16(uint8_t addr);
void write_reg(uint8_t addr, uint8_t data);
void write_u16(uint8_t addr, uint16_t data);
void write_u24(uint8_t addr, uint32_t data);
void setBw_KHz(int khz);
void setBw(uint8_t bw);
float get_symbol_period(void);
void set_frf_MHz( float MHz );
float get_frf_MHz(void);
void invert_tx(bool);
void invert_rx(bool);
void start_tx(uint8_t len);
void start_rx(chip_mode_e mode);
service_action_e service(void); // (SLIH) ISR bottom half 
int get_pkt_rssi(void);
uint8_t getCodingRate(bool from_rx);    // false:transmitted, true:last recevied packet
bool getHeaderMode(void);
bool getRxPayloadCrcOn(void);
int get_current_rssi(void);
bool getAgcAutoOn(void);

/* common registers */
extern RegOpMode_t RegOpMode;
extern RegPaConfig_t RegPaConfig;
extern RegOcp_t RegOcp;            // 0x0b

/* lora registers */
extern RegIrqFlags_t       RegIrqFlags;            // 0x12
extern uint8_t             RegRxNbBytes;           // 0x13
extern int8_t              RegPktSnrValue;         // 0x19  signed, s/n can be negative
extern RegHopChannel_t     RegHopChannel;          // 0x1c
extern RegModemConfig_t    RegModemConfig;         // 0x1d
extern RegModemConfig2_t   RegModemConfig2;        // 0x1e
extern uint8_t             RegPayloadLength;       // 0x22
extern uint8_t             RegHopPeriod;           // 0x24
extern RegModemConfig3_t   RegModemConfig3;        // 0x26
extern uint16_t            RegPreamble;            // 0x20->0x21
extern uint8_t             RegRxMaxPayloadLength;  // 0x23
extern RegTest31_t         RegTest31;              // 0x31
extern RegTest33_t         RegTest33;              // 0x33
extern RegAutoDrift_t      RegAutoDrift;           // 0x36  sx1276 only
extern RegGainDrift_t      RegGainDrift;           // 0x3a
extern RegDriftInvert_t    RegDriftInvert;         // 0x3b

extern RegDioMapping1_t RegDioMapping1;
extern RegDioMapping2_t RegDioMapping2;

/* exported variables: */
extern uint8_t SX127x_tx_buf[];    // lora fifo size
extern uint8_t SX127x_rx_buf[];    // lora fifo size
extern shield_type_e shield_type;
