#include <stdint.h>
#include <stdbool.h>

typedef struct {
    void (*DioPin_top_half)(void);
    /*!
     * \brief  Tx Done callback prototype.
     */
    void    (*TxDone_topHalf)(void);    // read irqAt for timestamp of interrupt
    void    (*TxDone_botHalf)(void);    // read irqAt for timestamp of interrupt
    /*!
     * \brief  Tx Timeout callback prototype.
     */
    void    ( *TxTimeout )( void );
    /*!
     * \brief Rx Done callback prototype.
     *
     * \param [IN] payload Received buffer pointer
     * \param [IN] size    Received buffer size
     * \param [IN] rssi    RSSI value computed while receiving the frame [dBm]
     * \param [IN] snr     Raw SNR value given by the radio hardware
     *                     FSK : N/A ( set to 0 )
     *                     LoRa: SNR value in dB
     * \param [IN] curTime captured time at RxDone event occurance
     */
    //void    ( *RxDone )(uint16_t size, int16_t rssi, int8_t snr);
    void    ( *RxDone )(uint8_t size, float rssi, float snr);    // read radio.rx_buf for payload, irqAt for timestamp of interrupt
    /*!
     * \brief  Rx Timeout callback prototype.
     */
    void    ( *RxTimeout )( void );
    /*!
     * \brief Rx Error callback prototype.
     */
    void    ( *RxError )( void );
    /*!
     * \brief  FHSS Change Channel callback prototype.
     *
     * \param [IN] currentChannel   Index number of the current channel
     */
    void ( *FhssChangeChannel )( uint8_t currentChannel );
 
    /*!
     * \brief CAD Done callback prototype.
     *
     * \param [IN] channelDetected    Channel Activity detected during the CAD
     */
    void ( *CadDone ) ( bool channelActivityDetected ); 
} RadioEvents_t;

typedef struct {
    void (*init)(const RadioEvents_t*);
     
    void (*standby)(void);
    void (*loRaModemConfig)(unsigned bwKHz, uint8_t sf, uint8_t cr);
    void (*setChannel)(unsigned hz);

    void (*set_tx_dbm)(int8_t dbm);
     
                   // preambleLen, fixLen, crcOn, invIQ
    void (*loRaPacketConfig)(unsigned preambleLen, bool fixLen, bool crcOn, bool invIQ);
    int (*send)(uint8_t size/*, timestamp_t maxListenTime, timestamp_t channelFreeTime, int rssiThresh*/);

    void (*printOpMode)(void);
    bool (*service)(void);
    void (*rx)(unsigned timeout);
    void (*irqTopHalf)(void);
    uint16_t irq_pin;
    void (*postcfgreadback)(void);
    bool (*bw_at_highest)(void);
    bool (*bw_at_lowest)(void);
    bool (*sf_at_slowest)(void);
    bool (*sf_at_fastest)(void);

    uint8_t *rx_buf;
    uint8_t *tx_buf;
} lorahal_t;

extern lorahal_t lorahal;

void sethal_lr20xx(void);
void step_bw_lr20xx(bool up);
void step_sf_lr20xx(bool up);
uint16_t get_bw_khz_lr20xx(void);
uint8_t get_sf_lr20xx(void);
uint32_t get_freq_hz_lr20xx(void);
uint8_t get_chip_mode_lr20xx(void);
void print_streaming_timing_analysis(void);

/* Streaming TX functions */
int Send_lr20xx_streaming(uint8_t total_size, uint8_t initial_bytes);
uint8_t Send_lr20xx_fifo_continue(void);

/* Streaming TX state (defined in radio_lr20xx.c) */
extern volatile uint8_t tx_fifo_idx;
extern volatile uint8_t tx_total_size;
extern volatile uint8_t streaming_tx_active;
