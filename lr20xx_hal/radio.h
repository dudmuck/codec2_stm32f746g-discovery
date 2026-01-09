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
int adjust_sf_for_streaming(void);

/* Streaming TX functions */
int Send_lr20xx_streaming(uint16_t total_size, uint16_t initial_bytes);
uint16_t Send_lr20xx_fifo_continue(void);

/* Streaming TX state (defined in radio_lr20xx.c) */
extern volatile uint16_t tx_fifo_idx;
extern volatile uint16_t tx_total_size;
extern volatile uint16_t tx_bytes_sent;    /* bytes actually transmitted (updated after each TxDone) */
extern volatile uint8_t streaming_tx_active;
extern volatile uint16_t tx_buf_produced;  /* encoder updates this when new frame ready */

/* Streaming TX state machine */
typedef enum {
    STREAM_IDLE,           /* Ready for new packet cycle */
    STREAM_ENCODING,       /* Encoding frames, TX not yet started */
    STREAM_TX_ACTIVE,      /* TX started, still encoding */
    STREAM_WAIT_TX,        /* All frames encoded, waiting for TX to finish */
    STREAM_BUFFERING_NEXT  /* Buffering next packet while current TX in progress */
} stream_state_t;

extern volatile stream_state_t stream_state;
extern volatile uint32_t stream_underflow_count;

/* Streaming TX configuration */
void calculate_streaming_config(void);
uint8_t apply_streaming_sf(void);
uint16_t get_streaming_initial_bytes(void);

/* Streaming config structure (read-only access) */
typedef struct {
    uint8_t min_initial_frames;
    uint8_t recommended_sf;
    uint8_t max_payload_bytes;
    uint8_t frames_per_packet;
    uint8_t packets_per_frame;  /* Number of LoRa packets per Opus frame (1 for normal, 2+ for 64K/96K) */
    int32_t margin_ms;
    uint8_t streaming_feasible;
} streaming_config_t;
extern streaming_config_t streaming_cfg;
