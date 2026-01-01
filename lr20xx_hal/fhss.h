/**
 * @file fhss.h
 * @brief Frequency Hopping Spread Spectrum for FCC Part 15 compliance
 *
 * FCC Part 15 requirements for bandwidths below 500kHz in USA:
 * - 50 channels in 902-928 MHz band
 * - Random hopping across all channels
 * - Maximum 400ms dwell time per channel
 *
 * Synchronization strategy:
 * - TX: Long preamble on random channel
 * - RX: CAD sweep to find preamble
 */

#ifndef FHSS_H
#define FHSS_H

#include <stdint.h>
#include <stdbool.h>

/* FCC Part 15.247 requirements */
#define FHSS_NUM_CHANNELS       50
#define FHSS_MAX_DWELL_MS       400

/* RX timeout threshold before returning to CAD scan */
#define FHSS_RX_TIMEOUT_MAX     3   /* Max consecutive timeouts before rescan */

/* Sync packet format - sent by TX to synchronize RX LFSR (explicit header mode) */
#define FHSS_SYNC_MARKER        0xAA
typedef struct __attribute__((packed)) {
    uint8_t  marker;           /* Sync marker (0xAA) */
    uint16_t lfsr_state;       /* Current LFSR state for channel hopping */
    uint8_t  next_channel;     /* Next channel TX will hop to */
    uint8_t  hop_count;        /* Number of hops since LFSR state - RX fast-forwards */
} fhss_sync_pkt_t;

#define FHSS_SYNC_PKT_SIZE      sizeof(fhss_sync_pkt_t)

/* Data packet header - prepended to codec2 data (implicit header mode)
 * The packet uses implicit mode with fixed length for efficiency.
 * Total packet = header (3 bytes) + codec2 payload */
#define FHSS_DATA_MARKER        0x55
typedef struct __attribute__((packed)) {
    uint8_t  marker;           /* Data marker (0x55) - distinguishes from sync */
    uint8_t  seq_num;          /* Sequence number for lost packet detection */
    uint8_t  channel;          /* Current channel - RX can verify/resync */
} fhss_data_hdr_t;

#define FHSS_DATA_HDR_SIZE      sizeof(fhss_data_hdr_t)
#define FHSS_DATA_PREAMBLE_LEN  8   /* Short preamble for data packets */

/* Number of sync packets to send before data mode.
 * Multiple syncs give RX more chances to catch sync if stuck on false CAD.
 * Each sync is on a different random channel. */
#define FHSS_SYNC_REPEAT_COUNT  3

/* Channel plan for 902-928 MHz band with 125kHz bandwidth
 * Channel spacing: 520kHz (26MHz / 50 channels)
 * Start frequency: 902.2 MHz (guard band from 902 MHz edge)
 * End frequency:   927.7 MHz (guard band from 928 MHz edge)
 */
#define FHSS_BASE_FREQ_HZ       902200000UL   /* 902.2 MHz */
#define FHSS_CHANNEL_STEP_HZ    520000UL      /* 520 kHz spacing for 50 channels */
#define FHSS_BAND_END_HZ        928000000UL   /* 928 MHz band edge */

/* CAD (Channel Activity Detection) configuration
 * Used for RX to quickly sweep channels looking for TX preamble
 */
typedef struct {
    uint8_t  cad_symb_nb;       /* Number of symbols for CAD (1-4) */
    uint8_t  cad_detect_peak;   /* Detection threshold (SF-dependent) */
    uint8_t  pnr_delta;         /* Best-effort CAD: 0=disabled, 8=enabled */
    uint32_t cad_timeout_ms;    /* Timeout after CAD detection to wait for packet */
} fhss_cad_config_t;

/* FHSS state machine states */
typedef enum {
    FHSS_STATE_IDLE,           /* Not hopping */
    FHSS_STATE_TX_PREAMBLE,    /* TX: Sending long preamble for sync */
    FHSS_STATE_TX_DATA,        /* TX: Sending data packets */
    FHSS_STATE_RX_SCAN,        /* RX: CAD scanning for preamble */
    FHSS_STATE_RX_SYNC,        /* RX: Found preamble, receiving packet */
    FHSS_STATE_RX_DATA,        /* RX: Synchronized, receiving data */
} fhss_state_t;

/* FHSS statistics */
typedef struct {
    uint32_t cad_done_count;      /* Number of CAD operations completed */
    uint32_t cad_detected_count;  /* Number of CAD detections */
    uint32_t cad_false_count;     /* Number of false CAD detections */
    uint32_t channels_scanned;    /* Total channels scanned */
    uint32_t sync_found_count;    /* Number of successful synchronizations */
    uint32_t sync_time_ms;        /* Last sync acquisition time */
    uint32_t sweep_start_ms;      /* Timestamp when current sweep started */
    uint32_t sweep_count;         /* Number of complete sweeps */
    uint32_t last_sweep_ms;       /* Duration of last complete sweep */
    uint32_t rx_timeout_count;    /* Total RX timeouts during data mode */
    uint32_t resync_count;        /* Number of times RX returned to scan */
} fhss_stats_t;

/* FHSS configuration */
typedef struct {
    bool     enabled;             /* FHSS enabled flag */
    bool     tx_continuous;       /* Continuous preamble TX mode */
    uint16_t preamble_len_symb;   /* Preamble length in symbols for sync TX */
    uint8_t  current_channel;     /* Current channel index (0-49) */
    uint8_t  tx_next_channel;     /* Next channel promised in sync packet (for first hop) */
    uint8_t  tx_sync_count;       /* Number of sync packets sent (for repeat mechanism) */
    uint16_t tx_sync_lfsr;        /* LFSR state to send in sync packets (saved at first sync) */
    uint8_t  data_pkt_len;        /* Fixed payload length for data packets (implicit mode) */
    uint8_t  tx_seq_num;          /* TX sequence number */
    uint8_t  rx_seq_num;          /* Expected RX sequence number */
    uint32_t dwell_start_ms;      /* Timestamp when current channel dwell started */
    uint16_t pkts_on_channel;     /* Packets sent/received on current channel */
    uint8_t  rx_timeout_consec;   /* Consecutive RX timeouts (reset on RxDone) */
    bool     rx_first_dwell;      /* RX: first dwell after sync (extended threshold) */
    uint8_t  preamble_confirm_count; /* Preamble confirmations during RX_SYNC */
    fhss_cad_config_t cad_cfg;    /* CAD configuration */
    fhss_state_t state;           /* Current state */
    fhss_stats_t stats;           /* Statistics */
} fhss_config_t;

/* Global FHSS configuration */
extern fhss_config_t fhss_cfg;

/* Initialize FHSS module */
void fhss_init(void);

/* Get frequency in Hz for a given channel index (0-49) */
uint32_t fhss_get_channel_freq(uint8_t channel);

/* Set radio to a specific channel */
void fhss_set_channel(uint8_t channel);

/* Select a random channel */
uint8_t fhss_random_channel(void);

/* Hop to next random channel */
void fhss_hop(void);

/* CAD operations */
void fhss_configure_cad(const fhss_cad_config_t *cfg);
void fhss_start_cad(void);
void fhss_cad_done_handler(bool detected);
void fhss_preamble_detected_handler(void);
void fhss_timeout_handler(void);

/* RX sync scan - sweep channels looking for TX preamble */
void fhss_start_scan(void);
void fhss_scan_next_channel(void);
bool fhss_is_scanning(void);

/* TX sync - send long preamble on random channel */
void fhss_start_tx_sync(void);

/* TX done handler - call from TxDone callback for continuous mode */
void fhss_tx_done_handler(void);

/* Process received sync packet - extracts LFSR state to synchronize hopping
 * Returns true if valid sync packet, false otherwise */
bool fhss_rx_sync_packet(const uint8_t *data, uint8_t size);

/* Get current LFSR state (for debugging/display) */
uint16_t fhss_get_lfsr_state(void);

/* Set LFSR state (for synchronization) */
void fhss_set_lfsr_state(uint16_t state);

/* Configure data packet mode (implicit header, fixed length)
 * Call after sync to set up for data transmission/reception
 * payload_len: codec2 payload size (header added automatically) */
void fhss_configure_data_mode(uint8_t payload_len);

/* Configure RX expected data packet length
 * Call before starting scan to set expected codec2 payload size
 * This is used to configure implicit header mode after sync */
void fhss_set_rx_payload_len(uint8_t payload_len);

/* Send data packet with automatic hopping
 * data: codec2 payload (header prepended automatically)
 * len: payload length (must match configured data_pkt_len)
 * Returns: 0 on success, -1 on error */
int fhss_send_data(const uint8_t *data, uint8_t len);

/* Process received data packet
 * Returns: payload length (excluding header) on success, -1 on error
 * Checks sequence number for lost packets */
int fhss_rx_data_packet(const uint8_t *data, uint8_t size);

/* Check if hop is needed (dwell time exceeded) and perform hop
 * Call periodically during data transfer */
void fhss_check_hop(void);

/* Start RX for next data packet on current or next channel */
void fhss_rx_data(void);

/* Handle RX timeout during data mode
 * Continues hopping for N channels, then returns to CAD scan */
void fhss_rx_data_timeout_handler(void);

/* Check if FHSS is in TX sync (preamble) mode */
bool fhss_is_tx_sync(void);

/* Check if FHSS is ready to send data (sync complete) */
bool fhss_is_tx_data_ready(void);

/* Stop FHSS TX and return to idle */
void fhss_tx_stop(void);

/* Get recommended CAD parameters for given SF */
void fhss_get_cad_params_for_sf(uint8_t sf, uint8_t cad_symb_nb,
                                 uint8_t *detect_peak);

/* Calculate required preamble length for sync
 * Based on CAD sweep time across all 50 channels */
uint16_t fhss_calc_sync_preamble_len(uint8_t sf, uint16_t bw_khz);

/* Print FHSS status and statistics */
void fhss_print_status(void);
void fhss_print_stats(void);

/* UART command interface for CAD parameter tuning */
void fhss_uart_command(char cmd);

/* Fast polling for CAD scan mode - tight loop until CAD done.
 * Returns true if should continue scanning, false if detected or stopped.
 * Call from main loop when in scan mode for fast channel hopping. */
bool fhss_poll(void);

#endif /* FHSS_H */
