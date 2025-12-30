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
} fhss_stats_t;

/* FHSS configuration */
typedef struct {
    bool     enabled;             /* FHSS enabled flag */
    bool     tx_continuous;       /* Continuous preamble TX mode */
    uint16_t preamble_len_symb;   /* Preamble length in symbols for sync TX */
    uint8_t  current_channel;     /* Current channel index (0-49) */
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
