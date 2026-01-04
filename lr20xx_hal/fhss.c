/**
 * @file fhss.c
 * @brief Frequency Hopping Spread Spectrum implementation for LR20xx
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fhss.h"
#include "radio.h"
#include "lr20xx_radio_lora.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_radio_fifo.h"
#include "lr20xx_system.h"
#include "lr20xx_hal.h"

/* Access chip status from lr20xx_hal.c */
extern lr20xx_stat_t stat;

/* TX defer threshold - defer packets when dwell time exceeds this percentage.
 * This ensures TX hops before sending the last packet, giving RX time to hop first.
 * Set to 95% - just before the hard FCC limit. The 90% check in fhss_check_hop()
 * handles normal hopping; this is only for edge cases.
 * Note: In streaming mode, packet gaps (TOA + buffer refill) can be ~500ms,
 * so 85% (340ms) was too aggressive - triggered after just 1 packet. */
#define FHSS_TX_DEFER_THRESHOLD_PCT  95

/* Global FHSS configuration */
fhss_config_t fhss_cfg;

/* TX packet deferral buffer - holds one packet to be sent after hop */
static uint8_t deferred_pkt_buf[256];  /* Max packet size (header + payload) */
static uint8_t deferred_pkt_len = 0;  /* 0 = no deferred packet */

/* TX hop counter - tracks hops since sync was sent.
 * This allows RX to fast-forward the LFSR if sync is received late. */
static uint8_t tx_hop_count = 0;

/* Re-sync interval - send re-sync packet every N hops during data mode.
 * This gives RX periodic opportunities to catch up if it missed initial sync.
 * 3 hops = ~1.2 seconds between re-syncs. */
#define FHSS_RESYNC_HOP_INTERVAL 3

/* Extended first dwell: TX and RX stay on first channel longer after sync.
 * TX sends this many packets before starting normal hop timer.
 * RX uses this as proactive hop threshold on first channel.
 * 10 packets = ~1 second of audio at 100ms/packet. */
#define FHSS_FIRST_DWELL_PKT_COUNT 10

/* Initial wait period after sync (ms).
 * RX should stay on the promised next_ch while TX finishes sync repeats.
 * TX sends up to 3 sync repeats with ~350ms preamble each (~1050ms total).
 * After syncs, TX starts data mode and sends 2-3 packets (~500ms) before hopping.
 * Total time from first sync to TX hopping = ~1500ms max.
 * Using 1500ms as the initial wait. If RX catches sync 3, TX is almost
 * immediately on next_ch. If RX catches sync 1, TX needs ~1200ms for syncs. */
#define FHSS_RX_INITIAL_WAIT_MS  1500

/* Flag indicating TX should send re-sync packet before next data packet.
 * Set by fhss_hop() when hop_count reaches FHSS_RESYNC_HOP_INTERVAL. */
static bool tx_resync_pending = false;

/* Forward declaration for re-sync function */
static void fhss_send_resync_packet(void);

/* Timestamp when RX started after CAD detection (for preamble timing analysis) */
static uint32_t cad_rx_start_ms = 0;

/* Print and reset radio RX statistics (header errors, CRC errors, etc.)
 * Called when returning to CAD scan after a data reception session. */
static void fhss_print_and_reset_rx_stats(void)
{
    lr20xx_radio_lora_rx_statistics_t stats;
    if (lr20xx_radio_lora_get_rx_statistics(NULL, &stats) == LR20XX_STATUS_OK) {
        /* Only print if there were any events */
        if (stats.n_received_packets > 0 || stats.n_crc_errors > 0 ||
            stats.n_header_errors > 0 || stats.n_false_synchronisation > 0) {
            printf("RX stats: rx=%u crc_err=%u hdr_err=%u false_sync=%u\r\n",
                   stats.n_received_packets, stats.n_crc_errors,
                   stats.n_header_errors, stats.n_false_synchronisation);
        }
        lr20xx_radio_common_reset_rx_stats(NULL);
    }
}

/* Simple LFSR-based pseudo-random number generator for channel selection */
static uint16_t lfsr_state = 0xACE1;

static uint16_t lfsr_random(void)
{
    uint16_t bit = ((lfsr_state >> 0) ^ (lfsr_state >> 2) ^
                    (lfsr_state >> 3) ^ (lfsr_state >> 5)) & 1;
    lfsr_state = (lfsr_state >> 1) | (bit << 15);
    return lfsr_state;
}

/* Recommended CAD detect_peak values from LR20xx datasheet
 * Table indexed by SF (5-12) and cad_symb_nb (1-4)
 */
static const uint8_t cad_detect_peak_table[8][4] = {
    /* SF5  */ { 60, 56, 51, 51 },
    /* SF6  */ { 60, 56, 51, 51 },
    /* SF7  */ { 60, 56, 52, 51 },
    /* SF8  */ { 64, 58, 54, 54 },
    /* SF9  */ { 64, 58, 56, 56 },
    /* SF10 */ { 66, 60, 60, 60 },
    /* SF11 */ { 70, 64, 60, 60 },
    /* SF12 */ { 74, 68, 65, 64 },
};

void fhss_init(void)
{
    fhss_cfg.enabled = true;  /* Enabled by default when built with ENABLE_HOPPING */
    fhss_cfg.tx_continuous = false;
    fhss_cfg.current_channel = 0;
    fhss_cfg.tx_next_channel = 0;  /* Set by fhss_start_tx_sync() */
    fhss_cfg.state = FHSS_STATE_IDLE;
    fhss_cfg.preamble_len_symb = 128;  /* Default sync preamble length */

    /* Data packet configuration */
    fhss_cfg.data_pkt_len = 0;  /* Set by fhss_configure_data_mode() */
    fhss_cfg.tx_seq_num = 0;
    fhss_cfg.rx_seq_num = 0;
    fhss_cfg.dwell_start_ms = 0;
    fhss_cfg.pkts_on_channel = 0;
    /* proactive_hop_count is set by fhss_set_proactive_hop_count() based on codec2 timing.
     * Only set a default if it wasn't already configured. Default=1 is conservative
     * (RX hops after every packet). Must never be 0 - would cause continuous hopping. */
    if (fhss_cfg.proactive_hop_count == 0) {
        fhss_cfg.proactive_hop_count = 1;
    }
    fhss_cfg.rx_timeout_consec = 0;
    fhss_cfg.rx_sync_achieved_ms = 0;  /* Set when sync is achieved */
    fhss_cfg.rx_first_dwell = false;  /* Set true when RX enters data mode */
    fhss_cfg.preamble_confirm_count = 0;

    /* ACK mode configuration */
    fhss_cfg.tx_sync_channel = 0;
    fhss_cfg.rx_sync_channel = 0;
    fhss_cfg.sync_attempts = 0;
    fhss_cfg.ack_wait_start_ms = 0;
    fhss_cfg.ack_received = false;
    fhss_cfg.ack_mode_enabled = true;   /* Enabled by default for reliable sync */

    /* Default CAD configuration: 4 symbols for reliable detection, best-effort enabled */
    fhss_cfg.cad_cfg.cad_symb_nb = 4;
    fhss_cfg.cad_cfg.pnr_delta = 8;  /* Best-effort CAD enabled */
    fhss_cfg.cad_cfg.cad_timeout_ms = 0;  /* 0 = calculate dynamically based on SF/BW */

    /* Clear statistics */
    fhss_cfg.stats.cad_done_count = 0;
    fhss_cfg.stats.cad_detected_count = 0;
    fhss_cfg.stats.cad_false_count = 0;
    fhss_cfg.stats.channels_scanned = 0;
    fhss_cfg.stats.sync_found_count = 0;
    fhss_cfg.stats.sync_time_ms = 0;
    fhss_cfg.stats.sweep_start_ms = 0;
    fhss_cfg.stats.sweep_count = 0;
    fhss_cfg.stats.last_sweep_ms = 0;
    fhss_cfg.stats.rx_timeout_count = 0;
    fhss_cfg.stats.resync_count = 0;

    /* Seed LFSR with a varying value (could use ADC noise, timer, etc.) */
    extern uint32_t HAL_GetTick(void);
    lfsr_state = (uint16_t)(HAL_GetTick() ^ 0xACE1);
    if (lfsr_state == 0) lfsr_state = 0xACE1;

    printf("FHSS initialized: %u channels, %.1f MHz - %.1f MHz\r\n",
           FHSS_NUM_CHANNELS,
           (float)FHSS_BASE_FREQ_HZ / 1000000.0f,
           (float)(FHSS_BASE_FREQ_HZ + (FHSS_NUM_CHANNELS - 1) * FHSS_CHANNEL_STEP_HZ) / 1000000.0f);
}

void fhss_set_proactive_hop_count(uint32_t production_time_ms, uint32_t pkt_toa_ms)
{
    /* FCC Part 15.247: Max 400ms of TRANSMIT time per channel, not wall-clock time.
     * The device can stay on a channel indefinitely; only actual TX counts.
     *
     * packets_per_channel = floor(400ms / pkt_toa_ms)
     * With 175ms TOA: 400/175 = 2.28 -> 2 packets per channel (350ms TX time)
     */
    uint8_t count = FHSS_MAX_DWELL_MS / pkt_toa_ms;

    /* Cap at reasonable values */
    if (count < 1) count = 1;
    if (count > 3) count = 3;  /* Cap to handle packet loss gracefully */

    fhss_cfg.proactive_hop_count = count;
    printf("FHSS: proactive hop count = %u (prod=%lu ms, toa=%lu ms)\r\n",
           count, production_time_ms, pkt_toa_ms);
}

uint32_t fhss_get_channel_freq(uint8_t channel)
{
    if (channel >= FHSS_NUM_CHANNELS)
        channel = 0;
    return FHSS_BASE_FREQ_HZ + (uint32_t)channel * FHSS_CHANNEL_STEP_HZ;
}

void fhss_set_channel(uint8_t channel)
{
    if (channel >= FHSS_NUM_CHANNELS)
        channel = 0;
    fhss_cfg.current_channel = channel;
    uint32_t freq = fhss_get_channel_freq(channel);
    lorahal.setChannel(freq);
}

uint8_t fhss_random_channel(void)
{
    return lfsr_random() % FHSS_NUM_CHANNELS;
}

void fhss_hop(void)
{
    uint8_t new_channel;
    uint16_t lfsr_before = fhss_get_lfsr_state();
    uint8_t loop_count = 0;

    /* Select a different random channel */
    do {
        new_channel = fhss_random_channel();
        loop_count++;
    } while (new_channel == fhss_cfg.current_channel);

    /* Increment hop counter during TX data mode.
     * This is included in periodic re-sync packets so RX can fast-forward. */
    if (fhss_cfg.state == FHSS_STATE_TX_DATA) {
        tx_hop_count++;

        /* Trigger re-sync at interval so RX can catch up if it missed initial sync */
        if ((tx_hop_count % FHSS_RESYNC_HOP_INTERVAL) == 0) {
            tx_resync_pending = true;
        }
    }

    fhss_set_channel(new_channel);
}

void fhss_get_cad_params_for_sf(uint8_t sf, uint8_t cad_symb_nb,
                                 uint8_t *detect_peak)
{
    if (sf < 5) sf = 5;
    if (sf > 12) sf = 12;
    if (cad_symb_nb < 1) cad_symb_nb = 1;
    if (cad_symb_nb > 4) cad_symb_nb = 4;

    *detect_peak = cad_detect_peak_table[sf - 5][cad_symb_nb - 1];
}

/* Calculate initial CAD timeout in ms - short timeout for false CAD detection.
 * This is just enough time to confirm preamble (~16 symbols).
 * Symbol time (ms) = 2^SF / BW_kHz
 */
uint32_t fhss_calc_cad_timeout_ms(uint8_t sf, uint16_t bw_khz)
{
    /* Calculate symbol time in microseconds: (2^sf * 1000) / bw_khz */
    uint32_t symbol_time_us = ((1UL << sf) * 1000UL) / bw_khz;

    /* 16 symbol periods + overhead - enough to confirm preamble
     * but short enough for quick false CAD recovery */
    uint32_t timeout_ms = (16 * symbol_time_us + 999) / 1000 + 3;

    /* Minimum 10ms, maximum 50ms for initial timeout */
    if (timeout_ms < 10)
        timeout_ms = 10;
    if (timeout_ms > 50)
        timeout_ms = 50;

    return timeout_ms;
}

/* Two-stage timeout for sync packet reception after CAD detection.
 * Stage 1: Short timeout to quickly detect false CAD (no preamble IRQ)
 * Stage 2: Extended timeout after preamble confirmed (restart RX from STBY_XOSC)
 *
 * PREAMBLE_DETECTED IRQ fires within ~10-20ms if real signal present.
 * False CAD penalty is only 50ms instead of 400ms.
 *
 * NOTE: Must use standbyXosc() not standby() for fast restart - STBY_RC requires
 * XOSC warmup (~5ms) which would lose preamble lock. STBY_XOSC is ~100us. */
/* CAD verify timeout: After CAD detection, wait this long for first preamble confirm.
 * Shorter = faster false CAD recovery, but might miss weak preambles.
 * With 50 channels and 350ms preamble, we need fast recovery to sweep all channels.
 * SF8/125kHz: symbol time = 2ms, 8 symbols for CAD confirm = 16ms minimum */
#define CAD_VERIFY_TIMEOUT_MS   25    /* Initial short timeout */
#define CAD_EXTENDED_TIMEOUT_MS 400   /* Extended timeout after preamble confirmed */

/* Calculate timeout for sync packet reception.
 * Must be long enough for preamble + payload.
 */
static uint32_t fhss_calc_sync_packet_timeout_ms(uint8_t sf, uint16_t bw_khz)
{
    uint32_t symbol_time_us = ((1UL << sf) * 1000UL) / bw_khz;

    /* Sync packet duration:
     * - Preamble: preamble_len_symb symbols
     * - Sync word: 2.25 symbols
     * - Header: 8 symbols (explicit mode)
     * - Payload: 4 bytes (~20 symbols)
     * Add 20% margin for timing variations.
     */
    uint32_t preamble_time_us = fhss_cfg.preamble_len_symb * symbol_time_us;
    uint32_t packet_time_us = 30 * symbol_time_us;  /* Sync word + header + payload */

    /* Total time with 20% margin - keep it tight to minimize false CAD penalty */
    uint32_t total_us = (preamble_time_us + packet_time_us) * 6 / 5;
    uint32_t timeout_ms = (total_us + 999) / 1000;

    /* Minimum 80ms (for short preambles), maximum 500ms */
    if (timeout_ms < 80)
        timeout_ms = 80;
    if (timeout_ms > 500)
        timeout_ms = 500;

    return timeout_ms;
}

void fhss_configure_cad(const fhss_cad_config_t *cfg)
{
    lr20xx_radio_lora_cad_params_t cad_params;
    extern uint8_t get_sf_lr20xx(void);
    extern uint16_t get_bw_khz_lr20xx(void);
    uint8_t sf = get_sf_lr20xx();
    uint16_t bw_khz = get_bw_khz_lr20xx();

    fhss_cfg.cad_cfg = *cfg;

    /* Get recommended detect_peak for current SF */
    fhss_get_cad_params_for_sf(sf, cfg->cad_symb_nb,
                                &fhss_cfg.cad_cfg.cad_detect_peak);

    /* Calculate initial CAD timeout (short, for false CAD detection) */
    if (cfg->cad_timeout_ms == 0 || cfg->cad_timeout_ms == 100) {
        fhss_cfg.cad_cfg.cad_timeout_ms = fhss_calc_cad_timeout_ms(sf, bw_khz);
    }

    cad_params.cad_symb_nb = cfg->cad_symb_nb;
    cad_params.pnr_delta = cfg->pnr_delta;
    cad_params.cad_detect_peak = fhss_cfg.cad_cfg.cad_detect_peak;

    /* CAD exit to RX mode: on CAD detection, radio automatically enters RX.
     * The cad_timeout_in_pll_step becomes the RX timeout.
     * This avoids XOSC warmup delay and preamble lock loss. */
    cad_params.cad_exit_mode = LR20XX_RADIO_LORA_CAD_EXIT_MODE_RX;

    /* RX timeout after CAD detection - must cover remaining preamble + sync packet */
    cad_params.cad_timeout_in_pll_step = (CAD_EXTENDED_TIMEOUT_MS * 1000) / 31;
    if (cad_params.cad_timeout_in_pll_step > 0x00FFFFFF)
        cad_params.cad_timeout_in_pll_step = 0x00FFFFFF;

    /* Must be in standby to configure CAD parameters.
     * Use XOSC standby for faster FS transitions during scanning. */
    lorahal.standbyXosc();
    lr20xx_radio_lora_configure_cad_params(NULL, &cad_params);

    /* Configure LoRa synch timeout for long sync preamble detection.
     * This tells the modem how many symbols to search for preamble.
     * Using mantissa=7, exponent=1 gives 7^(2*1+1) = 7^3 = 343 symbols,
     * which matches our ~342 symbol sync preamble. */
    lr20xx_radio_lora_configure_timeout_by_mantissa_exponent_symbols(NULL, 7, 1);

    /* Calculate optimal sync preamble length based on SF/BW and timeout.
     * Preamble must be long enough that RX can recover from a false CAD
     * detection on the wrong channel and still catch the sync packet. */
    fhss_cfg.preamble_len_symb = fhss_calc_sync_preamble_len(sf, bw_khz);

    printf("CAD configured: symb=%u, peak=%u, pnr_delta=%u, timeout=%lu ms (SF%u BW%ukHz)\r\n",
           cfg->cad_symb_nb, fhss_cfg.cad_cfg.cad_detect_peak,
           cfg->pnr_delta, fhss_cfg.cad_cfg.cad_timeout_ms, sf, bw_khz);
    printf("Sync preamble: %u symbols (%.0f ms)\r\n",
           fhss_cfg.preamble_len_symb,
           (float)fhss_cfg.preamble_len_symb * (1UL << sf) / bw_khz);
}

void fhss_start_cad(void)
{
    extern lr20xx_system_chip_modes_t LR20xx_chipMode;
    lr20xx_status_t rc;
    lr20xx_system_irq_mask_t irq_status;

    /* Ensure we're in STBY_XOSC for CAD (faster PLL lock) */
    lorahal.standbyXosc();

    /* Clear any pending IRQs before starting CAD */
    lr20xx_system_get_and_clear_irq_status(NULL, &irq_status);
    if (irq_status != 0) {
        printf("fhss_start_cad: cleared pending irq=0x%lx\r\n", (unsigned long)irq_status);
    }

    /* Update our tracking from the SPI response */
    extern lr20xx_stat_t stat;
    LR20xx_chipMode = stat.bits.chip_mode;

    rc = lr20xx_radio_lora_set_cad(NULL);
    if (rc != LR20XX_STATUS_OK) {
        printf("fhss_start_cad: FAILED rc=%d\r\n", rc);
    }
}

static const char* chip_mode_name(uint8_t mode)
{
    switch (mode) {
        case LR20XX_SYSTEM_CHIP_MODE_SLEEP: return "SLEEP";
        case LR20XX_SYSTEM_CHIP_MODE_STBY_RC: return "STBY_RC";
        case LR20XX_SYSTEM_CHIP_MODE_STBY_XOSC: return "STBY_XOSC";
        case LR20XX_SYSTEM_CHIP_MODE_FS: return "FS";
        case LR20XX_SYSTEM_CHIP_MODE_RX: return "RX";
        case LR20XX_SYSTEM_CHIP_MODE_TX: return "TX";
        default: return "?";
    }
}

void fhss_cad_done_handler(bool detected)
{
    uint8_t chip_mode = stat.bits.chip_mode;

    fhss_cfg.stats.cad_done_count++;

    if (detected) {
        fhss_cfg.stats.cad_detected_count++;

        if (fhss_cfg.state == FHSS_STATE_RX_SCAN) {
            /* Found signal during scan - switch to sync state.
             * With CAD_EXIT_MODE_RX, radio is already in RX mode with
             * CAD_EXTENDED_TIMEOUT_MS timeout - no need to call rx(). */
            fhss_cfg.state = FHSS_STATE_RX_SYNC;
            fhss_cfg.preamble_confirm_count = 0;

            /* Check and clear any stale RX FIFO data before receiving sync packet */
            uint16_t fifo_level = 0;
            lr20xx_radio_fifo_get_rx_level(NULL, &fifo_level);
            if (fifo_level > 0) {
                printf("CAD: stale RX FIFO data (%u bytes), clearing\r\n", fifo_level);
                lr20xx_radio_fifo_clear_rx(NULL);
            }

            /* Record start time for timeout tracking */
            extern uint32_t HAL_GetTick(void);
            cad_rx_start_ms = HAL_GetTick();

            /* Radio should be in RX mode (CAD_EXIT_MODE_RX) */
            printf("CAD detected on ch%u, mode=%s\r\n", fhss_cfg.current_channel,
                   chip_mode_name(chip_mode));
        }
    } else {
        /* No detection - continue scanning if in scan mode */
        if (fhss_cfg.state == FHSS_STATE_RX_SCAN) {
            fhss_scan_next_channel();
        }
    }
}

void fhss_preamble_detected_handler(void)
{
    extern uint32_t HAL_GetTick(void);
    uint32_t now = HAL_GetTick();

    /* Preamble detected confirms CAD detection was a real signal, not noise.
     *
     * Two-stage timeout approach:
     * - Initial RX starts with short timeout (CAD_VERIFY_TIMEOUT_MS = 50ms)
     * - On first preamble confirmation, restart RX with extended timeout
     * - This reduces false CAD penalty from 400ms to 50ms
     *
     * IMPORTANT: Use standbyXosc() for fast transition (~100us). Using standby()
     * goes to STBY_RC which requires XOSC warmup (~5ms) and loses preamble lock. */
    if (fhss_cfg.state == FHSS_STATE_RX_SYNC) {
        fhss_cfg.preamble_confirm_count++;

        /* Don't print during preamble detection - printf is blocking (~5ms per line)
         * and would cause sync loss during active reception. Only log count at end. */
#if 0
        /* Log timing for first few preamble detections */
        if (fhss_cfg.preamble_confirm_count <= 3) {
            uint32_t elapsed = now - cad_rx_start_ms;
            printf("Preamble confirmed on ch%u (%.3f MHz) [%u] +%lums\r\n",
                   fhss_cfg.current_channel,
                   (float)fhss_get_channel_freq(fhss_cfg.current_channel) / 1000000.0f,
                   fhss_cfg.preamble_confirm_count,
                   elapsed);
        }
#endif

        /* With extended timeout used directly in CAD handler, no need to
         * restart RX here - that would interrupt ongoing reception.
         * Just track preamble confirmations for debugging. */
    }
}

void fhss_timeout_handler(void)
{
    /* Timeout during RX_SYNC - analyze what happened based on preamble count */
    if (fhss_cfg.state == FHSS_STATE_RX_SYNC) {
        if (fhss_cfg.preamble_confirm_count == 0) {
            /* No preamble detected - pure false CAD (noise) */
            fhss_cfg.stats.cad_false_count++;
            printf("False CAD on ch%u (noise)\r\n", fhss_cfg.current_channel);
        } else if (fhss_cfg.preamble_confirm_count < 3) {
            /* Brief preamble then timeout - likely a data packet (wrong header mode)
             * This happens when RX lands on a data channel after TX hopped */
            printf("Short preamble on ch%u (%u confirms) - data channel?\r\n",
                   fhss_cfg.current_channel, fhss_cfg.preamble_confirm_count);
        } else {
            /* Extended timeout after long preamble - sync packet not received
             * This is unexpected - maybe interference or timing issue */
            printf("Timeout after long preamble on ch%u (%u confirms)\r\n",
                   fhss_cfg.current_channel, fhss_cfg.preamble_confirm_count);
        }

        /* In all cases, resume scanning */
        fhss_cfg.state = FHSS_STATE_RX_SCAN;
        fhss_scan_next_channel();
    }
}

void fhss_start_scan(void)
{
    extern uint32_t HAL_GetTick(void);

    /* Print and clear radio stats from previous session before starting new scan */
    fhss_print_and_reset_rx_stats();

    fhss_cfg.state = FHSS_STATE_RX_SCAN;
    fhss_cfg.stats.channels_scanned = 0;
    fhss_cfg.stats.sync_time_ms = HAL_GetTick();
    fhss_cfg.stats.sweep_start_ms = HAL_GetTick();
    fhss_cfg.stats.sweep_count = 0;

    printf("FHSS scan starting...\r\n");

    /* Configure CAD parameters (puts radio in standby) */
    fhss_configure_cad(&fhss_cfg.cad_cfg);

    /* Configure for sync packet reception: explicit header mode.
     * This is needed because fhss_configure_data_mode() sets implicit header. */
    lorahal.loRaPacketConfig(fhss_cfg.preamble_len_symb, false, true, false, 0);

    /* Set starting channel (radio already in standby from configure_cad) */
    fhss_set_channel(0);

    printf("FHSS scan started from ch0 (%.3f MHz)\r\n",
           (float)fhss_get_channel_freq(0) / 1000000.0f);

    /* Start CAD */
    fhss_start_cad();
}

void fhss_scan_next_channel(void)
{
    extern uint32_t HAL_GetTick(void);

    fhss_cfg.stats.channels_scanned++;

    if (fhss_cfg.stats.channels_scanned >= FHSS_NUM_CHANNELS) {
        /* Completed full scan without detection */
        uint32_t now = HAL_GetTick();
        fhss_cfg.stats.last_sweep_ms = now - fhss_cfg.stats.sweep_start_ms;
        fhss_cfg.stats.sweep_count++;
        fhss_cfg.stats.sweep_start_ms = now;  /* Start timing next sweep */

        printf("sweep %lu: %lu ms, mode=%s\r\n",
               fhss_cfg.stats.sweep_count,
               fhss_cfg.stats.last_sweep_ms,
               chip_mode_name(stat.bits.chip_mode));

        fhss_cfg.stats.channels_scanned = 0;
    }

    /* Sequential sweep through all channels */
    uint8_t next_ch = (fhss_cfg.current_channel + 1) % FHSS_NUM_CHANNELS;

    /* SetCad requires standby mode. Use XOSC for faster transitions. */
    lorahal.standbyXosc();
    fhss_set_channel(next_ch);

    /* Start CAD on new channel */
    fhss_start_cad();
}

bool fhss_is_scanning(void)
{
    /* Return true during RX_SCAN and RX_SYNC states.
     * RX_SYNC is still part of scanning - waiting for preamble confirmation
     * or timeout before resuming scan. */
    return fhss_cfg.state == FHSS_STATE_RX_SCAN ||
           fhss_cfg.state == FHSS_STATE_RX_SYNC;
}

uint16_t fhss_get_lfsr_state(void)
{
    return lfsr_state;
}

void fhss_set_lfsr_state(uint16_t state)
{
    if (state == 0) state = 0xACE1;  /* Avoid zero state */
    lfsr_state = state;
}

void fhss_start_tx_sync(void)
{
    /* Select random channel for sync */
    uint8_t sync_channel = fhss_random_channel();
    fhss_set_channel(sync_channel);

    fhss_cfg.state = FHSS_STATE_TX_PREAMBLE;
    fhss_cfg.tx_sync_count = 1;  /* First sync packet */
    fhss_cfg.tx_sync_channel = sync_channel;  /* Save for ACK verification */
    fhss_cfg.sync_attempts = 0;  /* Reset ACK retry counter */
    fhss_cfg.ack_received = false;

    /* Reset hop counter and resync flag - counts hops since this LFSR state.
     * RX uses this to fast-forward if it receives sync late. */
    tx_hop_count = 0;
    tx_resync_pending = false;

    /* Prepare sync packet with current LFSR state and next channel.
     * IMPORTANT: Save next_channel and LFSR to fhss_cfg so we can:
     * 1. Use same LFSR in repeat sync packets
     * 2. Hop to next_channel after all syncs complete */
    fhss_sync_pkt_t *sync_pkt = (fhss_sync_pkt_t *)lorahal.tx_buf;
    sync_pkt->marker = FHSS_SYNC_MARKER;
    sync_pkt->lfsr_state = lfsr_state;
    fhss_cfg.tx_sync_lfsr = lfsr_state;  /* Save for repeat syncs */
    sync_pkt->next_channel = fhss_random_channel();  /* Advances LFSR */
    fhss_cfg.tx_next_channel = sync_pkt->next_channel;  /* Save for first hop */
    sync_pkt->hop_count = tx_hop_count;  /* 0 at start */
    sync_pkt->sync_channel = sync_channel;  /* For ACK response */

    printf("TX sync on ch%u (%.3f MHz), lfsr=0x%04X, next_ch=%u, hops=%u, preamble=%u\r\n",
           sync_channel,
           (float)fhss_get_channel_freq(sync_channel) / 1000000.0f,
           sync_pkt->lfsr_state,
           sync_pkt->next_channel,
           sync_pkt->hop_count,
           fhss_cfg.preamble_len_symb);

    /* Configure long preamble and send sync packet (explicit header mode) */
    lorahal.loRaPacketConfig(fhss_cfg.preamble_len_symb, false, true, false, FHSS_SYNC_PKT_SIZE);

    /* For ACK mode, configure auto RX after TX completes.
     * This ensures immediate transition to RX without software delays. */
    if (fhss_cfg.ack_mode_enabled) {
        /* Configure packet format for receiving ACK BEFORE starting TX */
        lorahal.loRaPacketConfig(FHSS_ACK_PREAMBLE_LEN, false, true, false, FHSS_ACK_PKT_SIZE);

        /* Configure auto RX after TX with timeout */
        lr20xx_radio_common_auto_tx_rx_configuration_t auto_cfg = {
            .condition = LR20XX_RADIO_COMMON_AUTO_TX_RX_ALWAYS,
            .disable_on_failure = true,
            .delay_in_tick = 32000,  /* ~1ms delay (32MHz clock) */
            .tx_rx_timeout_in_rtc_step = lr20xx_radio_common_convert_time_in_ms_to_rtc_step(FHSS_ACK_TIMEOUT_MS)
        };
        lr20xx_radio_common_configure_auto_tx_rx(NULL, &auto_cfg);

        /* Now configure packet for TX (sync packet) and send */
        lorahal.loRaPacketConfig(fhss_cfg.preamble_len_symb, false, true, false, FHSS_SYNC_PKT_SIZE);
    }

    lorahal.send(FHSS_SYNC_PKT_SIZE);
}

/* Expected payload length for RX (set by app before scanning) */
static uint8_t fhss_rx_payload_len = 0;

void fhss_set_rx_payload_len(uint8_t payload_len)
{
    fhss_rx_payload_len = payload_len;
    printf("FHSS: RX expected payload=%u bytes\r\n", payload_len);
}

static uint8_t fhss_get_rx_payload_len(void)
{
    return fhss_rx_payload_len;
}

bool fhss_rx_sync_packet(const uint8_t *data, uint8_t size)
{
    if (size < FHSS_SYNC_PKT_SIZE) {
        return false;
    }

    const fhss_sync_pkt_t *sync_pkt = (const fhss_sync_pkt_t *)data;

    if (sync_pkt->marker != FHSS_SYNC_MARKER) {
        printf("FHSS: Invalid sync marker 0x%02X on ch%u\r\n", sync_pkt->marker, fhss_cfg.current_channel);
        return false;
    }

    /* Validate sync packet fields - reject obviously corrupted data */
    if (sync_pkt->next_channel >= FHSS_NUM_CHANNELS) {
        printf("FHSS: Invalid next_channel %u (max %u)\r\n",
               sync_pkt->next_channel, FHSS_NUM_CHANNELS - 1);
        return false;
    }

    /* Since resync is disabled, TX always sends initial sync with hop_count=0.
     * Any non-zero hop_count indicates corruption (likely image frequency).
     * This catches the case where packet data is subtly wrong but CRC passes. */
    if (sync_pkt->hop_count != 0) {
        printf("FHSS: Rejecting sync with hop_count=%u (expected 0)\r\n",
               sync_pkt->hop_count);
        return false;
    }

    /* Synchronize LFSR state with transmitter.
     * TX saved lfsr_state BEFORE calling random_channel() to get next_ch.
     * We must call random_channel() too to advance LFSR to match TX. */
    fhss_set_lfsr_state(sync_pkt->lfsr_state);
    uint8_t computed_next_ch = fhss_random_channel();  /* Advances LFSR */

    printf("FHSS sync on ch%u: lfsr=0x%04X, next_ch=%u, hop_count=%u\r\n",
           fhss_cfg.current_channel,
           sync_pkt->lfsr_state,
           sync_pkt->next_channel,
           sync_pkt->hop_count);

    /* Verify LFSR produces same next_ch as TX (sanity check) */
    if (computed_next_ch != sync_pkt->next_channel) {
        printf("FHSS warning: computed ch=%u != sync ch=%u\r\n",
               computed_next_ch, sync_pkt->next_channel);
    }

    /* Determine target channel - start with next_channel from sync packet */
    uint8_t target_ch = sync_pkt->next_channel;

    /* If TX has hopped since sending sync, fast-forward the LFSR to catch up.
     * Each hop advances LFSR once (ignoring loop_count since RX does same).
     * We need to advance hop_count times to reach TX's current position. */
    if (sync_pkt->hop_count > 0) {
        printf("FHSS: fast-forward %u hops: ch%u", sync_pkt->hop_count, target_ch);
        for (uint8_t i = 0; i < sync_pkt->hop_count; i++) {
            /* Advance LFSR same way as fhss_hop() does.
             * Note: fhss_hop() loops until new_ch != current_ch.
             * We simulate that here by calling random until different. */
            uint8_t prev_ch = target_ch;
            do {
                target_ch = fhss_random_channel();
            } while (target_ch == prev_ch);
        }
        printf(" -> ch%u (%.3f MHz)\r\n", target_ch,
               (float)fhss_get_channel_freq(target_ch) / 1000000.0f);
    }

    /* Save channel info for ACK packet.
     * In ACK mode, use TX's sync_channel from packet (where TX is listening).
     * Also save where RX detected the signal to detect image frequency issues. */
    uint8_t rx_detected_ch = fhss_cfg.current_channel;
    uint8_t tx_sync_ch = sync_pkt->sync_channel;

    /* Use TX's sync channel for ACK (where TX is listening for ACK) */
    fhss_cfg.rx_sync_channel = tx_sync_ch;

    /* Check for image frequency reception */
    if (rx_detected_ch != tx_sync_ch) {
        printf("FHSS: image detected! RX on ch%u, TX on ch%u\r\n", rx_detected_ch, tx_sync_ch);
    }

    /* Set channel to where TX should be now (the data channel) */
    fhss_set_channel(target_ch);

    /* Update stats - sync actually received */
    fhss_cfg.stats.sync_found_count++;

    /* Record when sync was achieved - used for initial wait period.
     * RX should stay on this channel while TX finishes sync repeats. */
    extern uint32_t HAL_GetTick(void);
    fhss_cfg.rx_sync_achieved_ms = HAL_GetTick();

    /* Reset streaming RX indices IMMEDIATELY to prevent streaming_rx_decode()
     * from running with stale indices while we process sync/send ACK.
     * This prevents premature stream_start with idx=0. */
    extern volatile uint16_t rx_fifo_read_idx;
    extern volatile uint16_t rx_decode_idx;
    rx_fifo_read_idx = 0;
    rx_decode_idx = FHSS_DATA_HDR_SIZE;  /* Skip FHSS header when decoding */

    if (fhss_cfg.ack_mode_enabled) {
        /* ACK mode: transition to send ACK state.
         * Caller should call fhss_send_sync_ack() to send ACK.
         * Data mode configuration happens after ACK is sent. */
        fhss_cfg.state = FHSS_STATE_RX_SEND_ACK;
        printf("FHSS: sync received, will send ACK on ch%u\r\n", fhss_cfg.rx_sync_channel);
    } else {
        /* No ACK mode: configure data mode and start receiving */
        uint8_t rx_payload = fhss_get_rx_payload_len();
        if (rx_payload > 0) {
            fhss_configure_data_mode(rx_payload);
        } else {
            printf("FHSS: warning - RX payload length not set\r\n");
        }
        /* Transition to data receive state */
        fhss_cfg.state = FHSS_STATE_RX_DATA;
    }

    return true;
}

/* Send ACK after receiving sync (called by RX when ack_mode_enabled) */
void fhss_send_sync_ack(void)
{
    if (fhss_cfg.state != FHSS_STATE_RX_SEND_ACK) {
        printf("FHSS: fhss_send_sync_ack called in wrong state %d\r\n", fhss_cfg.state);
        return;
    }

    /* Build ACK packet */
    fhss_ack_pkt_t *ack_pkt = (fhss_ack_pkt_t *)lorahal.tx_buf;
    ack_pkt->marker = FHSS_ACK_MARKER;
    ack_pkt->rx_channel = fhss_cfg.rx_sync_channel;

    /* ACK is sent on the sync channel (where RX received sync).
     * TX is still on sync channel waiting for ACK.
     * Need to switch back to sync channel temporarily. */
    uint8_t data_ch = fhss_cfg.current_channel;  /* Save data channel */
    fhss_set_channel(fhss_cfg.rx_sync_channel);

    printf("FHSS: sending ACK on ch%u (%.3f MHz), data will be on ch%u\r\n",
           fhss_cfg.rx_sync_channel,
           (float)fhss_get_channel_freq(fhss_cfg.rx_sync_channel) / 1000000.0f,
           data_ch);

    /* Configure short preamble, explicit header for ACK */
    lorahal.loRaPacketConfig(FHSS_ACK_PREAMBLE_LEN, false, true, false, FHSS_ACK_PKT_SIZE);

    /* Store data channel to restore after ACK TX done */
    fhss_cfg.tx_next_channel = data_ch;

    /* Note: TX uses auto-RX after sync TX, so it's already in RX mode
     * by the time we reach here. A small delay helps ensure TX has
     * completed the mode switch (auto-RX has ~1ms configured delay). */
    extern uint32_t HAL_GetTick(void);
    uint32_t delay_end = HAL_GetTick() + 5;  /* 5ms delay */
    while (HAL_GetTick() < delay_end) {
        /* busy wait */
    }

    lorahal.send(FHSS_ACK_PKT_SIZE);
    /* TxDone will transition to RX_DATA and switch to data channel */
}

/* Process received ACK packet (called by TX when in WAIT_ACK state) */
bool fhss_rx_ack_packet(const uint8_t *data, uint8_t size)
{
    if (size < FHSS_ACK_PKT_SIZE) {
        return false;
    }

    const fhss_ack_pkt_t *ack_pkt = (const fhss_ack_pkt_t *)data;

    if (ack_pkt->marker != FHSS_ACK_MARKER) {
        printf("FHSS: Invalid ACK marker 0x%02X\r\n", ack_pkt->marker);
        return false;
    }

    /* Calculate time from TxDone to ACK RxDone */
    extern uint32_t HAL_GetTick(void);
    uint32_t ack_delay_ms = HAL_GetTick() - fhss_cfg.ack_wait_start_ms;

    /* Check if RX received sync on the channel TX sent it on.
     * If rx_channel != tx_sync_channel, RX might have received via image frequency. */
    if (ack_pkt->rx_channel != fhss_cfg.tx_sync_channel) {
        printf("FHSS: ACK channel mismatch! TX sent on ch%u, RX heard on ch%u (image?) delay=%lu ms\r\n",
               fhss_cfg.tx_sync_channel, ack_pkt->rx_channel, ack_delay_ms);
        /* For now, accept anyway but log the mismatch for debugging */
    } else {
        printf("FHSS: ACK received on ch%u, delay=%lu ms after TxDone\r\n",
               ack_pkt->rx_channel, ack_delay_ms);
    }

    fhss_cfg.ack_received = true;
    return true;
}

/* Handle ACK timeout (TX didn't receive ACK, send another sync) */
void fhss_ack_timeout_handler(void)
{
    extern uint32_t HAL_GetTick(void);

    if (fhss_cfg.state != FHSS_STATE_TX_WAIT_ACK) {
        return;  /* Not waiting for ACK */
    }

    fhss_cfg.sync_attempts++;

    if (fhss_cfg.sync_attempts >= FHSS_SYNC_MAX_ATTEMPTS) {
        printf("FHSS: ACK timeout after %u attempts, giving up\r\n", fhss_cfg.sync_attempts);
        fhss_cfg.state = FHSS_STATE_IDLE;
        fhss_cfg.sync_attempts = 0;
        return;
    }

    printf("FHSS: ACK timeout, retry %u/%u\r\n", fhss_cfg.sync_attempts, FHSS_SYNC_MAX_ATTEMPTS);

    /* Send another sync on a new channel but with SAME LFSR state.
     * RX may have already synced to our first sync packet. If we change
     * the LFSR, TX and RX will hop to different channels after the first data channel.
     * Keep the same LFSR so we stay synchronized.
     *
     * IMPORTANT: Don't use fhss_random_channel() here - it advances the LFSR!
     * Use a deterministic offset instead, same as fhss_send_sync_repeat(). */
    uint8_t sync_channel = (fhss_cfg.tx_next_channel + fhss_cfg.sync_attempts * 17) % FHSS_NUM_CHANNELS;
    fhss_set_channel(sync_channel);

    fhss_cfg.state = FHSS_STATE_TX_PREAMBLE;
    fhss_cfg.tx_sync_count = 1;
    fhss_cfg.tx_sync_channel = sync_channel;
    /* DON'T reset sync_attempts - already incremented above */
    /* DON'T reset ack_received - keep tracking */

    /* Build sync packet with SAME LFSR state as original sync.
     * fhss_cfg.tx_sync_lfsr was saved in first fhss_start_tx_sync() call. */
    fhss_sync_pkt_t *sync_pkt = (fhss_sync_pkt_t *)lorahal.tx_buf;
    sync_pkt->marker = FHSS_SYNC_MARKER;
    sync_pkt->lfsr_state = fhss_cfg.tx_sync_lfsr;  /* Use SAVED lfsr, not current */
    sync_pkt->next_channel = fhss_cfg.tx_next_channel;  /* Use SAVED next_ch */
    sync_pkt->hop_count = 0;  /* RX will use this to know we haven't hopped */
    sync_pkt->sync_channel = sync_channel;

    printf("TX sync on ch%u (%.3f MHz), lfsr=0x%04X, next_ch=%u, hops=%u, preamble=%u\r\n",
           sync_channel, (float)fhss_get_channel_freq(sync_channel) / 1000000.0f,
           sync_pkt->lfsr_state, sync_pkt->next_channel, sync_pkt->hop_count,
           fhss_cfg.preamble_len_symb);

    /* Configure and send sync packet with long preamble */
    lorahal.loRaPacketConfig(fhss_cfg.preamble_len_symb, false, true, false, FHSS_SYNC_PKT_SIZE);

    if (fhss_cfg.ack_mode_enabled) {
        /* Configure packet format for receiving ACK BEFORE starting TX */
        lorahal.loRaPacketConfig(FHSS_ACK_PREAMBLE_LEN, false, true, false, FHSS_ACK_PKT_SIZE);

        /* Configure auto RX after TX with timeout */
        lr20xx_radio_common_auto_tx_rx_configuration_t auto_cfg = {
            .condition = LR20XX_RADIO_COMMON_AUTO_TX_RX_ALWAYS,
            .disable_on_failure = true,
            .delay_in_tick = 32000,  /* ~1ms delay (32MHz clock) */
            .tx_rx_timeout_in_rtc_step = lr20xx_radio_common_convert_time_in_ms_to_rtc_step(FHSS_ACK_TIMEOUT_MS)
        };
        lr20xx_radio_common_configure_auto_tx_rx(NULL, &auto_cfg);

        /* Now configure packet for TX (sync packet) and send */
        lorahal.loRaPacketConfig(fhss_cfg.preamble_len_symb, false, true, false, FHSS_SYNC_PKT_SIZE);
    }

    lorahal.send(FHSS_SYNC_PKT_SIZE);
}

/* Enable/disable ACK-based sync mode */
void fhss_set_ack_mode(bool enabled)
{
    fhss_cfg.ack_mode_enabled = enabled;
    printf("FHSS: ACK mode %s\r\n", enabled ? "enabled" : "disabled");
}

void fhss_configure_data_mode(uint8_t payload_len)
{
    extern uint32_t HAL_GetTick(void);
    extern volatile uint32_t terminate_spkr_at_tick;

    /* Store data packet length (codec2 payload + FHSS header) */
    fhss_cfg.data_pkt_len = payload_len + FHSS_DATA_HDR_SIZE;

    /* Reset sequence numbers and timeout counter */
    fhss_cfg.tx_seq_num = 0;
    fhss_cfg.rx_seq_num = 0;
    fhss_cfg.rx_timeout_consec = 0;

    /* Don't reset dwell timer for TX - it was already set by fhss_tx_done_handler().
     * For RX, reset to 0 so dwell timer starts when first data packet is received.
     * This prevents RX from hopping before TX finishes sync repeats. */
    if (fhss_cfg.state != FHSS_STATE_TX_DATA) {
        extern volatile uint8_t terminate_spkr_rx;
        fhss_cfg.dwell_start_ms = 0;  /* RX: timer starts on first packet */
        /* Set initial timeout to cover TX sync repeats.
         * RX must wait for TX to finish sync repeats and start data.
         * The first data packet reception will reset this to normal timeout.
         * Clear terminate_spkr_rx AFTER setting timeout to prevent races. */
        terminate_spkr_at_tick = HAL_GetTick() + FHSS_RX_INITIAL_WAIT_MS;
        terminate_spkr_rx = 0;       /* Clear any stale flag */
    }
    fhss_cfg.pkts_on_channel = 0;

    /* Must be in standby before changing packet parameters.
     * Use XOSC standby for faster transition back to RX. */
    lorahal.standbyXosc();

    /* Clear ALL system IRQs (including any stale TIMEOUT IRQ).
     * This is critical when transitioning from sync mode to data mode,
     * as the sync preamble detection may have armed a timeout. */
    lr20xx_system_clear_irq_status(NULL, LR20XX_SYSTEM_IRQ_ALL_MASK);

    /* Clear the RX FIFO and any pending FIFO IRQ flags.
     * This prevents stale IRQs from firing when we start RX. */
    lr20xx_radio_fifo_clear_rx(NULL);
    {
        lr20xx_radio_fifo_flag_t rx_flags, tx_flags;
        lr20xx_radio_fifo_get_and_clear_irq_flags(NULL, &rx_flags, &tx_flags);
    }

    /* Reset the streaming RX indices.
     * rx_fifo_read_idx: how many bytes read from radio FIFO
     * rx_decode_idx: where codec2 decode starts (after FHSS header) */
    extern volatile uint16_t rx_fifo_read_idx;
    extern volatile uint16_t rx_decode_idx;
    rx_fifo_read_idx = 0;
    rx_decode_idx = FHSS_DATA_HDR_SIZE;  /* Skip FHSS header when decoding */

    /* Clear first part of buffer to prevent stale data from being decoded
     * if there's any race condition with streaming_rx_decode() */
    extern uint8_t LR20xx_rx_buf[];
    memset(LR20xx_rx_buf, 0, fhss_cfg.data_pkt_len);

    /* Reset the codec2 frame sequence tracker for new FHSS session.
     * This ensures we start fresh when synchronizing with TX. */
    extern uint16_t expected_rx_seq;
    expected_rx_seq = 0;

    /* Also reset any test mode frame counters for fresh session */
    extern uint32_t rx_frame_count;
    rx_frame_count = 0;

    /* Configure radio for implicit header mode with fixed packet length.
     * IMPORTANT: payloadLen must be set for implicit header RX to work! */
    lorahal.loRaPacketConfig(FHSS_DATA_PREAMBLE_LEN,
                              true,   /* fixLen = implicit header */
                              true,   /* crcOn */
                              false,  /* invIQ */
                              fhss_cfg.data_pkt_len);  /* payload length for implicit RX */

    /* Disable LoRa synch timeout for data mode.
     * Data mode uses continuous RX with packet-level timeouts managed by FHSS.
     * Setting to 0 disables the synch timeout mechanism. */
    lr20xx_radio_lora_configure_timeout_by_number_of_symbols(NULL, 0);

    printf("FHSS data mode: pkt_len=%u (payload=%u + hdr=%u), implicit header\r\n",
           fhss_cfg.data_pkt_len, payload_len, FHSS_DATA_HDR_SIZE);
    printf("FHSS: indices reset fifo=%u dec=%u exp_seq=%u\r\n",
           rx_fifo_read_idx, rx_decode_idx, expected_rx_seq);
}

void fhss_check_hop(void)
{
    extern uint32_t HAL_GetTick(void);
    uint32_t now = HAL_GetTick();
    static uint32_t call_count = 0;
    call_count++;

    /* Debug: print first few calls and then periodically */
    if (call_count <= 3 || (call_count % 10 == 0)) {
        printf("check_hop#%lu: state=%d dwell_ms=%lu\r\n",
               call_count, fhss_cfg.state, fhss_cfg.dwell_start_ms);
    }

    /* Only TX hops proactively based on dwell timer.
     * RX follows TX by detecting channel changes in received packet headers.
     * This avoids RX hopping before TX (due to dwell timer offset). */
    if (fhss_cfg.state != FHSS_STATE_TX_DATA) {
        return;
    }

    /* FCC Part 15.247: Max 400ms of TRANSMIT time per channel.
     * We track this via packet count since each packet has known TOA.
     * proactive_hop_count = floor(400ms / packet_toa)
     *
     * Hop when we've sent proactive_hop_count packets on this channel.
     * The actual hop happens BEFORE sending the next packet in fhss_send_data().
     */
    if (fhss_cfg.pkts_on_channel >= fhss_cfg.proactive_hop_count) {
        uint8_t old_ch = fhss_cfg.current_channel;

        /* Must go to standby before changing frequency */
        lorahal.standby();

        /* Hop to next channel using synchronized LFSR */
        fhss_hop();

        /* Reset packet counter for new channel */
        fhss_cfg.pkts_on_channel = 0;

        printf("FHSS hop: ch%u->ch%u after %u pkts\r\n",
               old_ch, fhss_cfg.current_channel, fhss_cfg.proactive_hop_count);
    }
}

static uint32_t fhss_send_data_cnt = 0;

int fhss_send_data(const uint8_t *data, uint8_t len)
{
    fhss_data_hdr_t *hdr;
    extern uint32_t HAL_GetTick(void);
    uint32_t now = HAL_GetTick();
    fhss_send_data_cnt++;

    /* Debug: track calls to fhss_send_data */
    if (fhss_send_data_cnt <= 3)
        printf("send_data#%lu len=%u\r\n", fhss_send_data_cnt, len);

    if (len + FHSS_DATA_HDR_SIZE != fhss_cfg.data_pkt_len) {
        printf("FHSS: payload len %u != configured %u\r\n",
               len, fhss_cfg.data_pkt_len - FHSS_DATA_HDR_SIZE);
        return -1;
    }

    /* Re-sync during data mode is disabled for now.
     * The data packet channel field provides sufficient tracking for RX to
     * follow TX hops. If RX loses sync (3 timeouts), it returns to CAD scan
     * and catches the next initial sync.
     *
     * Re-sync was causing issues: RX receiving corrupted re-sync packets
     * (possibly due to image reception or weak signal interference). */
#if 0
    /* Send periodic re-sync packet if pending.
     * This gives RX a chance to catch up if it missed initial sync. */
    if (tx_resync_pending) {
        fhss_send_resync_packet();
    }
#endif
    tx_resync_pending = false;  /* Clear flag even though we're not sending */

    /* Check if there's a deferred packet from previous hop - shouldn't happen
     * in normal operation since we send deferred packet immediately after hop,
     * but handle it just in case. */
    if (deferred_pkt_len > 0) {
        printf("FHSS: warning - deferred packet still pending\r\n");
        memcpy(lorahal.tx_buf, deferred_pkt_buf, deferred_pkt_len);
        lorahal.send(deferred_pkt_len);
        deferred_pkt_len = 0;
        fhss_cfg.pkts_on_channel++;
        /* Fall through to handle current packet normally */
    }

    /* FCC Part 15.247: Max 400ms of TRANSMIT time per channel.
     * We track this via packet count (pkts_on_channel) since each packet has known TOA.
     * fhss_check_hop() will hop when we've sent proactive_hop_count packets.
     * No wall-clock timing needed - only actual TX time counts toward the 400ms limit. */
    fhss_check_hop();

    /* Shift payload to make room for header.
     * IMPORTANT: Copy backwards to handle overlapping buffers correctly
     * (when data == lorahal.tx_buf, which is the normal case). */
    for (int i = len - 1; i >= 0; i--) {
        lorahal.tx_buf[FHSS_DATA_HDR_SIZE + i] = data[i];
    }

    /* Build header at start of buffer */
    hdr = (fhss_data_hdr_t *)lorahal.tx_buf;
    hdr->marker = FHSS_DATA_MARKER;
    hdr->seq_num = fhss_cfg.tx_seq_num++;
    hdr->channel = fhss_cfg.current_channel;

    fhss_cfg.state = FHSS_STATE_TX_DATA;
    fhss_cfg.pkts_on_channel++;

    lorahal.send(fhss_cfg.data_pkt_len);
    return 0;
}

int fhss_rx_data_packet(const uint8_t *data, uint8_t size)
{
    const fhss_data_hdr_t *hdr;
    uint8_t expected_seq;
    int8_t seq_diff;

    if (size < FHSS_DATA_HDR_SIZE) {
        /* Invalid size - clear rx_fifo_read_idx to prevent garbage decode */
        extern volatile uint16_t rx_fifo_read_idx;
        rx_fifo_read_idx = 0;
        return -1;
    }

    hdr = (const fhss_data_hdr_t *)data;

    /* Check for re-sync packet (marker 0xAA) during data mode.
     * TX sends re-sync every 3 hops using the same implicit header mode as data,
     * so RX can receive it without mode switching. Process sync and continue. */
    if (hdr->marker == FHSS_SYNC_MARKER) {
        const fhss_sync_pkt_t *sync_pkt = (const fhss_sync_pkt_t *)data;
        extern volatile uint16_t rx_fifo_read_idx;

        /* Validate sync packet fields */
        if (sync_pkt->next_channel >= FHSS_NUM_CHANNELS || sync_pkt->hop_count > 100) {
            printf("FHSS: invalid re-sync (next_ch=%u, hops=%u)\r\n",
                   sync_pkt->next_channel, sync_pkt->hop_count);
            rx_fifo_read_idx = 0;
            return -1;
        }

        printf("FHSS: re-sync received, lfsr=0x%04X, next_ch=%u, hops=%u\r\n",
               sync_pkt->lfsr_state, sync_pkt->next_channel, sync_pkt->hop_count);

        /* Re-synchronize LFSR state */
        fhss_set_lfsr_state(sync_pkt->lfsr_state);
        (void)fhss_random_channel();  /* Advance LFSR past next_channel */

        /* Fast-forward LFSR by hop_count */
        uint8_t target_ch = sync_pkt->next_channel;
        for (uint8_t i = 0; i < sync_pkt->hop_count; i++) {
            uint8_t prev_ch = target_ch;
            do {
                target_ch = fhss_random_channel();
            } while (target_ch == prev_ch);
        }

        /* Update to current channel */
        fhss_set_channel(target_ch);
        fhss_cfg.pkts_on_channel = 0;
        fhss_cfg.dwell_start_ms = 0;

        printf("FHSS: re-synced to ch%u\r\n", target_ch);

        /* Clear rx_fifo_read_idx - sync packet has no audio data */
        rx_fifo_read_idx = 0;
        return -1;  /* Not a data packet */
    }

    /* Check for end-of-transmission packet */
    if (hdr->marker == FHSS_END_MARKER) {
        extern volatile uint16_t rx_fifo_read_idx;
        extern void end_rx_tone(void);

        printf("FHSS: end packet received, stopping audio\r\n");

        /* Stop audio immediately */
        end_rx_tone();

        /* Return to CAD scan */
        fhss_cfg.state = FHSS_STATE_IDLE;
        fhss_start_scan();

        /* Clear rx_fifo_read_idx - end packet has no audio data */
        rx_fifo_read_idx = 0;
        return -1;  /* Not a data packet */
    }

    if (hdr->marker != FHSS_DATA_MARKER) {
        /* Not a data packet and not a sync packet - garbage.
         * Clear rx_fifo_read_idx to prevent main loop from decoding garbage. */
        extern volatile uint16_t rx_fifo_read_idx;
        extern uint32_t HAL_GetTick(void);
        printf("FHSS: bad marker 0x%02X (expected 0x%02X or 0x%02X), size=%u, on ch%u, pkt_ch=%u seq=%u, time=%lu\r\n",
               hdr->marker, FHSS_DATA_MARKER, FHSS_SYNC_MARKER, size, fhss_cfg.current_channel,
               hdr->channel, hdr->seq_num, HAL_GetTick());
        printf("FHSS: first 8 bytes: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
               data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
        rx_fifo_read_idx = 0;

        /* DON'T count bad marker packets for proactive hop timing.
         * These could be:
         * - Sync repeats (marker 0xAA) misinterpreted in data mode
         * - Noise/interference generating false RxDone
         * - Corrupted packets from other sources
         * Since we can't verify TX actually sent this, don't advance LFSR.
         * The timeout handler will hop if TX has moved on. */
        return -1;
    }

    /* Verify channel - if different, TX has hopped. Advance LFSR to catch up. */
    if (hdr->channel != fhss_cfg.current_channel) {
        extern volatile uint32_t terminate_spkr_at_tick;
        extern unsigned inter_pkt_timeout;
        extern uint32_t HAL_GetTick(void);
        uint8_t next_ch;
        int tries = 0;

        /* Advance LFSR until we reach the channel TX is on */
        do {
            next_ch = fhss_random_channel();
            tries++;
        } while (next_ch != hdr->channel && tries < 5);

        if (next_ch == hdr->channel) {
            printf("FHSS: hop to ch%u (advanced LFSR %d step%s)\r\n",
                   hdr->channel, tries, tries > 1 ? "s" : "");
        } else {
            /* LFSR desync - likely received sync via image on wrong channel.
             * If this is the first data packet (rx_seq_num == 0), restart scan. */
            printf("FHSS: channel mismatch ch%u->ch%u (LFSR desync?)\r\n",
                   fhss_cfg.current_channel, hdr->channel);
            if (fhss_cfg.rx_seq_num == 0) {
                printf("FHSS: LFSR desync on first packet - restarting scan\r\n");
                fhss_start_scan();
                return -1;
            }
        }

        fhss_set_channel(hdr->channel);
        fhss_cfg.dwell_start_ms = 0;  /* Reset dwell timer - will start on next packet */
        fhss_cfg.pkts_on_channel = 0;

        /* Extend terminate_spkr timeout since we just hopped */
        if (terminate_spkr_at_tick != 0) {
            terminate_spkr_at_tick = HAL_GetTick() + inter_pkt_timeout + 100;
        }
    }

    /* Check sequence number for lost packets */
    expected_seq = fhss_cfg.rx_seq_num;
    seq_diff = (int8_t)(hdr->seq_num - expected_seq);

    if (seq_diff != 0) {
        if (seq_diff > 0 && seq_diff < 128) {
            /* Missed packets */
            printf("FHSS: missed %d packets (seq %u, expected %u)\r\n",
                   seq_diff, hdr->seq_num, expected_seq);
        } else if (seq_diff < 0 && seq_diff > -128) {
            /* Duplicate/old packet - likely stale data or image reception.
             * If this is the first packet after sync (expected_seq == 0),
             * this indicates we synced via image and are receiving garbage.
             * Go back to scanning. */
            printf("FHSS: duplicate packet (seq %u, expected %u)\r\n",
                   hdr->seq_num, expected_seq);
            if (expected_seq == 0) {
                printf("FHSS: bad first packet after sync - restarting scan\r\n");
                fhss_start_scan();
            }
            return -1;
        }
    }

    /* Update expected sequence number */
    fhss_cfg.rx_seq_num = hdr->seq_num + 1;
    fhss_cfg.pkts_on_channel++;

    /* Start dwell timer on first valid packet (RX syncs timing to TX) */
    if (fhss_cfg.dwell_start_ms == 0) {
        extern uint32_t HAL_GetTick(void);
        fhss_cfg.dwell_start_ms = HAL_GetTick();
    }

    /* Proactive hop: check if TX is about to hop based on packet count.
     * TX defers packets at 85% of FHSS_MAX_DWELL_MS (340ms).
     * proactive_hop_count is calculated from packet TOA at init time.
     * RX should hop after receiving that many packets to arrive at next
     * channel before TX sends first packet there. */
    if (fhss_cfg.pkts_on_channel >= fhss_cfg.proactive_hop_count) {
        /* TX is about to hop - proactively hop to next channel.
         * Radio can change frequency while in RX mode. */
        fhss_hop();
        fhss_cfg.dwell_start_ms = 0;  /* Reset - will be set on next packet */
        fhss_cfg.pkts_on_channel = 0;
    }

    /* Reset consecutive timeout counter on successful reception */
    fhss_cfg.rx_timeout_consec = 0;

    /* Reset the timeout timer - we just received a valid packet.
     * This is critical: terminate_spkr_at_tick was set to sync_time + 2000ms
     * in fhss_configure_data_mode(). Without resetting it here, that initial
     * timeout would fire even after receiving data, causing premature hop.
     * Set to 80% of dwell time (same as main loop does after packet complete). */
    extern volatile uint32_t terminate_spkr_at_tick;
    extern volatile uint8_t terminate_spkr_rx;
    extern uint32_t HAL_GetTick(void);
    uint32_t now = HAL_GetTick();
    /* NOTE: Debug print moved to fhss_rx_data_print() to minimize RX restart delay.
     * Store values for deferred printing. */
    fhss_cfg.last_pkt_info.pkts_on_ch = fhss_cfg.pkts_on_channel;
    fhss_cfg.last_pkt_info.channel = hdr->channel;
    fhss_cfg.last_pkt_info.seq_num = hdr->seq_num;
    fhss_cfg.last_pkt_info.now = now;
    fhss_cfg.last_pkt_info.old_timeout = terminate_spkr_at_tick;
    fhss_cfg.last_pkt_info.new_timeout = now + (FHSS_MAX_DWELL_MS * 8 / 10);
    fhss_cfg.last_pkt_info.valid = 1;
    terminate_spkr_at_tick = fhss_cfg.last_pkt_info.new_timeout;
    terminate_spkr_rx = 0;

    /* Return payload length (excluding header) */
    return size - FHSS_DATA_HDR_SIZE;
}

void fhss_rx_data(void)
{
    /* Check if we need to hop */
    fhss_check_hop();

    /* DON'T set rx_decode_idx here - the main loop handles it.
     * Setting it in IRQ context causes a race condition:
     * - streaming_rx_decode() at line 1453 may have already decoded frames
     * - Resetting rx_decode_idx = 3 here would cause re-decoding at line 1492
     *
     * rx_decode_idx is set by:
     * - fhss_configure_data_mode(): for FIRST data packet after sync
     * - lora_xcvr.c line 1522: for subsequent packets after processing */

    /* Always restart RX after receiving a packet.
     * After RX_DONE, the radio is in standby, so we must restart RX.
     * Clear any pending IRQs first to avoid stale timeout triggers. */
    extern lr20xx_system_chip_modes_t LR20xx_chipMode;
    int prev_mode = LR20xx_chipMode;

    extern uint32_t HAL_GetTick(void);
    uint32_t start_ms = HAL_GetTick();

    /* Clear pending IRQs before restarting RX */
    lr20xx_system_irq_mask_t pending_irqs;
    lr20xx_system_get_and_clear_irq_status(NULL, &pending_irqs);

    /* CRITICAL: Start RX first, print later.
     * Any printf before rx(0) delays RX restart and can cause missed preambles. */
    lorahal.standbyXosc();
    lr20xx_radio_fifo_clear_rx(NULL);
    lorahal.rx(0);

    uint32_t elapsed = HAL_GetTick() - start_ms;

    /* Now safe to print - RX is running.
     * LR20xx_chipMode is updated by lorahal.rx() */
    if (pending_irqs || elapsed > 5 || LR20xx_chipMode != LR20XX_SYSTEM_CHIP_MODE_RX)
        printf("FHSS: rx_restart %lums, irqs=0x%08X, mode=%d (prev=%d)\r\n",
               elapsed, (unsigned)pending_irqs, LR20xx_chipMode, prev_mode);
}

void fhss_rx_data_print(void)
{
    /* Print deferred packet info after RX has been restarted */
    if (fhss_cfg.last_pkt_info.valid) {
        printf("FHSS: pkt %u on ch%u seq=%u, now=%lu timeout=%lu->%lu\r\n",
               fhss_cfg.last_pkt_info.pkts_on_ch,
               fhss_cfg.last_pkt_info.channel,
               fhss_cfg.last_pkt_info.seq_num,
               fhss_cfg.last_pkt_info.now,
               fhss_cfg.last_pkt_info.old_timeout,
               fhss_cfg.last_pkt_info.new_timeout);
        fhss_cfg.last_pkt_info.valid = 0;
    }
}

void fhss_rx_data_timeout_handler(void)
{
    extern uint32_t HAL_GetTick(void);
    uint32_t now = HAL_GetTick();

    /* Only handle timeouts in RX_DATA state */
    if (fhss_cfg.state != FHSS_STATE_RX_DATA) {
        return;
    }

    fhss_cfg.stats.rx_timeout_count++;
    fhss_cfg.rx_timeout_consec++;

    /* Check if we're still in the initial wait period after sync.
     * During this time, TX is still sending sync repeats on random channels,
     * then will hop to the promised next_ch and start data.
     * RX should NOT hop during this period - stay on next_ch and wait.
     *
     * HOWEVER: if we've already received ANY data packets in this session
     * (rx_seq_num > 0), then TX has started data mode. End the wait.
     * Note: pkts_on_channel resets on hop, but rx_seq_num persists. */
    uint32_t since_sync = now - fhss_cfg.rx_sync_achieved_ms;
    if (since_sync < FHSS_RX_INITIAL_WAIT_MS && fhss_cfg.rx_seq_num == 0) {
        /* Still in initial wait and no data received - don't hop, just restart RX */
        printf("FHSS: RX timeout (wait %lu/%u ms), staying on ch%u\r\n",
               since_sync, FHSS_RX_INITIAL_WAIT_MS, fhss_cfg.current_channel);
        fhss_cfg.rx_timeout_consec = 0;  /* Reset since we're deliberately waiting */
        /* Must put radio in standby before restarting RX.
         * Use XOSC mode for faster RX restart (~100us vs 1-3ms). */
        lorahal.standbyXosc();
        lr20xx_radio_fifo_clear_rx(NULL);
        lorahal.rx(0);
        return;
    }

    if (fhss_cfg.rx_timeout_consec < FHSS_RX_TIMEOUT_MAX) {
        /* Check if we should stay on this channel or hop.
         * TX sends proactive_hop_count packets per channel before hopping.
         * If we haven't received that many, TX is probably still on this channel.
         * Only hop if:
         * 1. We've received proactive_hop_count packets (TX will also hop), OR
         * 2. We've had 2+ timeouts on same channel (missed packets, try to catch up)
         *
         * Use pkts_on_channel to estimate per-channel timeouts:
         * If pkts_on_channel > 0, we received some packets, so this is first timeout.
         * If pkts_on_channel == 0, we haven't received any on this channel yet. */
        uint8_t expected_pkts = fhss_cfg.proactive_hop_count;
        bool should_hop = false;

        if (fhss_cfg.pkts_on_channel >= expected_pkts) {
            /* Received enough packets - TX should also be hopping */
            should_hop = true;
        } else if (fhss_cfg.pkts_on_channel == 0) {
            /* No packets received on this channel - maybe we're out of sync.
             * Hop to try catching up. */
            should_hop = true;
        } else if (fhss_cfg.rx_timeout_consec >= 2) {
            /* Received some packets but 2+ timeouts now - TX has likely hopped.
             * Don't wait forever, hop to try catching up. */
            should_hop = true;
        }
        /* else: received some but not all expected packets, first timeout - stay and wait */

        if (should_hop) {
            /* Hop to next channel */
            uint8_t old_ch = fhss_cfg.current_channel;

            /* Ensure standby before changing frequency.
             * Use XOSC mode for faster RX restart (~100us vs 1-3ms). */
            lorahal.standbyXosc();
            lr20xx_radio_fifo_clear_rx(NULL);

            fhss_hop();
            fhss_cfg.pkts_on_channel = 0;
            printf("FHSS: RX timeout %u/%u, hop ch%u->ch%u\r\n",
                   fhss_cfg.rx_timeout_consec, FHSS_RX_TIMEOUT_MAX,
                   old_ch, fhss_cfg.current_channel);

            /* Restart RX on new channel */
            lorahal.rx(0);
        } else {
            /* Stay on current channel - TX probably still here */
            printf("FHSS: RX timeout on ch%u (pkts=%u/%u), staying\r\n",
                   fhss_cfg.current_channel, fhss_cfg.pkts_on_channel, expected_pkts);
            lorahal.standbyXosc();
            lr20xx_radio_fifo_clear_rx(NULL);
            lorahal.rx(0);
        }
    } else {
        /* Too many consecutive timeouts - TX probably stopped */
        printf("FHSS: %u consecutive timeouts, returning to CAD scan\r\n",
               fhss_cfg.rx_timeout_consec);
        fhss_cfg.stats.resync_count++;
        fhss_cfg.rx_timeout_consec = 0;

        /* Stop audio playback - TX has ended */
        extern void end_rx_tone(void);
        end_rx_tone();
        printf("FHSS: end_rx_tone called\n");

        /* Return to CAD scan to find next sync preamble */
        fhss_start_scan();
    }
}

void fhss_rx_error_handler(bool crc_error, bool len_error, bool hdr_error, float rssi, float snr)
{
    /* Track stats */
    if (crc_error)
        fhss_cfg.stats.cad_false_count++;  /* Reuse false_count for CRC errors */

    /* Handle based on current state */
    if (fhss_cfg.state == FHSS_STATE_RX_SYNC) {
        /* CRC error during sync - restart scan immediately.
         * This is faster than waiting for fhss_poll timeout (500ms). */
        printf("FHSS: RX error during sync (crc=%d len=%d hdr=%d), restarting scan\r\n",
               crc_error, len_error, hdr_error);
        fhss_start_scan();
    } else if (fhss_cfg.state == FHSS_STATE_RX_DATA) {
        /* CRC error during data mode.
         * Use RSSI to distinguish real TX packets from noise:
         * - Real packets have strong RSSI (typically -40 to -60 dBm at short range)
         * - Noise near sensitivity has weak RSSI (below -90 dBm)
         * Don't use SNR - near sensitivity, real packets can have SNR below -10dB.
         * RSSI threshold of -80 dBm provides good margin. */
        bool is_real_packet = (rssi >= -80.0f);

        if (!is_real_packet) {
            /* Weak signal (noise) - don't count as packet, just restart RX */
            uint8_t ch = fhss_cfg.current_channel;
            lorahal.standbyXosc();
            lorahal.rx(0);
            printf("FHSS: RX noise on ch%u (rssi=%.1f, snr=%.1f), ignoring\r\n", ch, rssi, snr);
            return;
        }

        /* Real packet with CRC error - TX is transmitting on this channel.
         * Count toward pkts_on_channel to maintain hop sync with TX.
         * Reset timeout counter AND extend deadline so RX waits for next packet.
         * Also clear terminate_spkr_rx in case the old deadline already passed. */
        extern uint32_t HAL_GetTick(void);
        extern volatile uint32_t terminate_spkr_at_tick;
        extern volatile uint8_t terminate_spkr_rx;
        fhss_cfg.rx_timeout_consec = 0;
        terminate_spkr_rx = 0;
        terminate_spkr_at_tick = HAL_GetTick() + (FHSS_MAX_DWELL_MS * 8 / 10);
        fhss_cfg.pkts_on_channel++;

        uint8_t expected_pkts = fhss_cfg.proactive_hop_count;
        uint8_t ch = fhss_cfg.current_channel;
        uint8_t pkts = fhss_cfg.pkts_on_channel;

        if (fhss_cfg.pkts_on_channel >= expected_pkts) {
            /* Received enough (good+bad) packets - TX should hop now.
             * Restart RX FIRST on new channel, then print.
             * NOTE: lr20xx.c already cleared FIFO before calling us. */
            lorahal.standbyXosc();
            fhss_hop();
            fhss_cfg.pkts_on_channel = 0;
            lorahal.rx(0);
            printf("FHSS: RX CRC error on ch%u (pkts=%u/%u, snr=%.1f), hopped\r\n",
                   ch, pkts, expected_pkts, snr);
        } else {
            /* More packets expected on this channel - stay.
             * Restart RX FIRST, then print.
             * NOTE: lr20xx.c already cleared FIFO before calling us. */
            lorahal.standbyXosc();
            lorahal.rx(0);
            printf("FHSS: RX CRC error on ch%u (pkts=%u/%u, snr=%.1f), staying\r\n",
                   ch, pkts, expected_pkts, snr);
        }
    }
    /* Ignore errors in other states */
}

/* Send another sync packet on a new channel (for repeat mechanism).
 * IMPORTANT: Don't use fhss_random_channel() as it advances the LFSR.
 * Use a simple offset from the saved next_channel instead. */
static void fhss_send_sync_repeat(void)
{
    /* Pick a different channel without advancing LFSR.
     * Use sync_count as offset to spread across the band. */
    uint8_t sync_channel = (fhss_cfg.tx_next_channel + fhss_cfg.tx_sync_count * 17) % FHSS_NUM_CHANNELS;
    fhss_set_channel(sync_channel);

    /* Prepare sync packet - use saved LFSR and next_channel from first sync.
     * Include current hop_count so late RX can fast-forward. */
    fhss_sync_pkt_t *sync_pkt = (fhss_sync_pkt_t *)lorahal.tx_buf;
    sync_pkt->marker = FHSS_SYNC_MARKER;
    sync_pkt->lfsr_state = fhss_cfg.tx_sync_lfsr;  /* Use saved LFSR, not current */
    sync_pkt->next_channel = fhss_cfg.tx_next_channel;
    sync_pkt->hop_count = tx_hop_count;  /* Current hop count */
    sync_pkt->sync_channel = sync_channel;  /* For ACK response */

    printf("FHSS: sync repeat %u/%u on ch%u, hops=%u\r\n",
           fhss_cfg.tx_sync_count, FHSS_SYNC_REPEAT_COUNT, sync_channel, tx_hop_count);

    lorahal.loRaPacketConfig(fhss_cfg.preamble_len_symb, false, true, false, FHSS_SYNC_PKT_SIZE);
    lorahal.send(FHSS_SYNC_PKT_SIZE);
}

/* Preamble length for re-sync packets during data mode.
 * Must be longer than RX sweep time (~150ms) so RX can catch it.
 * Also must be long enough that RX can restart after preamble detection
 * and still have enough symbols to acquire sync word.
 * 300 symbols = ~307ms at SF7/125kHz - provides good margin. */
#define FHSS_RESYNC_PREAMBLE_LEN 300

/* Send a re-sync packet during data mode to help late RX catch up.
 * Uses longer preamble and includes current hop_count.
 * Called from fhss_send_data() when tx_resync_pending is true.
 * Returns after TX completes (blocking).
 *
 * IMPORTANT: Re-sync uses the SAME implicit header mode as data packets,
 * with the same packet length (51 bytes). This allows RX to receive it
 * without mode switching. RX distinguishes sync from data by the marker byte.
 * The sync info is in the first 5 bytes; remaining bytes are zero-padded. */
static void fhss_send_resync_packet(void)
{
    /* Build re-sync packet with current hop count.
     * Use full data packet length so RX in implicit mode can receive it. */
    memset(lorahal.tx_buf, 0, fhss_cfg.data_pkt_len);  /* Zero-pad */
    fhss_sync_pkt_t *sync_pkt = (fhss_sync_pkt_t *)lorahal.tx_buf;
    sync_pkt->marker = FHSS_SYNC_MARKER;
    sync_pkt->lfsr_state = fhss_cfg.tx_sync_lfsr;  /* Original LFSR state */
    sync_pkt->next_channel = fhss_cfg.tx_next_channel;  /* Original next channel */
    sync_pkt->hop_count = tx_hop_count;  /* Current hop count */
    sync_pkt->sync_channel = fhss_cfg.current_channel;  /* For ACK response */

    printf("FHSS: re-sync on ch%u, hops=%u, preamble=%u\r\n",
           fhss_cfg.current_channel, tx_hop_count, FHSS_RESYNC_PREAMBLE_LEN);

    /* Use implicit header mode (same as data) with longer preamble.
     * RX will receive it as a 51-byte packet and check marker to identify as sync. */
    lorahal.loRaPacketConfig(FHSS_RESYNC_PREAMBLE_LEN,
                              true,   /* fixLen = implicit header (same as data) */
                              true,   /* crcOn */
                              false,  /* invIQ */
                              fhss_cfg.data_pkt_len);  /* Same length as data */
    lorahal.send(fhss_cfg.data_pkt_len);

    /* Wait for TX to complete */
    while (lorahal.service())
        ;

    /* Restore normal data preamble length (packet config unchanged) */
    lorahal.loRaPacketConfig(FHSS_DATA_PREAMBLE_LEN,
                              true,   /* fixLen = implicit header */
                              true,   /* crcOn */
                              false,  /* invIQ */
                              fhss_cfg.data_pkt_len);

    tx_resync_pending = false;
}

void fhss_tx_done_handler(void)
{
    extern uint32_t HAL_GetTick(void);

    if (fhss_cfg.state == FHSS_STATE_TX_PREAMBLE) {
        if (fhss_cfg.tx_continuous) {
            /* Continuous mode: Send another preamble on a new random channel */
            fhss_start_tx_sync();
        } else if (fhss_cfg.ack_mode_enabled) {
            /* ACK mode: after sending sync, wait for ACK from RX.
             * Auto RX was configured before TX, so radio is already listening.
             * Just update state and wait for RxDone or RxTimeout. */
            fhss_cfg.state = FHSS_STATE_TX_WAIT_ACK;
            fhss_cfg.ack_wait_start_ms = HAL_GetTick();
            fhss_cfg.ack_received = false;

            printf("FHSS: sync sent on ch%u, auto-RX waiting for ACK (timeout=%u ms)\r\n",
                   fhss_cfg.tx_sync_channel, FHSS_ACK_TIMEOUT_MS);
        } else if (fhss_cfg.tx_sync_count < FHSS_SYNC_REPEAT_COUNT) {
            /* Send another sync on a different channel to give RX more chances */
            fhss_cfg.tx_sync_count++;
            fhss_send_sync_repeat();
        } else {
            /* All sync repeats done, hop to the promised next_channel.
             * This uses the channel we stored in fhss_start_tx_sync() without
             * advancing the LFSR again - fixes the double-advance bug. */
            uint8_t old_ch = fhss_cfg.current_channel;
            fhss_set_channel(fhss_cfg.tx_next_channel);

            /* Extended first dwell: Give RX extra time to arrive at this channel.
             * RX may take 1-2 seconds to catch sync and process it.
             * Set dwell_start_ms to 0 as a flag - fhss_send_data will handle it. */
            fhss_cfg.dwell_start_ms = 0;  /* Special: first channel, delay hop */
            fhss_cfg.pkts_on_channel = 0;

            fhss_cfg.state = FHSS_STATE_TX_DATA;
            printf("FHSS: sync TX done, hop ch%u->ch%u, ready for data, dwell_ms=%lu\r\n",
                   old_ch, fhss_cfg.tx_next_channel, fhss_cfg.dwell_start_ms);
        }
    } else if (fhss_cfg.state == FHSS_STATE_RX_SEND_ACK) {
        /* RX finished sending ACK - now configure data mode and start receiving */
        uint8_t data_ch = fhss_cfg.tx_next_channel;  /* Saved in fhss_send_sync_ack */
        fhss_set_channel(data_ch);

        printf("FHSS: ACK sent, switching to data ch%u\r\n", data_ch);

        /* Configure data mode */
        uint8_t rx_payload = fhss_get_rx_payload_len();
        if (rx_payload > 0) {
            fhss_configure_data_mode(rx_payload);
        } else {
            printf("FHSS: warning - RX payload length not set\r\n");
        }

        fhss_cfg.state = FHSS_STATE_RX_DATA;

        /* Start RX for data packets */
        fhss_rx_data();
        printf("FHSS: waiting for data on ch%u\r\n", fhss_cfg.current_channel);
    } else if (fhss_cfg.state == FHSS_STATE_TX_DATA) {
        /* Data packet TX done - stay in TX_DATA state for next packet */
    } else if (fhss_cfg.state == FHSS_STATE_IDLE) {
        /* Already IDLE - nothing to do */
    }
    /* Note: RX states (RX_SCAN, RX_SYNC, RX_DATA) are not touched here.
     * txDoneCB() may have started a scan before this handler runs. */
}

bool fhss_is_tx_sync(void)
{
    return fhss_cfg.state == FHSS_STATE_TX_PREAMBLE;
}

bool fhss_is_tx_data_ready(void)
{
    return fhss_cfg.state == FHSS_STATE_TX_DATA;
}

/* Helper to wait for TX to complete.
 * Polls service() for IRQ_TX_DONE which updates LR20xx_chipMode. */
static void wait_for_tx_complete(void)
{
    extern lr20xx_system_chip_modes_t LR20xx_chipMode;
    extern uint32_t HAL_GetTick(void);
    uint32_t wait_start = HAL_GetTick();

    while (LR20xx_chipMode == LR20XX_SYSTEM_CHIP_MODE_TX) {
        lorahal.service();
        if (HAL_GetTick() - wait_start > 200) {
            /* Timeout - force standby */
            lorahal.standby();
            LR20xx_chipMode = LR20XX_SYSTEM_CHIP_MODE_STBY_RC;
            break;
        }
    }
}

void fhss_send_end_packet(void)
{
    /* Send end-of-transmission marker to RX.
     * Uses same packet length as data packets (implicit header mode).
     * RX checks first byte (marker) to identify packet type.
     *
     * TIMING CRITICAL: RX timeout is 320ms after last packet.
     * If we've sent proactive_hop_count packets on current channel,
     * RX has ALREADY hopped to the next channel. Sending end on
     * current channel is useless and wastes 268ms (TX + wait).
     *
     * Strategy:
     * - If pkts_on_channel >= proactive_hop_count: RX has hopped, skip current
     *   channel and send on next 2 channels. Total time ~268ms < 320ms timeout.
     * - If pkts_on_channel < proactive_hop_count: RX is still here, send on
     *   current + next channel. */
    if (fhss_cfg.state != FHSS_STATE_TX_DATA) {
        return;  /* Only send if we were in data mode */
    }

    /* Build end packet - same length as data packet for implicit mode compatibility */
    memset(lorahal.tx_buf, 0, fhss_cfg.data_pkt_len);
    lorahal.tx_buf[0] = FHSS_END_MARKER;

    /* Wait for any in-progress TX to complete */
    wait_for_tx_complete();

    uint8_t first_ch = fhss_cfg.current_channel;

    /* Check if RX has likely already hopped */
    if (fhss_cfg.pkts_on_channel >= fhss_cfg.proactive_hop_count) {
        /* RX has already hopped to next channel - send end there.
         * IMPORTANT: Don't hop again after first end! RX stays on that
         * channel for 320ms (timeout). If we hop and send on next channel,
         * RX won't be there yet.
         * Send twice on same channel in case first is lost to noise. */
        fhss_hop();
        uint8_t rx_channel = fhss_cfg.current_channel;
        printf("FHSS: end on ch%u (skipped ch%u, RX hopped)\r\n",
               rx_channel, first_ch);
        lorahal.send(fhss_cfg.data_pkt_len);
        wait_for_tx_complete();

        /* Delay to let RX restart after processing first end packet.
         * RX needs ~5-10ms to handle CRC error IRQ and restart.
         * Preamble is 8 symbols = ~16ms at SF8, so 20ms delay ensures
         * RX catches most of the second packet's preamble. */
        extern void HAL_Delay(uint32_t);
        HAL_Delay(20);

        /* Send again on same channel in case RX was processing noise */
        printf("FHSS: end on ch%u (repeat)\r\n", rx_channel);
        lorahal.send(fhss_cfg.data_pkt_len);
        wait_for_tx_complete();

        printf("FHSS: end packets sent on ch%u (x2)\r\n", rx_channel);
    } else {
        /* RX is still on current channel - send twice here.
         * Don't hop to next channel because RX won't be there yet.
         * RX is waiting for more packets on this channel. */
        printf("FHSS: end on ch%u\r\n", first_ch);
        lorahal.send(fhss_cfg.data_pkt_len);
        wait_for_tx_complete();

        /* Delay to let RX restart after processing first end packet */
        extern void HAL_Delay(uint32_t);
        HAL_Delay(20);

        /* Send again on same channel in case first is lost to noise */
        printf("FHSS: end on ch%u (repeat)\r\n", first_ch);
        lorahal.send(fhss_cfg.data_pkt_len);
        wait_for_tx_complete();

        printf("FHSS: end packets sent on ch%u (x2)\r\n", first_ch);
    }
}

void fhss_tx_stop(void)
{
    if (fhss_cfg.state == FHSS_STATE_TX_PREAMBLE ||
        fhss_cfg.state == FHSS_STATE_TX_DATA ||
        fhss_cfg.state == FHSS_STATE_TX_WAIT_ACK) {

        /* Send end packet to RX so it can stop audio immediately */
        if (fhss_cfg.state == FHSS_STATE_TX_DATA) {
            fhss_send_end_packet();
        }

        printf("FHSS: TX stopped (data_cnt=%lu)\r\n", fhss_send_data_cnt);
        fhss_cfg.state = FHSS_STATE_IDLE;
        fhss_send_data_cnt = 0;

        /* Return to CAD scan after TX stops */
        fhss_start_scan();
    }
}

uint16_t fhss_calc_sync_preamble_len(uint8_t sf, uint16_t bw_khz)
{
    /* Preamble must be long enough that RX can catch it even if it gets
     * stuck on a false CAD detection on a different channel.
     *
     * With our two-stage timeout approach:
     * 1. Initial timeout is short (CAD_VERIFY_TIMEOUT_MS = 50ms)
     * 2. Extended only after 3+ preamble confirmations
     *
     * Required preamble duration:
     * - Full sweep time to ensure RX passes sync channel: ~50ms
     * - One false CAD penalty: ~50ms (short initial timeout)
     * - Margin for second false CAD and timing: 50ms
     * - Plus enough time for RX to detect and extend timeout
     *
     * Target: ~200-250ms preamble = sufficient for 2-3 false CADs
     */

    uint32_t symbol_time_us = (1UL << sf) * 1000 / bw_khz;  /* microseconds */

    /* Base sweep time for all channels (pessimistic) */
    uint32_t cad_time_us = symbol_time_us * fhss_cfg.cad_cfg.cad_symb_nb + 1000;
    uint32_t sweep_time_us = cad_time_us * FHSS_NUM_CHANNELS;

    /* False CAD penalty with short initial timeout */
    uint32_t false_cad_penalty_us = CAD_VERIFY_TIMEOUT_MS * 1000;

    /* Preamble = sweep + 2 false CADs + margin
     * This ensures RX can survive 2 false CADs and still find sync */
    uint32_t total_us = sweep_time_us + (2 * false_cad_penalty_us);

    /* Add 50% margin for reliability */
    total_us = total_us * 3 / 2;

    /* FCC Part 15.247 limits transmission on any single channel to 400ms.
     * This applies to sync preambles too. Cap at 350ms to leave headroom
     * for the sync packet that follows the preamble. */
    if (total_us > (FHSS_MAX_DWELL_MS * 1000 - 50000))
        total_us = FHSS_MAX_DWELL_MS * 1000 - 50000;

    /* Convert to preamble symbols */
    uint16_t preamble_symbols = (total_us + symbol_time_us - 1) / symbol_time_us;

    /* Minimum 64 symbols for reliable detection */
    if (preamble_symbols < 64)
        preamble_symbols = 64;

    return preamble_symbols;
}

void fhss_print_status(void)
{
    printf("\r\n=== FHSS Status ===\r\n");
    printf("Enabled: %s\r\n", fhss_cfg.enabled ? "yes" : "no");
    printf("State: ");
    switch (fhss_cfg.state) {
        case FHSS_STATE_IDLE:        printf("IDLE\r\n"); break;
        case FHSS_STATE_TX_PREAMBLE: printf("TX_PREAMBLE\r\n"); break;
        case FHSS_STATE_TX_WAIT_ACK: printf("TX_WAIT_ACK\r\n"); break;
        case FHSS_STATE_TX_DATA:     printf("TX_DATA\r\n"); break;
        case FHSS_STATE_RX_SCAN:     printf("RX_SCAN\r\n"); break;
        case FHSS_STATE_RX_SYNC:     printf("RX_SYNC\r\n"); break;
        case FHSS_STATE_RX_SEND_ACK: printf("RX_SEND_ACK\r\n"); break;
        case FHSS_STATE_RX_DATA:     printf("RX_DATA\r\n"); break;
    }
    printf("Current channel: %u (%.3f MHz)\r\n",
           fhss_cfg.current_channel,
           (float)fhss_get_channel_freq(fhss_cfg.current_channel) / 1000000.0f);
    printf("Sync preamble: %u symbols\r\n", fhss_cfg.preamble_len_symb);
    printf("\r\nCAD config:\r\n");
    printf("  Symbols: %u\r\n", fhss_cfg.cad_cfg.cad_symb_nb);
    printf("  Detect peak: %u\r\n", fhss_cfg.cad_cfg.cad_detect_peak);
    printf("  PNR delta: %u\r\n", fhss_cfg.cad_cfg.pnr_delta);
    printf("  Timeout: %lu ms\r\n", fhss_cfg.cad_cfg.cad_timeout_ms);
    printf("===================\r\n\r\n");
}

void fhss_print_stats(void)
{
    printf("\r\n=== FHSS Statistics ===\r\n");
    printf("CAD operations: %lu\r\n", fhss_cfg.stats.cad_done_count);
    printf("CAD detections: %lu\r\n", fhss_cfg.stats.cad_detected_count);
    printf("CAD false positives: %lu\r\n", fhss_cfg.stats.cad_false_count);
    printf("Channels scanned: %lu\r\n", fhss_cfg.stats.channels_scanned);
    printf("Sync acquisitions: %lu\r\n", fhss_cfg.stats.sync_found_count);
    if (fhss_cfg.stats.sync_found_count > 0) {
        printf("Last sync time: %lu ms\r\n", fhss_cfg.stats.sync_time_ms);
    }
    printf("RX timeouts: %lu\r\n", fhss_cfg.stats.rx_timeout_count);
    printf("Resyncs (returned to scan): %lu\r\n", fhss_cfg.stats.resync_count);
    printf("=======================\r\n\r\n");
}

/* UART command interface for CAD parameter tuning
 * Commands:
 *   H - Toggle FHSS enable
 *   h - Start/stop CAD scan
 *   1-4 - Set CAD symbol count
 *   + - Increase detect_peak
 *   - - Decrease detect_peak
 *   p - Toggle PNR delta (0 or 8)
 *   c - Print current channel
 *   s - Print FHSS status
 *   S - Print FHSS statistics
 *   R - Reset statistics
 *   n - Hop to next random channel
 *   0-9 - Set channel (0-9 or 10*digit for 10-49)
 */
void fhss_uart_command(char cmd)
{
    switch (cmd) {
        case 'H':
            fhss_cfg.enabled = !fhss_cfg.enabled;
            printf("FHSS %s\r\n", fhss_cfg.enabled ? "enabled" : "disabled");
            if (fhss_cfg.enabled) {
                /* Start CAD scan when FHSS is enabled (RX side) */
                fhss_start_scan();
            } else {
                /* Stop scanning and return to idle */
                fhss_cfg.state = FHSS_STATE_IDLE;
                lorahal.standby();
            }
            break;

        case 'h':
            if (fhss_cfg.state == FHSS_STATE_RX_SCAN) {
                fhss_cfg.state = FHSS_STATE_IDLE;
                printf("FHSS scan stopped\r\n");
            } else {
                fhss_start_scan();
            }
            break;

        case '1': case '2': case '3': case '4':
            fhss_cfg.cad_cfg.cad_symb_nb = cmd - '0';
            fhss_configure_cad(&fhss_cfg.cad_cfg);
            break;

        case '+':
            if (fhss_cfg.cad_cfg.cad_detect_peak < 100)
                fhss_cfg.cad_cfg.cad_detect_peak++;
            printf("CAD detect_peak: %u\r\n", fhss_cfg.cad_cfg.cad_detect_peak);
            fhss_configure_cad(&fhss_cfg.cad_cfg);
            break;

        case '-':
            if (fhss_cfg.cad_cfg.cad_detect_peak > 20)
                fhss_cfg.cad_cfg.cad_detect_peak--;
            printf("CAD detect_peak: %u\r\n", fhss_cfg.cad_cfg.cad_detect_peak);
            fhss_configure_cad(&fhss_cfg.cad_cfg);
            break;

        case 'p':
            fhss_cfg.cad_cfg.pnr_delta = (fhss_cfg.cad_cfg.pnr_delta == 0) ? 8 : 0;
            printf("CAD pnr_delta: %u (best-effort %s)\r\n",
                   fhss_cfg.cad_cfg.pnr_delta,
                   fhss_cfg.cad_cfg.pnr_delta ? "ON" : "OFF");
            fhss_configure_cad(&fhss_cfg.cad_cfg);
            break;

        case 'c':
            printf("Channel %u: %.3f MHz\r\n",
                   fhss_cfg.current_channel,
                   (float)fhss_get_channel_freq(fhss_cfg.current_channel) / 1000000.0f);
            break;

        case 'i':
            fhss_print_status();
            break;

        case 'I':
            fhss_print_stats();
            break;

        case 'R':
            fhss_cfg.stats.cad_done_count = 0;
            fhss_cfg.stats.cad_detected_count = 0;
            fhss_cfg.stats.cad_false_count = 0;
            fhss_cfg.stats.channels_scanned = 0;
            fhss_cfg.stats.sync_found_count = 0;
            printf("FHSS statistics reset\r\n");
            break;

        case 'n':
            fhss_hop();
            printf("Hopped to ch%u: %.3f MHz\r\n",
                   fhss_cfg.current_channel,
                   (float)fhss_get_channel_freq(fhss_cfg.current_channel) / 1000000.0f);
            break;

        case 'P':
            /* Toggle continuous preamble TX mode */
            if (fhss_cfg.state == FHSS_STATE_TX_PREAMBLE) {
                /* Stop continuous TX */
                fhss_cfg.tx_continuous = false;
                printf("Stopping continuous TX after current preamble\r\n");
            } else {
                /* Start continuous TX */
                fhss_cfg.tx_continuous = true;
                fhss_start_tx_sync();
            }
            break;

        case 'L':
            /* Adjust preamble length: L increases (max 65535, but practical limit ~2000 for 400ms dwell) */
            if (fhss_cfg.preamble_len_symb < 2000)
                fhss_cfg.preamble_len_symb += 16;
            printf("Preamble length: %u symbols\r\n", fhss_cfg.preamble_len_symb);
            break;

        case 'l':
            /* Decrease preamble length */
            if (fhss_cfg.preamble_len_symb > 16)
                fhss_cfg.preamble_len_symb -= 16;
            printf("Preamble length: %u symbols\r\n", fhss_cfg.preamble_len_symb);
            break;

        case 'A':
            /* Enable ACK-based sync mode */
            if (!fhss_cfg.ack_mode_enabled)
                fhss_set_ack_mode(true);
            else
                printf("FHSS: ACK mode already enabled\r\n");
            break;

        case 'a':
            /* Disable ACK-based sync mode */
            if (fhss_cfg.ack_mode_enabled)
                fhss_set_ack_mode(false);
            else
                printf("FHSS: ACK mode already disabled\r\n");
            break;

        default:
            break;
    }
}

bool fhss_poll(void)
{
    extern uint32_t HAL_GetTick(void);
    uint32_t start_ch = fhss_cfg.stats.channels_scanned;
    uint32_t timeout = HAL_GetTick() + 500;  /* 500ms timeout */

    if (fhss_cfg.state != FHSS_STATE_RX_SCAN &&
        fhss_cfg.state != FHSS_STATE_RX_SYNC) {
        return false;  /* Not scanning or waiting for sync */
    }

    /* Tight polling loop - service radio until CAD done for current channel
     * or until preamble confirmed/timeout in RX_SYNC state */
    while (fhss_cfg.state == FHSS_STATE_RX_SCAN ||
           fhss_cfg.state == FHSS_STATE_RX_SYNC) {
        lorahal.service();

        /* Check if a channel was scanned (channels_scanned incremented) */
        if (fhss_cfg.stats.channels_scanned != start_ch) {
            start_ch = fhss_cfg.stats.channels_scanned;  /* Reset for next iteration */
            return true;  /* Still scanning, can continue */
        }

        /* Safety timeout - if radio IRQ didn't fire, force return to scanning */
        if (HAL_GetTick() > timeout) {
            extern lr20xx_system_chip_modes_t LR20xx_chipMode;
            printf("fhss_poll timeout: state=%d chipMode=%d ch_scanned=%lu\r\n",
                   fhss_cfg.state, LR20xx_chipMode, fhss_cfg.stats.channels_scanned);
            if (fhss_cfg.state == FHSS_STATE_RX_SYNC) {
                printf("fhss_poll: forcing return to scan from RX_SYNC\r\n");
                /* Restart scan (also prints RX stats) */
                fhss_start_scan();
            }
            return false;
        }
    }

    /* State changed (detected signal or stopped) */
    return false;
}
