/**
 * @file fhss.c
 * @brief Frequency Hopping Spread Spectrum implementation for LR20xx
 */

#include <stdio.h>
#include <stdlib.h>
#include "fhss.h"
#include "radio.h"
#include "lr20xx_radio_lora.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_system.h"
#include "lr20xx_hal.h"

/* Access chip status from lr20xx_hal.c */
extern lr20xx_stat_t stat;

/* Global FHSS configuration */
fhss_config_t fhss_cfg;

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
    fhss_cfg.enabled = false;
    fhss_cfg.tx_continuous = false;
    fhss_cfg.current_channel = 0;
    fhss_cfg.state = FHSS_STATE_IDLE;
    fhss_cfg.preamble_len_symb = 128;  /* Default sync preamble length */

    /* Data packet configuration */
    fhss_cfg.data_pkt_len = 0;  /* Set by fhss_configure_data_mode() */
    fhss_cfg.tx_seq_num = 0;
    fhss_cfg.rx_seq_num = 0;
    fhss_cfg.dwell_start_ms = 0;
    fhss_cfg.pkts_on_channel = 0;

    /* Default CAD configuration: 2 symbols, best-effort enabled */
    fhss_cfg.cad_cfg.cad_symb_nb = 2;
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

    /* Seed LFSR with a varying value (could use ADC noise, timer, etc.) */
    extern uint32_t HAL_GetTick(void);
    lfsr_state = (uint16_t)(HAL_GetTick() ^ 0xACE1);
    if (lfsr_state == 0) lfsr_state = 0xACE1;

    printf("FHSS initialized: %u channels, %.1f MHz - %.1f MHz\r\n",
           FHSS_NUM_CHANNELS,
           (float)FHSS_BASE_FREQ_HZ / 1000000.0f,
           (float)(FHSS_BASE_FREQ_HZ + (FHSS_NUM_CHANNELS - 1) * FHSS_CHANNEL_STEP_HZ) / 1000000.0f);
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

    /* Select a different random channel */
    do {
        new_channel = fhss_random_channel();
    } while (new_channel == fhss_cfg.current_channel);

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

/* Calculate optimal CAD timeout in ms based on SF and BW.
 * Timeout should be ~8 symbol periods - enough time to confirm preamble
 * if signal is real, but short enough that false detections don't waste
 * much time.
 * Symbol time (ms) = 2^SF / BW_kHz
 * Timeout = 8 * symbol_time + small overhead
 */
uint32_t fhss_calc_cad_timeout_ms(uint8_t sf, uint16_t bw_khz)
{
    /* Calculate symbol time in microseconds: (2^sf * 1000) / bw_khz */
    uint32_t symbol_time_us = ((1UL << sf) * 1000UL) / bw_khz;

    /* 8 symbol periods + 2ms overhead for processing */
    uint32_t timeout_ms = (8 * symbol_time_us + 999) / 1000 + 2;

    /* Minimum 5ms, maximum 100ms */
    if (timeout_ms < 5)
        timeout_ms = 5;
    if (timeout_ms > 100)
        timeout_ms = 100;

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

    /* Calculate optimal timeout based on SF and BW if not explicitly set */
    if (cfg->cad_timeout_ms == 0 || cfg->cad_timeout_ms == 100) {
        /* Default or old value - calculate optimal timeout */
        fhss_cfg.cad_cfg.cad_timeout_ms = fhss_calc_cad_timeout_ms(sf, bw_khz);
    }

    cad_params.cad_symb_nb = cfg->cad_symb_nb;
    cad_params.pnr_delta = cfg->pnr_delta;
    cad_params.cad_detect_peak = fhss_cfg.cad_cfg.cad_detect_peak;

    /* CAD_RX mode: if CAD detects activity, go to RX to receive packet */
    cad_params.cad_exit_mode = LR20XX_RADIO_LORA_CAD_EXIT_MODE_RX;

    /* Timeout in PLL steps (30.52µs per step) */
    cad_params.cad_timeout_in_pll_step = (fhss_cfg.cad_cfg.cad_timeout_ms * 1000) / 31;
    if (cad_params.cad_timeout_in_pll_step > 0x00FFFFFF)
        cad_params.cad_timeout_in_pll_step = 0x00FFFFFF;

    /* Must be in standby to configure CAD parameters.
     * Use XOSC standby for faster FS transitions during scanning. */
    lorahal.standbyXosc();
    lr20xx_radio_lora_configure_cad_params(NULL, &cad_params);

    printf("CAD configured: symb=%u, peak=%u, pnr_delta=%u, timeout=%lu ms (SF%u BW%ukHz)\r\n",
           cfg->cad_symb_nb, fhss_cfg.cad_cfg.cad_detect_peak,
           cfg->pnr_delta, fhss_cfg.cad_cfg.cad_timeout_ms, sf, bw_khz);
}

void fhss_start_cad(void)
{
    lr20xx_radio_lora_set_cad(NULL);
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
        printf("CAD detected on ch%u (%.3f MHz) mode=%s\r\n",
               fhss_cfg.current_channel,
               (float)fhss_get_channel_freq(fhss_cfg.current_channel) / 1000000.0f,
               chip_mode_name(chip_mode));

        if (fhss_cfg.state == FHSS_STATE_RX_SCAN) {
            /* Found signal during scan - switch to sync state */
            fhss_cfg.state = FHSS_STATE_RX_SYNC;
            /* Radio should already be in RX mode (CAD_RX exit mode) */
        }
    } else {
        /* No detection - continue scanning if in scan mode */
		/*printf("no detection on ch%u mode=%s\r\n",
               fhss_cfg.current_channel, chip_mode_name(chip_mode));*/
        if (fhss_cfg.state == FHSS_STATE_RX_SCAN) {
            fhss_scan_next_channel();
        }
    }
}

void fhss_preamble_detected_handler(void)
{
    /* Preamble detected confirms CAD detection was a real signal, not noise */
    if (fhss_cfg.state == FHSS_STATE_RX_SYNC) {
        printf("Preamble confirmed on ch%u (%.3f MHz)\r\n",
               fhss_cfg.current_channel,
               (float)fhss_get_channel_freq(fhss_cfg.current_channel) / 1000000.0f);
        fhss_cfg.stats.sync_found_count++;
        /* Transition to RX_DATA state - waiting for full packet */
        fhss_cfg.state = FHSS_STATE_RX_DATA;
    }
}

void fhss_timeout_handler(void)
{
    /* Timeout during RX_SYNC means CAD detection was a false positive.
     * The preamble was not confirmed within the timeout period.
     * Resume scanning on the next channel. */
    if (fhss_cfg.state == FHSS_STATE_RX_SYNC) {
        fhss_cfg.stats.cad_false_count++;
        printf("False CAD on ch%u, resuming scan\r\n", fhss_cfg.current_channel);
        fhss_cfg.state = FHSS_STATE_RX_SCAN;
        fhss_scan_next_channel();
    }
}

void fhss_start_scan(void)
{
    extern uint32_t HAL_GetTick(void);

    fhss_cfg.state = FHSS_STATE_RX_SCAN;
    fhss_cfg.stats.channels_scanned = 0;
    fhss_cfg.stats.sync_time_ms = HAL_GetTick();
    fhss_cfg.stats.sweep_start_ms = HAL_GetTick();
    fhss_cfg.stats.sweep_count = 0;

    printf("FHSS scan starting...\r\n");

    /* Configure CAD parameters (puts radio in standby) */
    fhss_configure_cad(&fhss_cfg.cad_cfg);

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

    /* Prepare sync packet with current LFSR state and next channel */
    fhss_sync_pkt_t *sync_pkt = (fhss_sync_pkt_t *)lorahal.tx_buf;
    sync_pkt->marker = FHSS_SYNC_MARKER;
    sync_pkt->lfsr_state = lfsr_state;
    sync_pkt->next_channel = fhss_random_channel();  /* Pre-compute next channel */

    printf("TX sync on ch%u (%.3f MHz), lfsr=0x%04X, next_ch=%u, preamble=%u\r\n",
           sync_channel,
           (float)fhss_get_channel_freq(sync_channel) / 1000000.0f,
           sync_pkt->lfsr_state,
           sync_pkt->next_channel,
           fhss_cfg.preamble_len_symb);

    /* Configure long preamble and send sync packet */
    lorahal.loRaPacketConfig(fhss_cfg.preamble_len_symb, false, true, false);
    lorahal.send(FHSS_SYNC_PKT_SIZE);
}

bool fhss_rx_sync_packet(const uint8_t *data, uint8_t size)
{
    if (size < FHSS_SYNC_PKT_SIZE) {
        return false;
    }

    const fhss_sync_pkt_t *sync_pkt = (const fhss_sync_pkt_t *)data;

    if (sync_pkt->marker != FHSS_SYNC_MARKER) {
        printf("FHSS: Invalid sync marker 0x%02X\r\n", sync_pkt->marker);
        return false;
    }

    /* Synchronize LFSR state with transmitter */
    fhss_set_lfsr_state(sync_pkt->lfsr_state);

    printf("FHSS sync: lfsr=0x%04X, next_ch=%u (%.3f MHz)\r\n",
           sync_pkt->lfsr_state,
           sync_pkt->next_channel,
           (float)fhss_get_channel_freq(sync_pkt->next_channel) / 1000000.0f);

    /* Hop to the next channel that TX will use */
    fhss_set_channel(sync_pkt->next_channel);

    fhss_cfg.state = FHSS_STATE_RX_DATA;
    return true;
}

void fhss_configure_data_mode(uint8_t payload_len)
{
    extern uint32_t HAL_GetTick(void);

    /* Store data packet length (codec2 payload + FHSS header) */
    fhss_cfg.data_pkt_len = payload_len + FHSS_DATA_HDR_SIZE;

    /* Reset sequence numbers */
    fhss_cfg.tx_seq_num = 0;
    fhss_cfg.rx_seq_num = 0;

    /* Start dwell timer */
    fhss_cfg.dwell_start_ms = HAL_GetTick();
    fhss_cfg.pkts_on_channel = 0;

    /* Configure radio for implicit header mode with fixed packet length */
    lorahal.loRaPacketConfig(FHSS_DATA_PREAMBLE_LEN,
                              true,   /* fixLen = implicit header */
                              true,   /* crcOn */
                              false); /* invIQ */

    printf("FHSS data mode: pkt_len=%u (payload=%u + hdr=%u), implicit header\r\n",
           fhss_cfg.data_pkt_len, payload_len, FHSS_DATA_HDR_SIZE);
}

void fhss_check_hop(void)
{
    extern uint32_t HAL_GetTick(void);
    uint32_t now = HAL_GetTick();
    uint32_t dwell_time = now - fhss_cfg.dwell_start_ms;

    /* Check if we've exceeded 90% of max dwell time (leave margin) */
    if (dwell_time >= (FHSS_MAX_DWELL_MS * 9 / 10)) {
        uint8_t old_ch = fhss_cfg.current_channel;

        /* Hop to next channel using synchronized LFSR */
        fhss_hop();

        /* Reset dwell timer */
        fhss_cfg.dwell_start_ms = now;
        fhss_cfg.pkts_on_channel = 0;

        printf("FHSS hop: ch%u->ch%u after %lums\r\n",
               old_ch, fhss_cfg.current_channel, dwell_time);
    }
}

int fhss_send_data(const uint8_t *data, uint8_t len)
{
    fhss_data_hdr_t *hdr;

    if (len + FHSS_DATA_HDR_SIZE != fhss_cfg.data_pkt_len) {
        printf("FHSS: payload len %u != configured %u\r\n",
               len, fhss_cfg.data_pkt_len - FHSS_DATA_HDR_SIZE);
        return -1;
    }

    /* Check if we need to hop before sending */
    fhss_check_hop();

    /* Build packet: header + payload */
    hdr = (fhss_data_hdr_t *)lorahal.tx_buf;
    hdr->marker = FHSS_DATA_MARKER;
    hdr->seq_num = fhss_cfg.tx_seq_num++;
    hdr->channel = fhss_cfg.current_channel;

    /* Copy payload after header */
    for (uint8_t i = 0; i < len; i++) {
        lorahal.tx_buf[FHSS_DATA_HDR_SIZE + i] = data[i];
    }

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
        return -1;
    }

    hdr = (const fhss_data_hdr_t *)data;

    if (hdr->marker != FHSS_DATA_MARKER) {
        /* Not a data packet - might be sync packet or garbage */
        return -1;
    }

    /* Verify channel - resync if different */
    if (hdr->channel != fhss_cfg.current_channel) {
        printf("FHSS: channel mismatch, resyncing ch%u->ch%u\r\n",
               fhss_cfg.current_channel, hdr->channel);
        fhss_set_channel(hdr->channel);
        fhss_cfg.dwell_start_ms = 0;  /* Reset dwell timer - TX controls timing */
        fhss_cfg.pkts_on_channel = 0;
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
            /* Duplicate/old packet */
            printf("FHSS: duplicate packet (seq %u, expected %u)\r\n",
                   hdr->seq_num, expected_seq);
            return -1;
        }
    }

    /* Update expected sequence number */
    fhss_cfg.rx_seq_num = hdr->seq_num + 1;
    fhss_cfg.pkts_on_channel++;

    /* Return payload length (excluding header) */
    return size - FHSS_DATA_HDR_SIZE;
}

void fhss_rx_data(void)
{
    /* Check if we need to hop */
    fhss_check_hop();

    /* Start RX with timeout */
    lorahal.rx(0);  /* 0 = continuous RX, or set appropriate timeout */
}

void fhss_tx_done_handler(void)
{
    if (fhss_cfg.state == FHSS_STATE_TX_PREAMBLE && fhss_cfg.tx_continuous) {
        /* Send another preamble on a new random channel */
        fhss_start_tx_sync();
    } else {
        fhss_cfg.state = FHSS_STATE_IDLE;
    }
}

uint16_t fhss_calc_sync_preamble_len(uint8_t sf, uint16_t bw_khz)
{
    /* Calculate worst-case sweep time based on observed measurements:
     *
     * Components of sweep time:
     * 1. CAD time per channel: ~symbol_time * cad_symb_nb + SPI overhead
     * 2. Total base sweep: CAD_time * 50 channels
     * 3. False CAD penalty: ~10ms each, worst case 2 per sweep = 20ms
     *
     * Observed at SF6/125kHz:
     *   - Normal sweep: ~48ms
     *   - Worst case (2 false CADs): ~70ms
     *
     * Preamble must exceed worst-case sweep time with margin.
     */

    uint32_t symbol_time_us = (1UL << sf) * 1000 / bw_khz;  /* microseconds */

    /* Base CAD time per channel (CAD symbols + SPI/processing overhead) */
    uint32_t cad_time_us = symbol_time_us * fhss_cfg.cad_cfg.cad_symb_nb + 500;

    /* Total base sweep time for all channels */
    uint32_t base_sweep_us = cad_time_us * FHSS_NUM_CHANNELS;

    /* Add false CAD penalty: worst case ~2 false detections per sweep
     * Each false CAD costs the timeout period (~8 symbol periods + overhead) */
    uint32_t false_cad_timeout_us = fhss_calc_cad_timeout_ms(sf, bw_khz) * 1000;
    uint32_t false_cad_penalty_us = 2 * false_cad_timeout_us;

    /* Total worst-case sweep time */
    uint32_t worst_sweep_us = base_sweep_us + false_cad_penalty_us;

    /* Add 50% margin for reliability */
    uint32_t total_scan_us = worst_sweep_us * 3 / 2;

    /* Ensure under 400ms dwell limit (leave 50ms headroom) */
    if (total_scan_us > (FHSS_MAX_DWELL_MS * 1000 - 50000))
        total_scan_us = FHSS_MAX_DWELL_MS * 1000 - 50000;

    /* Convert to preamble symbols */
    uint16_t preamble_symbols = (total_scan_us + symbol_time_us - 1) / symbol_time_us;

    /* Minimum 16 symbols for reliable detection */
    if (preamble_symbols < 16)
        preamble_symbols = 16;

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
        case FHSS_STATE_TX_DATA:     printf("TX_DATA\r\n"); break;
        case FHSS_STATE_RX_SCAN:     printf("RX_SCAN\r\n"); break;
        case FHSS_STATE_RX_SYNC:     printf("RX_SYNC\r\n"); break;
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

        /* Safety timeout */
        if (HAL_GetTick() > timeout) {
            printf("fhss_poll timeout\r\n");
            return false;
        }
    }

    /* State changed (detected signal or stopped) */
    return false;
}
