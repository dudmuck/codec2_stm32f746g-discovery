
void print_streaming_timing_analysis(uint8_t codec2_frame_period_ms)
{
    uint32_t frame_toa = get_frame_toa_ms();
    uint32_t pkt_toa_ms;
    lr20xx_radio_lora_pkt_params_t pkt = lora_pkt_params;
    lr20xx_radio_lora_mod_params_t mod_test;
    uint8_t frames_per_packet;
    uint32_t codec2_production_time;
    uint8_t max_sf;

    printf("\r\n=== Streaming Timing Analysis ===\r\n");
    printf("SF:%u BW:%ukHz frame_size:%u bytes\r\n",
           lora_mod_params.sf, get_bw_khz_lr20xx(), _bytes_per_frame);
    printf("Single frame time-on-air: %lu ms (includes preamble overhead)\r\n", frame_toa);
    printf("Codec2 frame period: %u ms\r\n", codec2_frame_period_ms);
    printf("(Note: packet-level timing below is what matters for streaming)\r\n");

    /* Calculate packet timing */
    pkt.pld_len_in_bytes = lora_payload_length;
    pkt_toa_ms = lr20xx_radio_lora_get_time_on_air_in_ms(&pkt, &lora_mod_params);
    frames_per_packet = lora_payload_length / _bytes_per_frame;
    codec2_production_time = frames_per_packet * codec2_frame_period_ms;

    printf("Packet: %u bytes = %u frames\r\n", lora_payload_length, frames_per_packet);
    printf("Packet time-on-air: %lu ms\r\n", pkt_toa_ms);
    printf("Codec2 production time for packet: %lu ms\r\n", codec2_production_time);

    if (pkt_toa_ms <= codec2_production_time) {
        printf("OK: Encoder keeps up (margin: %lu ms, buffer accumulates)\r\n",
               codec2_production_time - pkt_toa_ms);
    } else {
        printf("WARNING: packet_toa > production_time - buffer will drain!\r\n");
    }

    /* Calculate maximum usable SF */
    printf("\r\n--- SF Feasibility Analysis ---\r\n");
    mod_test = lora_mod_params;
    max_sf = lora_mod_params.sf;

    for (uint8_t sf = 5; sf <= 12; sf++) {
        uint32_t test_toa;
        mod_test.sf = sf;
        mod_test.ppm = lr20xx_radio_lora_get_recommended_ppm_offset(sf, mod_test.bw);
        test_toa = lr20xx_radio_lora_get_time_on_air_in_ms(&pkt, &mod_test);

        if (test_toa <= codec2_production_time) {
            printf("SF%u: %lu ms - OK (margin %lu ms)\r\n",
                   sf, test_toa, codec2_production_time - test_toa);
            max_sf = sf;
        } else {
            printf("SF%u: %lu ms - TOO SLOW (exceeds by %lu ms)\r\n",
                   sf, test_toa, test_toa - codec2_production_time);
        }
    }

    printf("\r\nMaximum usable SF: %u (current: %u)\r\n", max_sf, lora_mod_params.sf);
    if (max_sf > lora_mod_params.sf) {
        printf("*** SF%u could be used for better range! ***\r\n", max_sf);
    }

    /* Calculate optimal packet size for SF12 */
    printf("\r\n--- SF12 Optimal Packet Size ---\r\n");
    mod_test.sf = 12;
    mod_test.ppm = lr20xx_radio_lora_get_recommended_ppm_offset(12, mod_test.bw);

    uint8_t max_bytes_sf12 = 0;
    uint8_t max_frames_sf12 = 0;

    /* Search from largest to smallest to find max packet size for SF12 */
    for (uint8_t num_frames = 255 / _bytes_per_frame; num_frames >= 1; num_frames--) {
        uint8_t test_bytes = num_frames * _bytes_per_frame;
        uint32_t test_production = num_frames * codec2_frame_period_ms;
        uint32_t test_toa;

        pkt.pld_len_in_bytes = test_bytes;
        test_toa = lr20xx_radio_lora_get_time_on_air_in_ms(&pkt, &mod_test);

        if (test_toa <= test_production) {
            max_bytes_sf12 = test_bytes;
            max_frames_sf12 = num_frames;
            printf("SF12 max packet: %u bytes (%u frames)\r\n", max_bytes_sf12, max_frames_sf12);
            printf("  TOA: %lu ms, production: %lu ms, margin: %lu ms\r\n",
                   test_toa, test_production, test_production - test_toa);
            break;
        }
    }

    if (max_bytes_sf12 == 0) {
        printf("SF12: No viable packet size (even 1 frame is too slow)\r\n");
    } else if (max_bytes_sf12 < lora_payload_length) {
        printf("To use SF12: reduce lora_payload_length from %u to %u\r\n",
               lora_payload_length, max_bytes_sf12);
    }

    printf("=================================\r\n\r\n");
}


