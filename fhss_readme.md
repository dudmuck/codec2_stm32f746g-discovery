# FHSS (Frequency Hopping Spread Spectrum) for Codec2 Voice

## Overview

This implementation provides FCC Part 15.247 compliant frequency hopping for codec2 voice transmission over LoRa in the 902-928 MHz ISM band.

### FCC Requirements
- 50 channels minimum
- 400ms maximum dwell time per channel
- Pseudo-random hopping pattern

## Codec2 Rate Summary

| Mode | Frames/Pkt | Bytes/Pkt | SF | TOA (ms) | Prod Time (ms) | Hop Count | Audio/Pkt (ms) |
|------|------------|-----------|-----|----------|----------------|-----------|----------------|
| 3200 | 6 | 48 | 7 | 93 | 120 | 3 | 240 |
| 2400 | 6 | 36 | 7* | 78 | 120 | 3 | 240 |
| 1600 | 9 | 72 | 8* | 226 | 360 | 1 | 360 |
| 1400 | 7 | 49 | 8* | 175 | 280 | 2 | 280 |
| 1300 | 2 | 26 | 7 | 62 | 80 | 3 | 80 |
| 1200 | 6 | 36 | 8* | 134 | 240 | 2 | 240 |
| 700C | 7 | 49 | 8* | 175 | 280 | 2 | 280 |

*SF adjusted down automatically to ensure TOA < Production Time

### Column Definitions

- **Frames/Pkt**: Number of codec2 frames per LoRa packet
- **Bytes/Pkt**: Payload size (excluding 3-byte FHSS header)
- **SF**: Spreading Factor used (after auto-adjustment)
- **TOA**: Time-on-air for one packet
- **Prod Time**: Time to produce one packet's worth of audio (Frames × 40ms)
- **Hop Count**: Packets sent per channel before hopping (proactive_hop_count)
- **Audio/Pkt**: Audio duration contained in one packet

## Reliability by Mode

### Most Reliable (Hop Count = 1)
- **1600**: Hops after every packet. RX only needs to catch 1 packet per channel.

### Good Reliability (Hop Count = 3)
- **3200, 2400, 1300**: RX has 3 chances to catch packets on each channel.

### Marginal Reliability (Hop Count = 2)
- **700C, 1400, 1200**: Only 2 packets per channel. If RX misses one, synchronization becomes difficult.

## Timing Constraints

The FCC 400ms dwell limit combined with packet TOA determines how many packets can be sent per channel:

```
Packets per channel = floor(340ms / Production_Time) + 1
```

Where 340ms = 85% of 400ms (defer threshold to ensure hop completes before limit).

### Why 700C/1400/1200 Are Challenging

With only 2 packets per ~340ms dwell:
- TX sends packet 0, packet 1, then hops
- If RX misses packet 0 (due to sync delay), it only has 1 chance left
- If RX processing takes >175ms, it may miss packet 1 entirely
- Once out of sync, RX must return to CAD scanning

## Channel Plan

- **Band**: 902-928 MHz (US ISM)
- **Channels**: 50
- **Base Frequency**: 902.2 MHz
- **Channel Spacing**: 520 kHz
- **Bandwidth**: 125 kHz

## Synchronization

1. **TX Sync Phase**: Sends 3 sync packets on random channels with long preamble (171 symbols, ~350ms)
2. **RX Scan**: CAD sweeps all 50 channels looking for preamble
3. **Sync Packet**: Contains LFSR state and next data channel
4. **Data Phase**: Both sides hop using synchronized LFSR

## Known Issues

### Image Frequency Reception
RX sometimes detects sync packets on image frequencies (wrong channel but correct data). This can cause channel misalignment. Mitigation: SNR-based filtering ignores weak signals (SNR < 0 dB).

### Late Sync
If RX syncs late in the sync phase, TX may have already hopped away from the first data channel. RX will timeout and attempt to catch up by hopping.

## Test Results

Typical results with 15-second test duration:

| Mode | Status | Notes |
|------|--------|-------|
| 3200 | PASS | High reliability |
| 2400 | PASS | High reliability |
| 1600 | PASS | Most reliable (hop every packet) |
| 1400 | PASS/WARN | May have occasional CRC errors |
| 1300 | PASS | High reliability |
| 1200 | PASS/WARN | May have occasional sync issues |
| 700C | WARN | Most challenging, occasional packet loss |

## Configuration

FHSS is enabled by default. Key parameters in `fhss.h`:

```c
#define FHSS_NUM_CHANNELS       50
#define FHSS_MAX_DWELL_MS       400
#define FHSS_RX_TIMEOUT_MAX     3    // Timeouts before rescan
#define FHSS_SYNC_REPEAT_COUNT  3    // Sync packets sent
```

## Files

- `lr20xx_hal/fhss.c` - FHSS state machine and hopping logic
- `lr20xx_hal/fhss.h` - Configuration and API
- `test_fhss.sh` - Single rate test script
- `test_fhss_rates.sh` - All rates test script
