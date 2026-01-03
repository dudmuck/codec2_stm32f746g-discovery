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

FCC Part 15.247 limits **transmit time** to 400ms per channel, not wall-clock time. The device can stay on a channel indefinitely; only actual TX counts toward the limit.

```
Packets per channel = floor(400ms / Packet_TOA)
```

For 700C/1400 with 175ms TOA: floor(400/175) = 2 packets per channel.

### Proactive Hopping

TX and RX use synchronized LFSR-based pseudo-random hopping:
- TX hops after sending `proactive_hop_count` packets
- RX tracks packet count and proactively hops to match TX
- Both use the same LFSR sequence, so they stay synchronized

## Channel Plan

- **Band**: 902-928 MHz (US ISM)
- **Channels**: 50
- **Base Frequency**: 902.2 MHz
- **Channel Spacing**: 520 kHz
- **Bandwidth**: 125 kHz

## Synchronization

### Basic Mode (default)
1. **TX Sync Phase**: Sends 3 sync packets on different channels with long preamble (171 symbols, ~350ms)
2. **RX Scan**: CAD sweeps all 50 channels looking for preamble
3. **Sync Packet**: Contains LFSR state, next data channel, and hop count
4. **Data Phase**: Both sides hop using synchronized LFSR

### ACK Mode (default)
Enabled by default. Use 'a' command to disable if needed.

1. **TX Sync**: Sends sync packet with long preamble, then auto-switches to RX to wait for ACK
2. **RX Scan**: CAD sweeps channels, detects sync, sends ACK back to TX
3. **ACK Timeout**: If TX doesn't receive ACK within 200ms, retries on a different channel (up to 10 attempts)
4. **Data Phase**: Once ACK received, both sides transition to synchronized data mode

ACK mode provides reliable sync establishment even with image frequency reception or packet loss. The retry mechanism uses deterministic channel selection to avoid LFSR desynchronization.

## Known Issues

### Image Frequency Reception
RX sometimes detects sync packets on image frequencies (wrong channel but correct data). The sync packet now includes `sync_channel` so RX knows where TX is actually transmitting. Mitigation: SNR-based filtering ignores weak signals (SNR < 0 dB).

### Late Sync
If RX syncs late in the sync phase, TX may have already hopped away from the first data channel. RX will timeout and attempt to catch up by hopping. ACK mode eliminates this issue by ensuring TX waits for RX confirmation.

### LFSR Synchronization (Fixed)
Previously, ACK timeout retries called `fhss_random_channel()` which advanced the LFSR, causing TX and RX to hop to different channels. Fixed by using deterministic channel selection for retries: `(tx_next_channel + sync_attempts * 17) % 50`.

## Test Results

### With ACK Mode (15-second test, recommended)

| Mode | TX Pkts | RX Pkts | Lost | CRC Err | Status |
|------|---------|---------|------|---------|--------|
| 3200 | 96 | 97 | 0 | 0 | PASS |
| 2400 | 118 | 120 | 0 | 1 | PASS |
| 1600 | 38 | 39 | 0 | 0 | PASS |
| 1400 | 52 | 53 | 0 | 0 | PASS |
| 1300 | 72 | 73 | 0 | 0 | PASS |
| 1200 | 61 | 62 | 0 | 0 | PASS |
| 700C | 21 | 23 | 0 | 1 | PASS |

*RX count includes ACK packet. All modes achieve 0 lost packets with ACK mode enabled.*

### Without ACK Mode

| Mode | Status | Notes |
|------|--------|-------|
| 3200 | PASS | High reliability |
| 2400 | PASS | High reliability |
| 1600 | PASS | Most reliable (hop every packet) |
| 1400 | PASS/WARN | May have occasional sync issues |
| 1300 | PASS | High reliability |
| 1200 | PASS/WARN | May have occasional sync issues |
| 700C | WARN | Sync timing sensitive, use ACK mode |

## Configuration

FHSS is enabled by default. Key parameters in `fhss.h`:

```c
#define FHSS_NUM_CHANNELS       50
#define FHSS_MAX_DWELL_MS       400
#define FHSS_RX_TIMEOUT_MAX     3    // Timeouts before rescan
#define FHSS_SYNC_REPEAT_COUNT  3    // Sync packets sent (basic mode)
#define FHSS_SYNC_MAX_ATTEMPTS  10   // Max sync retries (ACK mode)
#define FHSS_ACK_TIMEOUT_MS     200  // ACK wait timeout
```

### UART Commands

| Command | Description |
|---------|-------------|
| `A` | Enable ACK mode |
| `a` | Disable ACK mode |
| `H` | Enable FHSS and start CAD scan |
| `h` | Disable FHSS |
| `t` | Start TX (PTT) |
| `r` | Stop TX |
| `T` | Enable test mode (tone output) |

## Files

- `lr20xx_hal/fhss.c` - FHSS state machine and hopping logic
- `lr20xx_hal/fhss.h` - Configuration and API
- `test_fhss.sh` - Single rate test script (usage: `./test_fhss.sh [duration] [rate] [delay_ms] [ack]`)
- `test_fhss_rates.sh` - All rates test script (usage: `./test_fhss_rates.sh [duration] [delay_ms] [ack]`)
