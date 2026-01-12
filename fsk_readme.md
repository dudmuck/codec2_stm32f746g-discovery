# FSK Streaming Voice Mode

This document describes the FSK streaming configuration for real-time voice transmission using the Opus vocoder over LR2021 radio.

## Overview

The system transmits Opus-encoded voice frames over FSK modulation. Different Opus bitrate modes result in different frame sizes, which determine the required FSK bitrate for real-time streaming.

### Architecture

- **Encoder**: Opus vocoder running on STM32F746G-Discovery
- **Radio**: Semtech LR2021 with FSK modulation
- **Frame period**: 40ms (25 fps) for most modes, 62ms (16 fps) for 64K
- **Packet size**: 240 bytes maximum (hardware FIFO constraint)

### Multi-Packet Modes

For lower bitrate modes where Opus frames are small, multiple frames are batched into a single 240-byte packet to improve efficiency. For 64K mode where frames exceed 240 bytes, each frame spans 2 packets.

## Opus Mode Timing Table

| Mode | Opus bps | Bytes/Frame | Frames/Pkt | FPS | Frame Period | FSK kbps | Packet TOA | Margin | RX Sensitivity |
|------|----------|-------------|------------|-----|--------------|----------|------------|--------|----------------|
| 6K   | 6,000    | 30          | 8          | 25  | 40 ms        | 13       | 156 ms     | 164 ms | ~-120 dBm      |
| 8K   | 8,000    | 40          | 6          | 25  | 40 ms        | 16       | 126 ms     | 114 ms | ~-119 dBm      |
| 12K  | 12,000   | 60          | 4          | 25  | 40 ms        | 22       | 92 ms      | 68 ms  | ~-117 dBm      |
| 16K  | 16,000   | 80          | 3          | 25  | 40 ms        | 28       | 72 ms      | 48 ms  | ~-115 dBm      |
| 24K  | 24,000   | 120         | 2          | 25  | 40 ms        | 40       | 51 ms      | 29 ms  | ~-113 dBm      |
| 32K  | 32,000   | 160         | 1          | 25  | 40 ms        | 52       | 27 ms      | 13 ms  | ~-115 dBm      |
| 48K  | 48,000   | 240         | 1          | 25  | 40 ms        | 76       | 27 ms      | 13 ms  | ~-112 dBm      |
| 64K  | 64,000   | 480         | 1 (2 pkt)  | 16  | 62 ms        | 95       | 44 ms      | 18 ms  | ~-111 dBm      |

### Column Descriptions

- **Opus bps**: Target Opus encoder bitrate
- **Bytes/Frame**: Opus encoded frame size in bytes
- **Frames/Pkt**: Number of Opus frames per radio packet (or packets per frame for 64K)
- **FPS**: Frames per second (audio frame rate)
- **Frame Period**: Time to produce one Opus frame of audio
- **FSK kbps**: Calculated FSK bitrate = (payload + overhead) * fps * 1.5
- **Packet TOA**: Time on air for radio packet transmission
- **Margin**: Production time - TX time (buffer headroom)
- **RX Sensitivity**: Estimated receiver sensitivity (interpolated from datasheet)

## Measured Performance

From test logs with concurrent FreeRTOS encode/TX:

| Mode | enc_max (ms) | cycle_max (ms) | Overruns | Status |
|------|--------------|----------------|----------|--------|
| 6K   | 14           | 40             | 0        | OK     |
| 8K   | 15           | 41             | 0        | OK     |
| 12K  | 15           | 40             | 0        | OK     |
| 16K  | 24           | 43             | 2        | TX_SLOW |
| 24K  | 26           | 44             | 3        | TX_SLOW |
| 32K  | 21           | 43             | 19       | TX_SLOW |
| 48K  | 21           | 42             | 0        | OK     |
| 64K  | 31           | 57             | 0        | OK     |

- **enc_max**: Maximum Opus encode time observed
- **cycle_max**: Maximum total cycle time (encode + TX + overhead)
- **Overruns**: Number of times cycle exceeded frame period
- **TX_SLOW**: Warning when cycle_max > frame_period (audio still OK due to buffering)

## LR2021 FSK Sensitivity Reference

From LR2021 V1.1 Datasheet, Table 3-14 (FSK Sensitivity @0.1% BER):

| Bitrate   | Fdev   | RX BW   | Sensitivity |
|-----------|--------|---------|-------------|
| 250 kbps  | 100kHz | 555 kHz | -106 dBm    |
| 100 kbps  | 50 kHz | 333 kHz | -110.5 dBm  |
| 50 kbps   | 25 kHz | 138 kHz | -113 dBm    |
| 50 kbps   | 12.5kHz| 138 kHz | -116 dBm    |
| 38.4 kbps | 40 kHz | 185 kHz | -113 dBm    |
| 4.8 kbps  | 5 kHz  | 27 kHz  | -121.5 dBm  |
| 1.2 kbps  | 5 kHz  | 17 kHz  | -124.5 dBm  |

Note: Sensitivity measured with rx_boost=7 (boosted gain mode).

## FSK Configuration

The FSK bitrate is calculated dynamically based on Opus frame size:

```
FSK_bitrate = (payload_bits + overhead_bits) * fps * 1.5
```

Where:
- `payload_bits` = bytes_per_frame * 8
- `overhead_bits` = 96 (40 preamble + 32 sync + 8 length + 16 CRC)
- `fps` = frames per second
- `1.5` = 50% margin for timing tolerance

Frequency deviation is set to `bitrate / 2` (modulation index beta = 1.0).

## Notes

1. **64K mode** uses reduced Opus complexity (2 instead of 5) to meet timing constraints
2. **TX_SLOW warnings** in 24K/32K modes don't impact audio quality due to double-buffering
3. **Multi-packet batching** adds latency but improves spectral efficiency for low bitrates
4. **RX FIFO** requires high-priority radio service task to prevent overflow during Opus decode
