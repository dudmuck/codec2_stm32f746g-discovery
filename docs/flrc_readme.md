# FLRC Mode for High-Bitrate Opus Streaming

FLRC (Fast Long Range Communication) is used for 64K and 96K Opus modes where LoRa modulation cannot provide sufficient data rate. Lower bitrate modes (6K-48K) use LoRa modulation.

## Supported Modes

| Mode | Opus Bitrate | Frame Size | Packets/Frame | FLRC Bitrate | TX Time | Margin | enc_max |
|------|--------------|------------|---------------|--------------|---------|--------|---------|
| 64K  | 64 kbps      | 480 bytes  | 2 × 240 bytes | 1040 kbps    | 4 ms    | 58 ms  | 29 ms   |
| 96K  | 96 kbps      | 720 bytes  | 3 × 240 bytes | 1040 kbps    | 6 ms    | 56 ms  | 17 ms   |

- **Frame period**: 62 ms (60 ms Opus frame + overhead)
- **Packet payload**: 240 bytes (fits in 256-byte hardware FIFO)
- **Packet TOA**: ~2 ms per packet at 1040 kbps

## Encoder Timing

Both modes use Opus complexity 5 with FreeRTOS concurrent encoding:

| Mode | enc_max | Notes |
|------|---------|-------|
| 64K  | 29 ms   | Lower bitrate requires more CPU work to compress |
| 96K  | 17 ms   | Higher bitrate = simpler encoding decisions |

The counterintuitive result (higher bitrate encodes faster) is normal for VBR codecs - Opus works harder to fit audio into fewer bits at lower bitrates.

## Available FLRC Bitrates

| Bitrate (kbps) | Bandwidth (kHz) | Sensitivity (dBm) | Used By |
|----------------|-----------------|-------------------|---------|
| 260            | 307             | -111              | -       |
| 325            | 357             | ~-110             | -       |
| 520            | 571             | ~-108             | -       |
| 650            | 740             | ~-107             | -       |
| 1040           | 1333            | **-104.5**        | 64K, 96K |
| 1300           | 1333            | ~-103             | -       |
| 2080           | 2222            | ~-102             | -       |
| 2600           | 2666            | -101.5            | -       |

*Sensitivity values from LR2021 V1.1 datasheet (1% PER, 915 MHz, sub-GHz). Values marked ~ are interpolated.*

Both 64K and 96K use 1040 kbps - the 50% increase in data (720 vs 480 bytes) is handled by using 50% more packets (3 vs 2) per frame.

## Multi-Packet Mode

Since Opus frames exceed the 240-byte LoRa packet limit:
- **64K**: 480 bytes/frame = 2 packets × 240 bytes
- **96K**: 720 bytes/frame = 3 packets × 240 bytes

Packets are transmitted back-to-back within the 62 ms frame period. The receiver reassembles packets using sequence numbers embedded in each Opus frame.

## LoRa vs FLRC Comparison

64K mode can use either LoRa (SF6/1000kHz) or FLRC (1040 kbps). From LR2021 V1.1 datasheet:

| Modulation | Setting | Sensitivity | Data Rate |
|------------|---------|-------------|-----------|
| LoRa | SF6 @ 1000kHz | -116 dBm | ~37 kbps |
| LoRa | SF5 @ 1000kHz | -113.5 dBm | ~62 kbps |
| FLRC | 1040 kbps | -104.5 dBm | 1040 kbps |

**Sensitivity difference: ~11.5 dB** (LoRa SF6 vs FLRC)

This translates to roughly **3-4× range advantage for LoRa** at the cost of tighter timing margins. FLRC provides 56-58ms margin vs ~6ms for LoRa, making FLRC more robust for streaming.

### When to use each:
- **LoRa SF6/1000kHz**: Maximum range for 64K, but tight timing margins
- **FLRC 1040kbps**: Relaxed timing (56ms margin), easier multi-packet handling, slightly reduced range
