## contains 3 projects

* **passthru**: for checking [stm32f746g-discovery](https://www.st.com/en/evaluation-tools/32f746gdiscovery.html) hardware: LCD-touchscreen, microphone and speaker interface. Requires only one discovery board. Useful for anybody wishing to use this board for audio purpose.
* **vocoder_passthru**: adds vocoder to passthru test using single discovery board: For checking codec2 from microphone encoding directly decoding out to speaker.  This doubles CPU power required since both encoding and decoduing must be done for each frame.
* **lora_transceiver**: adds LoRa radio [sx127x](https://os.mbed.com/components/SX1276MB1xAS) or [sx126x](https://os.mbed.com/components/SX126xMB2xAS), half duplex transceiver.  Requires two discovery boards, each with its own LoRa arduino shield.  PTT is blue button (push to talk)

Each microphone on the board can be individually disabled by touch screen selection.  Used for comparing performance between microphone in case one becomes damaged.  This discovery board has two microphones, but only one needed for vocoder.   Microphone can be permanently disabled, instead of touch-screen selection, by defining ``MIC_DISABLE`` as shown in ``CMakeLists.txt``

## QuickStart
After cloning this repository:
```
$ git submodule init
$ git submodule update --init --recursive
```
Reminder that this retrieves STM32CubeF7, and its 1.1 Gbytes. Follow GCC install instructions here: [steps 2 and 4 for toolchain](https://github.com/drowe67/codec2/tree/main/stm32).
```
(from root directory of this project)
$ mkdir build
$ cd build
$ cmake ..   (for LR2021 add -DRADIO=LR2021)
$ make
```
Ensure gcc-arm is in your path prior to running cmake, if not delete your build directory and re-run cmake with gcc-arm in path.

### Flashing stm32f7-discovery
~~https://github.com/texane/stlink
providing address isnt needed when using st-flash with .hex files
i.e: 
``st-flash --format ihex write <project>.hex``~~ the bin files in build directory can be copied directly to the stlink usb drive on the discovery board.  passthru.bin or lora_transceiver.bin or vocoder_passthru.bin

## detailed operation of each project
### passthru
Used for evaluating audio on stm32f746g-discovery.  Upon startup, LCD screen awaits user selection of audio sample rate, permitted comparison of performance and various audio sampling rates.  This is needed to check board audio quality by itself without vocoder.

### vocoder_passthru
Used for evaluating codec2 performace on stm32f746g-discovery with simultaneous encoding -> decoding.   Upon startup, LCD screen awaits  user selection of codec2 mode (bitrate), along with audio hardware sampling rate.   Hardware sample rate greater than 8ksps is decimated-interpolated to 8ksps for codec2.

If needed to select different hardware audio rate, it must be selected prior to codec2 mode selection.   Codec2 mode selection on touchscreen starts codec2.

Latency of playback is adjusted on LCD-touchscreen.  Long latency here is required to properly hear speech quality of vocoder.
### lora_transceiver
Used for testing Opus vocoder over sx1272/sx1276/sx1261/sx1262/LR2021.   LoRa shield is auto-detected on startup, allowing different radio device to be plugged in and quickly compared.  Upon startup, touchscreen awaits user selection of Opus mode (bitrate). To change Opus rate, reset is required using black button.

During operation, the touchscreen permits adjustment of LoRa bandwidth, LoRa spreading factor and microphone gain.  Bottom of LCD shows radio mode (tx or rx) along with TX duty cycle or RX RSSI/SNR.  LoRa payload length is also adjustable, but both sides of link must be adjusted to the same.

During RX mode, upon reception of first packet, short tone is heard, indicating by sound the RSSI+SNR as tone frequency.  At end of last packet received, a short false squelch tail is added to indicate end of transmission.

#### lora_transceiver configuration
in radio.c:
* ``LORA_BW_KHZ``: default startup LoRa bandwith
* ``TX_DBM``: transmit power
* ``CF_HZ``: radio operating frequency

``TX_DBM`` and ``CF_HZ`` could be adjustable by LCD-touchscreen in future, if desired?

variables in main.c:
* ``lora_payload_length``: length of LoRa packet, needs to hold integer number of Opus frames.
* ``sf_at_500KHz``: spreading factor is decremented when bandwith reduced to keep same data-rate, but SF is manually adjusted by LCD-touchscreen.


## LoRa operation with Opus

### Opus mode parameters

| Opus mode | samples/frame | frame period (ms) | encoded bytes/frame | frames/sec |
|-----------|---------------|-------------------|---------------------|------------|
| 6K  | 320 | 40 | 30  | 25 |
| 8K  | 320 | 40 | 40  | 25 |
| 12K | 320 | 40 | 60  | 25 |
| 16K | 320 | 40 | 80  | 25 |
| 24K | 320 | 40 | 120 | 25 |
| 32K | 320 | 40 | 160 | 25 |
| 48K | 320 | 40 | 240 | 25 |
| 64K | 2880 | 60 | 480 | 16.7 |
| 96K | 2880 | 60 | 720 | 16.7 |

**Notes:**
- Modes 6K-48K: Audio sample rate 8000 Hz, frame period 40ms.
- Modes 64K-96K: Audio sample rate 48000 Hz, frame period 60ms (high-fidelity mode).

### Default LoRa settings (LR20xx)

With FreeRTOS concurrent encoding, the encoder runs in parallel with radio transmission, allowing higher SF values than sequential operation.

| Opus mode | BW (kHz) | auto SF | payload (bytes) | frames/pkt | production (ms) | TOA (ms) | margin (ms) |
|-----------|----------|---------|-----------------|------------|-----------------|----------|-------------|
| 6K  | 500  | 9 | 240 | 8 | 320 | 298 | 22 |
| 8K  | 500  | 8 | 240 | 6 | 240 | 164 | 76 |
| 12K | 500  | 7 | 240 | 4 | 160 | 94  | 66 |
| 16K | 500  | 7 | 240 | 3 | 120 | 94  | 26 |
| 24K | 500  | 6 | 240 | 2 | 80  | 55  | 25 |
| 32K | 500  | 5 | 160 | 1 | 40  | 22  | 18 |
| 48K | 812  | 6 | 240 | 1 | 40  | 34  | 6 |
| 64K | 1000 | 6 | 240×2 | 1 | 62  | 56  | 6 |
| 96K | 1000 | 5 | 240×3 | 1 | 60  | 51  | ~0* |

**Notes:**
- **Auto SF** = SF automatically adjusted based on concurrent vs sequential mode
- **BW** = Bandwidth. Higher rates use wider bandwidth for faster transmission.
- **TOA** = Time On Air (packet transmission duration at auto SF)
- **Production** = frames/pkt × frame period. Streaming requires TOA ≤ production.
- **Margin** = production - TOA. Minimum 5ms required with FreeRTOS concurrent encoding.
- 64K and 96K use **multi-packet mode**: each Opus frame spans 2-3 LoRa packets (see below).
- *96K margin is very tight; may require complexity reduction.

### General LoRa considerations

LoRa data rate selection is only possible in steps by a factor of two.  Opus bit-rate change will affect LoRa packet duty cycle.  When duty cycle is under 50%, the LoRa data-rate can be reduced (bandwidth reduced or SF increased).  If packet duty cycle is over 100%, then LoRa data-rate must be increased to faster.  Typical 2.5 to 2.7dB change in link budget for each step of LoRa data-rate.

Latency across the radio link is due to LoRa packet duration.

### Receiver sensitivity comparison (500kHz BW)

| SF | SX126x (dBm) | LR2021 Sub-GHz (dBm) | LR2021 2.4GHz (dBm) | Improvement vs SX126x |
|----|--------------|----------------------|---------------------|----------------------|
| 7  | -117         | -122                 | -119                | +5 dB |
| 8  | -119*        | -124.5               | -122                | +5.5 dB |
| 9  | -122*        | -127.5               | -124.5              | +5.5 dB |
| 10 | -124*        | -130                 | -127.5              | +6 dB |
| 11 | -127*        | -133                 | -130                | +6 dB |
| 12 | -129         | -135.5               | -133                | +6.5 dB |

*SX126x values for SF8-11 are interpolated (datasheet only specifies SF7 and SF12 at 500kHz).

**Sources:**
- SX126x: DS_SX1261-2_V2.2 Table 3-8
- LR2021: LR2021_V1.1_datasheet Tables 3-9 (Sub-GHz) and 3-11 (2.4GHz)

The LR2021 provides 5-6.5 dB better receiver sensitivity compared to SX126x at 500kHz bandwidth, translating to roughly 2x improved range or equivalent performance at lower transmit power.

### Bandwidth and SF Adjustment

The LoRa bandwidth can be adjusted at runtime using serial commands. SF is automatically adjusted to maintain streaming feasibility with a minimum 10ms margin.

**Serial Commands:**
- `b` - Step bandwidth down (500→406→250→203→125→101→83→62→41→31 kHz)
- `B` - Step bandwidth up (reverse direction)
- `f` - Step SF down (faster, shorter range)
- `F` - Step SF up (slower, longer range)
- `c` - Decrease Opus complexity (faster encode)
- `C` - Increase Opus complexity (better quality)

**Note:** Both TX and RX must use the same bandwidth and SF settings. When testing, step both boards together.

### LR20xx Streaming Configuration at Different Bandwidths

With FreeRTOS concurrent encoding, the firmware auto-adjusts SF to ensure streaming is feasible with sufficient timing margin (minimum 5ms). Concurrent mode allows higher SF than sequential because encoding overlaps with TX.

| Opus mode | Default BW | Auto SF (concurrent) | Max SF (sequential) |
|-----------|------------|----------------------|---------------------|
| 6K  | 500kHz  | SF9 | SF7 |
| 8K  | 500kHz  | SF8 | SF6 |
| 12K | 500kHz  | SF7 | SF6 |
| 16K | 500kHz  | SF7 | SF5 |
| 24K | 500kHz  | SF6 | SF5 |
| 32K | 500kHz  | SF6* | SF5 |
| 48K | 812kHz  | SF6 | SF5 |
| 64K | 1000kHz | SF6 | SF5 |
| 96K | 1000kHz | SF5 | - |

*32K at SF6 has only 2ms margin; SF5 (18ms margin) is more reliable.

**Link Budget Trade-offs:**
- Higher BW + higher SF = lower latency, similar range
- Lower BW + lower SF = higher latency, potentially better range (BW dominates)
- Each SF step: ~2.5 dB sensitivity change
- Each BW halving: ~3 dB sensitivity improvement

For maximum range, use lower bandwidth. For lower latency, use higher bandwidth.

### LR20xx Streaming TX

When using the LR2021 radio (build with `-DRADIO=LR2021`), the firmware supports streaming TX via the radio's FIFO threshold interrupt. This enables continuous audio streaming without gaps between packets when the radio data rate is sufficient.

#### How it works

On startup, the firmware analyzes timing feasibility and automatically adjusts SF if the margin is less than 10ms:
- Calculates packet time-on-air vs Opus frame production time
- Determines if streaming is feasible at current SF/BW settings
- Recommends optimal SF if current settings are too slow
- Calculates minimum initial frames needed before starting TX

The streaming TX state machine:
1. **ENCODING**: Buffering initial frames before TX starts
2. **TX_ACTIVE**: TX in progress, continuing to encode frames
3. **WAIT_TX**: All frames encoded, waiting for FIFO to drain
4. **BUFFERING_NEXT**: Encoding next packet while current TX completes

When TX is faster than encoding (e.g., 3200 mode at SF9), the firmware buffers a full packet before each transmission. When TX is slower than encoding, true streaming is possible with minimal initial buffering.

#### Test Mode

Press 'T' via serial to enable test mode, which sends sequence numbers instead of encoded audio. This allows verification of streaming integrity:
- 't': Start TX
- 'r': Stop TX
- '?': Show stats (frames received, dropped, duplicates, gaps)

The test script `test_radio_link.sh` automates testing across all Opus rates.

### Multi-Packet Mode (64K/96K)

For high-fidelity 64K and 96K modes, each Opus frame (480 or 720 bytes) exceeds the maximum LoRa payload size (255 bytes). These modes use **multi-packet operation** where each Opus frame is split across multiple LoRa packets.

#### Configuration

| Mode | Opus frame | Packets per frame | Packet size | Total TX time |
|------|------------|-------------------|-------------|---------------|
| 64K  | 480 bytes  | 2                 | 240 bytes   | 56 ms (SF6)   |
| 96K  | 720 bytes  | 3                 | 240 bytes   | 51 ms (SF5)   |

#### How Multi-Packet TX Works

1. **Encoder produces 480-byte frame** (60ms of 48kHz audio)
2. **First packet sent** (240 bytes) via streaming FIFO
3. **Second packet sent immediately** after first completes
4. TX done callback triggers next packet until full frame transmitted
5. State machine transitions to buffer next Opus frame

The TX state machine tracks `tx_bytes_sent` separately from FIFO writes to ensure all packets of a frame are transmitted before moving to the next frame.

#### How Multi-Packet RX Works

1. **FIFO threshold set to 64 bytes** for frequent draining (prevents overflow)
2. **First packet arrives** (240 bytes accumulated in buffer)
3. **Second packet arrives** (buffer now has 480 bytes)
4. **Decode triggered** when `rx_fifo_read_idx >= bytes_per_frame`
5. Buffer compacted via memmove, indices reset for next frame

Key implementation details:
- Hardware FIFO is only 256 bytes, so frequent draining via threshold IRQ is critical
- Back-to-back packets at SF5 (17ms TOA) require fast FIFO servicing
- Overflow recovery clears hardware FIFO and resets buffer indices

#### Timing Constraints

At SF6 with 1000kHz bandwidth (concurrent encoding):
- Packet time-on-air: 28 ms
- Two packets back-to-back: 56 ms total TX time
- Opus frame production: 62 ms
- **Margin: 6 ms** (sufficient with FreeRTOS concurrent encoding)

With FreeRTOS concurrent encoding, the encoder runs in a separate task overlapping with TX, allowing SF6 (vs SF5 in sequential mode). The auto-SF adjustment ensures at least 5ms margin in concurrent mode.

#### 64K Mode Timing Analysis

With FreeRTOS concurrent encoding, the encoder task runs in parallel with radio transmission, providing better timing margins than sequential operation.

**Measured timing with concurrent encoding (SF6 @ 1000kHz):**

| Component | Time |
|-----------|------|
| Opus encode (max) | ~30 ms |
| Packet 1 time-on-air | 28 ms |
| Packet 2 time-on-air | 28 ms |
| **Total TX time** | **56 ms** |
| **Frame period** | **62 ms** |
| **Margin** | **6 ms** |

Because encoding runs concurrently with transmission, the constraint is simply `TOA ≤ production_time` (56ms ≤ 62ms). This allows SF6 operation which was not feasible in sequential mode.

**Opus complexity tuning:**

To reduce encode time, multi-packet modes automatically reduce Opus complexity from the default of 5:

| Complexity | Encode time (audio) | Effect |
|------------|---------------------|--------|
| 5 (default)| ~35+ ms | Streaming fails |
| 3-4 | ~30 ms | Frequent TX_SLOW |
| 2 | ~25-31 ms | TX_SLOW during audio only |
| 0-1 | ~19-25 ms | Rare TX_SLOW |

The firmware defaults to complexity 2 for 64K/96K modes, providing a balance between audio quality and timing margin.

**Runtime complexity adjustment:**

Complexity can be adjusted via serial commands during operation:
- `c` - Decrease complexity (faster encoding, lower quality)
- `C` - Increase complexity (slower encoding, higher quality)

**TX_SLOW monitoring:**

During TX, the firmware monitors cycle time and reports:
- Real-time warnings: `TX_SLOW: cycle=65ms > frame=60ms`
- Summary at TX end: `TX end enc_max=31 cycle_max=72 overruns=8`

The `enc_max` shows peak encode time, `cycle_max` shows peak total cycle time, and `overruns` counts frames exceeding the 60ms period.

### FHSS (Frequency Hopping)

For FCC Part 15.247 compliant frequency hopping spread spectrum operation, see the `lr20xx_fhss` branch. This adds:
- 50-channel hopping in the 902-928 MHz ISM band
- LFSR-based pseudo-random hopping pattern
- ACK-based synchronization for reliable link establishment
- Proactive hop coordination between TX and RX

See [fhss_readme.md](https://github.com/dudmuck/codec2_stm32f746g-discovery/blob/lr20xx_fhss/fhss_readme.md) for documentation.

