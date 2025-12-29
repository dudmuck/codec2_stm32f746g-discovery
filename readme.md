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
Used for testing codec2 over sx1272/sx1276/sx1261/sx1262.   LoRa shield is auto-detected on startup, allowing different radio device to be plugged in and quickly compared.  Upon startup, touchscreen awaits  user selection of codec2 mode (bitrate). To change codec2 rate, reset is required using black button.

During operation, the touchscreen permits adjustment of LoRa bandwidth, LoRa spreading factor and microphone gain.  Bottom of LCD shows radio mode (tx or rx) along with TX duty cycle or RX RSSI/SNR.  LoRa payload length is also adjustable, but both sides of link must be adjusted to the same.

During RX mode, upon reception of first packet, short tone is heard, indicating by sound the RSSI+SNR as tone frequency.  At end of last packet received, a short false squelch tail is added to indicate end of transmission.

#### lora_transceiver configuration
in radio.c:
* ``LORA_BW_KHZ``: default startup LoRa bandwith
* ``TX_DBM``: transmit power
* ``CF_HZ``: radio operating frequency

``TX_DBM`` and ``CF_HZ`` could be adjustable by LCD-touchscreen in future, if desired?

variables in main.c:
* ``lora_payload_length``: length of LoRa packet, needs to hold even number of codec2 frames.
* ``sf_at_500KHz``: spreading factor is decremented when bandwith reduced to keep same data-rate, but SF is manually adjusted by LCD-touchscreen.


## LoRa operation with codec2

### Codec2 mode parameters

| codec2 mode | samples/frame | frame period (ms) | encoded bytes/frame | frames/sec |
|-------------|---------------|-------------------|---------------------|------------|
| 3200 | 160 | 20 | 8 | 50 |
| 2400 | 160 | 20 | 6 | 50 |
| 1600 | 320 | 40 | 8 | 25 |
| 1400 | 320 | 40 | 7 | 25 |
| 1300 | 320 | 40 | 6.5 (13 bytes/2 frames) | 25 |
| 1200 | 320 | 40 | 6 | 25 |
| 700C | 320 | 40 | 3.5 (7 bytes/2 frames) | 25 |

### Default LoRa settings (LR20xx, 500kHz BW)

| codec2 mode | default SF | default payload (bytes) | frames/packet | production time (ms) | approx TOA (ms) | max streaming SF |
|-------------|------------|-------------------------|---------------|----------------------|-----------------|------------------|
| 3200 | 9 | 128 | 16 | 320 | 170 | 10 |
| 2400 | 10 | 96 | 16 | 320 | 280 | 10 |
| 1600 | 11 | 128 | 16 | 640 | 554 | 11 |
| 1400 | 11 | 112 | 16 | 640 | 504 | 11 |
| 1300 | 11 | 52 | 8 | 320 | 277 | 11 |
| 1200 | 11 | 96 | 16 | 640 | 455 | 11 |
| 700C | 12 | 56 | 16 | 640 | 530 | 12 |

**Notes:**
- Production time = frames/packet × frame period. Streaming requires TOA ≤ production time.
- Max streaming SF shows the highest SF where streaming is feasible at 500kHz BW.
- 1300 and 700C modes use dual-frame encoding (2 codec2 frames packed together).
- TOA values are approximate and depend on preamble length and coding rate.

### General LoRa considerations

LoRa data rate selection is only possible in steps by a factor of two.  Codec2 bit-rate change will affect LoRa packet duty cycle.  When duty cycle is under 50%, the LoRa data-rate can be reduced (bandwidth reduced or SF increased).  If packet duty cycle is over 100%, then LoRa data-rate must be increased to faster.  Typical 2.5 to 2.7dB change in link budget for each step of LoRa data-rate.

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

### LR20xx Streaming TX

When using the LR2021 radio (build with `-DRADIO=LR2021`), the firmware supports streaming TX via the radio's FIFO threshold interrupt. This enables continuous audio streaming without gaps between packets when the radio data rate is sufficient.

#### How it works

On startup, the firmware analyzes timing feasibility:
- Calculates packet time-on-air vs codec2 frame production time
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

The test script `test_radio_link.sh` automates testing across all codec2 rates.


