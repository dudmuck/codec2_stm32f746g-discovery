## contains 3 projects

* **passthru**: for checking [stm32f746g-discovery](https://www.st.com/en/evaluation-tools/32f746gdiscovery.html) hardware: LCD-touchscreen, microphone and speaker interface. Requires only one discovery board. Useful for anybody wishing to use this board for audio purpose.
* **vocoder_passthru**: adds vocoder to passthru test using single discovery board: For checking codec2 from microphone encoding directly decoding out to speaker.  This doubles CPU power required since both encoding and decoduing must be done for each frame.
* **lora_transceiver**: adds LoRa radio [sx127x](https://os.mbed.com/components/SX1276MB1xAS) or [sx126x](https://os.mbed.com/components/SX126xMB2xAS), half duplex transceiver.  Requires two discovery boards, each with its own LoRa arduino shield.  PTT is blue button (push to talk)

## QuickStart
After cloning this repository:
```
$ cd codec2
$ git submodule init
$ git submodule update
```
Follow GCC install instructions here: [steps 2 and 4 for toolchain](https://github.com/drowe67/codec2/tree/master/stm32).  Then, install [STM32CubeF7](https://www.st.com/en/embedded-software/stm32cubef7.html) into location of your choice, `/opt` for example
```
(from root directory of this project)
$ mkdir build
$ cd build
$ cmake .. -DCMAKE_TOOLCHAIN_FILE=../codec2/stm32/cmake/STM32_Toolchain.cmake -DPERIPHLIBDIR=/opt/STM32Cube_FW_F7_V1.15.0
$ make
```
Ensure gcc-arm is in your path prior to running cmake, if not delete your build directory and re-run cmake with gcc-arm in path.

### Flashing stm32f7-discovery
https://github.com/texane/stlink
providing address isnt needed when using st-flash with .hex files
i.e: 
``st-flash --format ihex write <project>.hex``

## detailed operation of each project
### passthru
Used for evaluating audio on stm32f746g-discovery.  Upon startup, LCD screen awaits user selection of audio sample rate, permitted comparison of performance and various audio sampling rates.  This is needed to check board audio quality by itself without vocoder.
### vocoder_passthru
Used for evaluating codec2 performace on stm32f746g-discovery with simultaneous encoding -> decoding.   Upon startup, LCD screen awaits  user selection of codec2 mode (bitrate), along with audio hardware sampling rate.   Hardware sample rate greater than 8ksps is decimated-interpolated to 8ksps for codec2.

If needed to select different hardware audio rate, it must be selected prior to codec2 mode selection.   Codec2 mode selection on touchscreen starts codec2.

Latency of playback is adjusted on LCD-touchscreen.  Long latency here is required to properly hear speech quality of vocoder.
### lora_transceiver
Used for testing codec2 over sx1272/sx1276/sx1261/sx1262.   LoRa shield is auto-detected on startup, allowing different radio device to be plugged in and quickly compared.  Upon startup, touchscreen awaits  user selection of codec2 mode (bitrate). To change codec2 rate, reset is required using black button.

During operation, the touchscreen permits adjustment of LoRa bandwidth, LoRa spreading factor and microphone gain.  Bottom of LCD shows radio mode (tx or rx) along with TX duty cycle or RX RSSI/SNR.

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
codec2 mode | samples per frame |  encoded bytes per frame | frames per lora packet | LoRa packet length bytes | LoRa BW KHz | LoRa SF | air-time used percent | packet duration (ms)
----------- | ----------------- | ------------------------ | ---------------------- | ------------------------ | ----------- | ------- | --------------------  | ---------
   3200     |       160         |          8               |       16               |      128                 |     500     |   10    |       96              |  
   2400     |       160         |          6               |       16               |       96                 |     500     |   10    |       77              |  237
   1600     |       320         |          8               |        8               |       64                 |     500     |   10    |       56              |
   1600     |       320         |          8               |       16               |      128                 |     500     |   11    |       86              |  555
   1400     |       320         |          7               |       16               |      112                 |     500     |   11    |       77              |  493
   1300     |       320         |         6.5              |        8               |       52                 |     500     |   11    |       90              |  288
   1200     |       320         |          6               |       16               |       96                 |     500     |   11    |       71              |  452
   1200     |       320         |          6               |       32               |      192                 |     500     |   11    |       62              |  1281
   700C     |       320         |         3.5              |       16               |       56                 |     250     |   11    |       96              |  576
   450      |       320         |         2.25             |       16               |       36                 |     250     |   11    |       71              |  453

LoRa data rate selection is only possible in steps by a factor of two.  Codec2 bit-rate change will affect LoRa packet duty cycle.  When duty cycle is under 50%, the LoRa data-rate can be reduced (bandwidth reduced or SF increased).  If packet duty cycle is over 100%, then LoRa data-rate must be increased to faster.  Typical 2.5 to 2.7dB change in link budget for each step of LoRa data-rate.

Latency across the radio link is due to LoRa packet duration.

