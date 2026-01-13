# Opus Streaming Flow Diagrams

## System Overview

```mermaid
flowchart TB
    subgraph TX["TX Path (Transmitter)"]
        direction TB

        subgraph AudioIn["Audio Input (DMA)"]
            MIC[("Microphone")]
            DMA_IN["DMA Double Buffer"]
            MIC --> DMA_IN
            DMA_IN -->|"Half Complete IRQ"| HALF_IN["audio_in_half_pending"]
            DMA_IN -->|"Full Complete IRQ"| FULL_IN["audio_in_full_pending"]
        end

        subgraph Encoder["Opus Encoder"]
            ENC_TASK["Encoder Task<br/>(FreeRTOS)"]
            ENC_BUF["Encoded Frame<br/>(_bytes_per_frame)"]
        end

        subgraph TXBuffer["TX Buffer Management"]
            TX_BUF["tx_buf[512]"]
            TX_IDX["tx_buf_idx"]
        end

        subgraph RadioTX["Radio TX"]
            FIFO_TX["256-byte FIFO"]
            RF_TX[("RF Transmit")]
            FIFO_IRQ["FIFO_TX IRQ<br/>(refill FIFO)"]
            TX_DONE["TX_DONE IRQ"]
        end

        HALF_IN --> ENC_TASK
        FULL_IN --> ENC_TASK
        ENC_TASK --> ENC_BUF
        ENC_BUF --> TX_BUF
        TX_BUF --> FIFO_TX
        FIFO_TX --> RF_TX
        FIFO_TX -.->|"Almost empty"| FIFO_IRQ
        FIFO_IRQ -.->|"Write more"| TX_BUF
        RF_TX --> TX_DONE
    end

    subgraph RX["RX Path (Receiver)"]
        direction TB

        subgraph RadioRX["Radio RX"]
            RF_RX[("RF Receive")]
            FIFO_RX["256-byte FIFO"]
            RX_BUF["rx_buf[512]"]
            FIFO_RX_IRQ["FIFO_RX IRQ<br/>(read FIFO)"]
            RX_DONE["RX_DONE IRQ"]
        end

        subgraph Decoder["Opus Decoder"]
            DEC_TASK["streaming_rx_decode()"]
            DEC_IDX["rx_decode_idx"]
            FIFO_IDX["rx_fifo_read_idx"]
            DEC_BUF["Decoded PCM<br/>(double buffer)"]
        end

        subgraph AudioOut["Audio Output (DMA)"]
            DMA_OUT["DMA Double Buffer"]
            HALF_OUT["audio_half_pending"]
            FULL_OUT["audio_full_pending"]
            SPKR[("Speaker")]
        end

        RF_RX --> FIFO_RX
        FIFO_RX -.->|"Threshold"| FIFO_RX_IRQ
        FIFO_RX_IRQ --> RX_BUF
        FIFO_RX --> RX_DONE
        RX_BUF --> DEC_TASK
        DEC_IDX -->|"< rx_fifo_read_idx"| DEC_TASK
        DEC_TASK --> DEC_BUF
        DEC_BUF --> DMA_OUT
        DMA_OUT -->|"Half Complete"| HALF_OUT
        DMA_OUT -->|"Full Complete"| FULL_OUT
        HALF_OUT -.->|"put_spkr()"| DEC_BUF
        FULL_OUT -.->|"put_spkr()"| DEC_BUF
        DMA_OUT --> SPKR
    end
```

## TX Stream State Machine

```mermaid
flowchart TD
    IDLE["STREAM_IDLE<br/>Ready for new cycle"]
    ENCODING["STREAM_ENCODING<br/>Encoding frames"]
    TX_ACTIVE["STREAM_TX_ACTIVE<br/>TX + Encoding"]
    WAIT_TX["STREAM_WAIT_TX<br/>Wait for TxDone"]
    BUFFERING["STREAM_BUFFERING_NEXT<br/>Buffer next pkt"]
    FIFO_REFILL["FIFO_TX IRQ<br/>Refill FIFO"]

    IDLE -->|"Audio DMA"| ENCODING
    ENCODING -->|"Buffer full"| TX_ACTIVE
    TX_ACTIVE -->|"FIFO almost empty"| FIFO_REFILL
    FIFO_REFILL -->|"Continue"| TX_ACTIVE
    TX_ACTIVE -->|"All encoded"| WAIT_TX
    WAIT_TX -->|"TxDone + more"| BUFFERING
    WAIT_TX -->|"TxDone + end"| IDLE
    BUFFERING -->|"memmove"| TX_ACTIVE
```

## Concurrent Operation Sequence

```mermaid
sequenceDiagram
    participant MIC as Microphone DMA
    participant ENC as Opus Encoder
    participant TXBUF as TX Buffer
    participant RADIO as Radio FIFO
    participant RF as RF TX/RX
    participant RXBUF as RX Buffer
    participant DEC as Opus Decoder
    participant SPKR as Speaker DMA

    Note over MIC,SPKR: TX Side - Concurrent Encode & Transmit

    MIC->>ENC: Audio samples (DMA half)
    activate ENC
    ENC->>TXBUF: Encoded frame 1
    deactivate ENC
    MIC->>ENC: Audio samples (DMA full)
    activate ENC
    ENC->>TXBUF: Encoded frame 2

    Note over TXBUF,RADIO: Buffer threshold reached
    TXBUF->>RADIO: Initial FIFO fill
    RADIO->>RF: Start TX
    activate RF

    ENC->>TXBUF: Encoded frame 3
    deactivate ENC
    RADIO-->>TXBUF: FIFO_TX IRQ (refill)
    TXBUF->>RADIO: More data

    MIC->>ENC: Audio samples
    activate ENC
    ENC->>TXBUF: Encoded frame 4
    deactivate ENC

    RF-->>RADIO: TX Done
    deactivate RF

    Note over MIC,SPKR: RX Side - Concurrent Receive & Decode

    RF->>RADIO: RX packet
    activate RF
    RADIO-->>RXBUF: FIFO_RX IRQ
    RXBUF->>DEC: Decode frame 1
    activate DEC
    DEC->>SPKR: PCM samples
    deactivate DEC

    RADIO-->>RXBUF: More FIFO data
    RXBUF->>DEC: Decode frame 2
    activate DEC
    DEC->>SPKR: PCM samples
    SPKR-->>DEC: DMA half callback
    deactivate DEC

    RF-->>RADIO: RX Done
    deactivate RF
    RXBUF->>DEC: Decode remaining
    activate DEC
    DEC->>SPKR: Final PCM
    deactivate DEC
```

## Timing Constraints

```
TX Timing - Single Packet Mode (60ms Opus frame period)

Time(ms)  0    20    40    60    80   100   120   140   160   180
          |-----|-----|-----|-----|-----|-----|-----|-----|-----|
Audio DMA |<--- DMA Half --->|<--- DMA Full --->|<--- DMA Half -->|
          |     960 samp     |     960 samp     |     960 samp    |

Encoder   |Enc1 |           |Enc2 |            |Enc3 |
          |~25ms|           |~25ms|            |~25ms|

Radio TX                    |<-- Packet TOA -->|
                            |   ~45ms SF7      |

Margin                                         |<-- Available -->|
                                               |    ~35ms        |

Key: Streaming works when Packet TOA < Frame Period (60ms)
     Higher SF = longer TOA = less margin
     SF12 @ 500kHz ~ 200ms TOA (won't work for streaming)
```

```mermaid
gantt
    title Opus 16kbps LoRa SF7 500kHz - 60ms Frame Period
    dateFormat x
    axisFormat %L

    section Audio DMA
    DMA Half 960 samples     :a1, 0, 60ms
    DMA Full 960 samples     :a2, 60, 60ms

    section Encoder
    Encode Frame 1           :e1, 0, 25ms
    Encode Frame 2           :e2, 60, 25ms

    section Radio TX
    Packet TOA 1             :tx1, 25, 25ms
    Packet TOA 2             :tx2, 85, 25ms

    section Margin
    Margin frame 1           :crit, m1, 50, 10ms
    Margin frame 2           :crit, m2, 110, 10ms
```

### Timing varies by configuration

| Opus Mode | Sample Rate | Frame Period | Bytes/Frame |
|-----------|-------------|--------------|-------------|
| 16 kbps   | 16 kHz      | 60 ms        | 120 bytes   |
| 24 kbps   | 16 kHz      | 60 ms        | 180 bytes   |
| 32 kbps   | 16 kHz      | 60 ms        | 240 bytes   |
| 64 kbps   | 48 kHz      | 60 ms        | 480 bytes   |
| 96 kbps   | 48 kHz      | 60 ms        | 720 bytes   |

| Radio Config | Packet TOA (255 bytes) | Viable for 60ms? |
|--------------|------------------------|------------------|
| SF5 500kHz   | ~8 ms                  | Yes              |
| SF7 500kHz   | ~25 ms                 | Yes              |
| SF9 500kHz   | ~82 ms                 | No               |
| SF12 500kHz  | ~530 ms                | No               |
| FLRC 650kbps | ~3 ms                  | Yes              |
| FLRC 2.6Mbps | ~1 ms                  | Yes              |

## Key Variables

| Variable | Description |
|----------|-------------|
| `stream_state` | TX state machine state |
| `tx_buf_idx` | Write position in TX buffer |
| `tx_buf_produced` | Bytes produced by encoder |
| `tx_fifo_idx` | Bytes sent to radio FIFO |
| `rx_fifo_read_idx` | Bytes read from radio FIFO |
| `rx_decode_idx` | Bytes decoded by Opus |
| `_bytes_per_frame` | Opus encoded frame size |
| `lora_payload_length` | Radio packet payload size |

## Stream States

| State | Description |
|-------|-------------|
| `STREAM_IDLE` | Ready for new packet cycle |
| `STREAM_ENCODING` | Encoding frames, TX not yet started |
| `STREAM_TX_ACTIVE` | TX started, still encoding more frames |
| `STREAM_WAIT_TX` | All frames encoded, waiting for TX to finish |
| `STREAM_BUFFERING_NEXT` | Buffering next packet while current TX in progress |
