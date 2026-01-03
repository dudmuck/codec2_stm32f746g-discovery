# FHSS Voice System Flow Diagrams

## TX Flow

```mermaid
flowchart TD
    subgraph Audio["Audio Input"]
        MIC[Microphone] --> ADC[ADC @ 8kHz]
        ADC --> BUF[Audio Buffer<br/>320 samples]
    end

    subgraph Vocoder["Codec2 Encoder"]
        BUF --> ENC[Codec2 Encode]
        ENC --> FRAME[Codec2 Frame<br/>4-8 bytes]
        FRAME --> ACCUM{Frames<br/>Accumulated?}
        ACCUM -->|No| WAIT[Wait for more<br/>audio]
        WAIT --> BUF
    end

    subgraph FHSS_TX["FHSS TX"]
        ACCUM -->|Yes| PKT[Build Packet<br/>HDR + Payload]
        PKT --> FREQ[Set Frequency<br/>ch = LFSR mod 50]
        FREQ --> SEND[LoRa TX]
        SEND --> DONE{TX Done}
        DONE --> HOP{Hop Count<br/>Reached?}
        HOP -->|No| WAIT
        HOP -->|Yes| LFSR[Advance LFSR]
        LFSR --> WAIT
    end

    style MIC fill:#e1f5fe
    style SEND fill:#ffcdd2
    style LFSR fill:#fff9c4
```

## RX Flow

```mermaid
flowchart TD
    subgraph Scan["CAD Scan"]
        IDLE[Idle] --> CAD[Start CAD<br/>on channel]
        CAD --> DET{Activity<br/>Detected?}
        DET -->|No| NEXT[Next Channel<br/>ch = LFSR mod 50]
        NEXT --> CAD
        DET -->|Yes| RX_SYNC[RX Mode<br/>Wait for packet]
    end

    subgraph Sync["Sync Phase"]
        RX_SYNC --> SYNC_PKT{Sync Packet<br/>Received?}
        SYNC_PKT -->|Timeout| CAD
        SYNC_PKT -->|Yes| PARSE[Parse Sync:<br/>LFSR state, next_ch]
        PARSE --> ACK_EN{ACK Mode?}
        ACK_EN -->|Yes| SEND_ACK[Send ACK]
        SEND_ACK --> SET_LFSR
        ACK_EN -->|No| SET_LFSR[Set LFSR State]
        SET_LFSR --> DATA_CH[Hop to Data Channel]
    end

    subgraph Data["Data Phase"]
        DATA_CH --> RX_DATA[RX Mode]
        RX_DATA --> PKT{Packet<br/>Received?}
        PKT -->|Timeout x3| CAD
        PKT -->|End Marker| CAD
        PKT -->|Data| DECODE[Extract Payload]
        DECODE --> HOP_CHK{Proactive<br/>Hop?}
        HOP_CHK -->|Yes| HOP[Advance LFSR<br/>Set new freq]
        HOP --> RX_DATA
        HOP_CHK -->|No| RX_DATA
    end

    subgraph Audio["Audio Output"]
        DECODE --> C2DEC[Codec2 Decode]
        C2DEC --> DAC[DAC @ 8kHz]
        DAC --> SPK[Speaker]
    end

    style CAD fill:#fff9c4
    style RX_DATA fill:#c8e6c9
    style SPK fill:#e1f5fe
```

## Sync Sequence (ACK Mode)

```mermaid
sequenceDiagram
    participant TX
    participant Air
    participant RX

    Note over RX: CAD Scanning all 50 channels

    TX->>Air: Sync Packet (long preamble)<br/>ch=N, contains LFSR state
    TX->>TX: Switch to RX mode

    Air->>RX: CAD detects preamble
    RX->>RX: Receive sync packet
    RX->>RX: Set LFSR = sync.lfsr_state

    RX->>Air: ACK Packet
    Air->>TX: ACK received

    TX->>TX: Hop to data channel
    RX->>RX: Hop to data channel

    Note over TX,RX: Both now synchronized on same channel sequence

    loop Data Phase
        TX->>Air: Data Packet (codec2 frames)
        Air->>RX: Receive & decode
        Note over TX,RX: Proactive hop after N packets
    end

    TX->>Air: End Packet (current ch)
    TX->>Air: End Packet (next ch)
    Note over RX: Return to CAD scan
```

## Channel Hopping Pattern

```mermaid
flowchart LR
    subgraph LFSR["16-bit LFSR"]
        STATE[State] --> XOR[XOR taps<br/>16,14,13,11]
        XOR --> SHIFT[Shift Right]
        SHIFT --> STATE
    end

    STATE --> MOD[mod 50]
    MOD --> CH[Channel 0-49]
    CH --> FREQ_CALC["902.2 + ch × 0.52 MHz"]
    FREQ_CALC --> RADIO[Set Radio Frequency]
```

## Packet Structure

```
Sync Packet (long preamble ~350ms):
┌──────────┬──────────┬──────────┬──────────┬──────────┐
│ sync_ch  │ next_ch  │ lfsr_hi  │ lfsr_lo  │ hop_cnt  │
│ (1 byte) │ (1 byte) │ (1 byte) │ (1 byte) │ (1 byte) │
└──────────┴──────────┴──────────┴──────────┴──────────┘

Data Packet:
┌──────────┬──────────┬──────────┬─────────────────────┐
│ pkt_type │ seq_num  │ hop_cnt  │ codec2 frames       │
│ (1 byte) │ (1 byte) │ (1 byte) │ (N bytes)           │
└──────────┴──────────┴──────────┴─────────────────────┘

End Packet:
┌──────────┬──────────┬──────────┐
│ 0xFF     │ 0x00...  │ padding  │
│ (marker) │          │          │
└──────────┴──────────┴──────────┘
```
