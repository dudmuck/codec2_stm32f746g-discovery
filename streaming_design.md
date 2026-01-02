# Streaming TX State Machine Design

## Problem
Early TX start (transmitting before buffer is full) has race conditions:
1. TX might complete before all frames are encoded
2. Multiple TX starts can occur for same buffer
3. Buffer position confusion after TX completes

## State Machine

```
                    ┌─────────────────────────────────────────┐
                    │                                         │
                    v                                         │
    ┌──────────┐  first frame   ┌───────────┐               │
    │   IDLE   │ ─────────────> │ ENCODING  │               │
    └──────────┘                └───────────┘               │
         ^                           │                       │
         │                           │ tx_buf_idx >= initial_bytes
         │                           │ (start TX)
         │                           v                       │
         │                     ┌───────────┐                │
         │    TxDone +         │ TX_ACTIVE │                │
         │    encoding done    └───────────┘                │
         │         │                 │                       │
         │         │                 │ tx_buf_idx >= lora_payload_length
         │         │                 │ (all frames produced)
         │         │                 v                       │
         │         │           ┌───────────┐                │
         │         └───────────│  WAIT_TX  │────────────────┘
         │                     └───────────┘    TxDone
         │                           │
         │    TxDone before          │
         │    encoding done          │
         │    (UNDERFLOW!)           │
         └───────────────────────────┘
```

## States

| State | Description | tx_buf_idx | streaming_tx_active |
|-------|-------------|------------|---------------------|
| IDLE | Ready for new cycle | 0 | 0 |
| ENCODING | Producing frames, TX not started | 0 < idx < initial | 0 |
| TX_ACTIVE | TX started, still encoding | initial <= idx < total | 1 |
| WAIT_TX | All frames produced, TX in progress | total | 1 |

## Transitions

1. **IDLE → ENCODING**: When first frame of new cycle starts
2. **ENCODING → TX_ACTIVE**: When tx_buf_idx >= initial_bytes → start TX
3. **TX_ACTIVE → WAIT_TX**: When tx_buf_idx >= lora_payload_length
4. **WAIT_TX → IDLE**: When TxDone callback fires → reset for next cycle
5. **TX_ACTIVE → IDLE (error)**: TxDone before encoding done = underflow

## Key Variables

```c
typedef enum {
    STREAM_IDLE,
    STREAM_ENCODING,
    STREAM_TX_ACTIVE,
    STREAM_WAIT_TX
} stream_state_t;

volatile stream_state_t stream_state;
volatile uint32_t stream_underflow_count;  // Error counter
```

## Synchronization Rules

1. **Only start TX once per cycle**: Check state == ENCODING before starting
2. **FIFO callback checks state**: Only send data if state == TX_ACTIVE or WAIT_TX
3. **TxDone checks state**: Different handling for WAIT_TX vs TX_ACTIVE
4. **Reset only in IDLE transition**: tx_buf_idx = 0 only when entering IDLE

## Implementation Changes

### In encoder loop (lora_xcvr.c):
```c
// At start of new cycle
if (tx_buf_idx == 0 && mid == 0 && stream_state == STREAM_IDLE) {
    stream_state = STREAM_ENCODING;
}

// After encoding frame, update producer
tx_buf_produced = tx_buf_idx;

// Start TX when ready
if (stream_state == STREAM_ENCODING && tx_buf_idx >= initial_bytes) {
    tx_encoded(lora_payload_length);
    stream_state = STREAM_TX_ACTIVE;
}

// All frames produced
if (stream_state == STREAM_TX_ACTIVE && tx_buf_idx >= lora_payload_length) {
    stream_state = STREAM_WAIT_TX;
}

// Skip encoding if waiting for TX
if (stream_state == STREAM_WAIT_TX) {
    // Don't encode, just wait
}
```

### In TxDone callback (radio_lr20xx.c):
```c
void Radio_txDoneBottom() {
    streaming_tx_active = 0;

    if (stream_state == STREAM_WAIT_TX) {
        // Normal completion
        stream_state = STREAM_IDLE;
    } else if (stream_state == STREAM_TX_ACTIVE) {
        // Underflow! TX completed before encoding done
        stream_underflow_count++;
        stream_state = STREAM_IDLE;
    }

    if (RadioEvents->TxDone_botHalf)
        RadioEvents->TxDone_botHalf();
}
```

### In Send_lr20xx_fifo_continue():
```c
if (stream_state != STREAM_TX_ACTIVE && stream_state != STREAM_WAIT_TX)
    return 0;
```
