# CAD (Channel Activity Detection) Testing

Testing CAD parameters for frequency hopping synchronization on LR20xx.

## Build with FHSS enabled

```bash
cd build
cmake .. -DRADIO=LR2021 -DENABLE_HOPPING=ON
make lora_transceiver_LR2021
```

This sets default bandwidth to 125kHz (required for FCC Part 15 hopping).

## UART Commands for CAD Testing

| Command | Description |
|---------|-------------|
| `h` | Start/stop CAD scan |
| `i` | Show FHSS status (CAD config) |
| `I` | Show FHSS statistics |
| `1-4` | Set CAD symbol count |
| `+`/`-` | Adjust detect_peak +/-1 |
| `p` | Toggle PNR delta (0 or 8) |
| `P` | Toggle continuous preamble TX |
| `L`/`l` | Adjust preamble length +/-16 symbols |
| `n` | Hop to next random channel |
| `c` | Print current channel |

## CAD Parameters

### Symbol Count (`cad_symb_nb`: 1-4)
Number of symbols used for CAD detection.
- **1 symbol**: Fastest, but less reliable
- **2 symbols**: Good balance (default)
- **4 symbols**: Most reliable, but slower

### Detect Peak (`cad_detect_peak`)
Detection threshold - SF dependent. Default values from datasheet:

| SF | 1 symb | 2 symb | 4 symb |
|----|--------|--------|--------|
| 5  | 60 | 56 | 51 |
| 6  | 60 | 56 | 51 |
| 7  | 60 | 56 | 51 |
| 8  | 64 | 58 | 54 |
| 9  | 64 | 58 | 56 |
| 10 | 66 | 60 | 60 |
| 11 | 70 | 64 | 60 |
| 12 | 74 | 68 | 64 |

- **Higher value**: Fewer false positives, may miss weak signals
- **Lower value**: More sensitive, more false positives

### PNR Delta (`pnr_delta`: 0 or 8)
Best-effort CAD mode.
- **8 (on)**: CAD stops early if clear non-detection, faster when no signal
- **0 (off)**: Full CAD duration always

## Test Scripts

### Simple Single Test
```bash
./cad_test.sh <serial_port> <symb_nb> <peak_adj> <pnr> <sweeps>
```

Example - test with 2 symbols, default peak, PNR on, 20 sweeps:
```bash
./cad_test.sh /dev/ttyACM0 2 0 8 20
```

Example - test with increased detect_peak (+10):
```bash
./cad_test.sh /dev/ttyACM0 2 10 8 20
```

### Automated Test Matrix
```bash
./test_cad_params.sh <serial_port> <sweeps_per_test>
```

Tests multiple parameter combinations and saves results to `test_logs/cad/summary.csv`.

```bash
./test_cad_params.sh /dev/ttyACM0 20
```

## TX Preamble Detection Test (2 boards)

Setup:
- RX board on `/dev/ttyACM0`
- TX board on `/dev/ttyACM1`

### Start continuous TX:
```bash
echo 'P' > /dev/ttyACM1
```
TX will continuously send preambles on random channels until stopped.

### Start RX scanning:
```bash
./cad_test.sh /dev/ttyACM0 2 0 8 50
```

The RX should detect the preamble and print which channel was found.

### Stop continuous TX:
```bash
echo 'P' > /dev/ttyACM1
```

### Adjust preamble length on TX:
```bash
# Increase preamble by 16 symbols
echo 'L' > /dev/ttyACM1

# Check current preamble length
echo 'i' > /dev/ttyACM1
```

## Interpreting Results

### Sweep Output
```
sweep 1: 245 ms, no signal
sweep 2: 243 ms, no signal
...
CAD detected on ch12 (908.440 MHz)
```

- **Sweep time**: Time to scan all 50 channels
- **False detections**: "CAD detected" when no TX is active

### Tuning Goals

1. **Minimize false detections**: Increase `detect_peak` or use more symbols
2. **Ensure preamble detection**: Test with TX board, ensure detection within 1-2 sweeps
3. **Optimize sweep time**: Fewer symbols = faster sweep, but balance with reliability

### Expected Sweep Times (SF6, 125kHz BW)

| Symbols | Approx. sweep time |
|---------|-------------------|
| 1 | ~150 ms |
| 2 | ~250 ms |
| 4 | ~450 ms |

Preamble length should be > sweep time to ensure detection.

## Channel Plan

- **Channels**: 50 (FCC Part 15 compliant)
- **Base frequency**: 902.2 MHz
- **Channel spacing**: 520 kHz
- **End frequency**: 927.7 MHz
- **Bandwidth**: 125 kHz
