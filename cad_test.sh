#!/bin/bash
#
# Interactive CAD test - single parameter combination
# Usage: ./cad_test.sh <serial_port> <symb_nb> <peak_adj> <pnr> <sweeps>
#
# Example: ./cad_test.sh /dev/ttyACM0 2 0 8 20
#          Tests with 2 symbols, default peak, PNR on, 20 sweeps
#
# For TX preamble test (2 boards):
#   RX board: ./cad_test.sh /dev/ttyACM0 2 0 8 20
#   TX board: echo 'P' > /dev/ttyACM1  (send preamble)

SERIAL_PORT="${1:-/dev/ttyACM0}"
SYMB="${2:-2}"
PEAK_ADJ="${3:-0}"
PNR="${4:-8}"
SWEEPS="${5:-20}"

BAUD=115200

if [ ! -e "$SERIAL_PORT" ]; then
    echo "Error: $SERIAL_PORT not found"
    exit 1
fi

# Initialize serial port
stty -F "$SERIAL_PORT" $BAUD raw -echo -echoe -echok

send_cmd() {
    echo -n "$1" > "$SERIAL_PORT"
    sleep 0.1
}

echo "CAD Test: symb=$SYMB peak_adj=$PEAK_ADJ pnr=$PNR sweeps=$SWEEPS"
echo "Port: $SERIAL_PORT"
echo ""

# Set symbol count
send_cmd "$SYMB"
sleep 0.2

# Adjust detect_peak
if [ $PEAK_ADJ -gt 0 ]; then
    for ((i=0; i<PEAK_ADJ; i++)); do send_cmd "+"; sleep 0.03; done
elif [ $PEAK_ADJ -lt 0 ]; then
    for ((i=0; i<${PEAK_ADJ#-}; i++)); do send_cmd "-"; sleep 0.03; done
fi

# Toggle PNR if needed (starts at 8)
if [ "$PNR" = "0" ]; then
    send_cmd "p"
fi

# Show status
send_cmd "i"
sleep 0.3

# Capture output
echo "Starting scan... (Ctrl+C to stop)"
send_cmd "h"

# Show output in real-time, count results
timeout $((SWEEPS / 3 + 5))s cat "$SERIAL_PORT" | tee /dev/stderr | {
    sweeps=0
    detections=0
    while IFS= read -r line; do
        if [[ "$line" == *"sweep"*"no signal"* ]]; then
            ((sweeps++))
        elif [[ "$line" == *"CAD detected"* ]]; then
            ((detections++))
        fi
        if [ $sweeps -ge $SWEEPS ]; then
            break
        fi
    done
    echo ""
    echo "=== Results: $sweeps sweeps, $detections detections ==="
}

# Stop scan
send_cmd "h"

# Restore PNR if we changed it
if [ "$PNR" = "0" ]; then
    send_cmd "p"
fi

echo "Done"
