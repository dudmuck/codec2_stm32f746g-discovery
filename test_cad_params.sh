#!/bin/bash
#
# Test CAD parameters for false detection rate
# Usage: ./test_cad_params.sh [serial_port] [num_sweeps]
#
# CAD parameter commands:
#   1-4: set CAD symbol count
#   +/-: adjust detect_peak
#   p: toggle PNR delta (best-effort CAD)
#   h: start/stop CAD scan
#   i: show FHSS status

SERIAL_PORT="${1:-/dev/ttyACM0}"
NUM_SWEEPS="${2:-20}"
BAUD=115200
LOGDIR="test_logs/cad"
TIMEOUT_CMD="timeout"

mkdir -p "$LOGDIR"

# Check if serial port exists
if [ ! -e "$SERIAL_PORT" ]; then
    echo "Error: Serial port $SERIAL_PORT not found"
    exit 1
fi

# Function to send command and wait
send_cmd() {
    echo -n "$1" > "$SERIAL_PORT"
    sleep 0.1
}

# Function to run a CAD test with given parameters
# Args: symb_nb detect_peak_adj pnr_delta_on
run_cad_test() {
    local symb=$1
    local peak_adj=$2
    local pnr=$3
    local test_name="symb${symb}_peak${peak_adj}_pnr${pnr}"
    local logfile="$LOGDIR/cad_${test_name}_$(date +%Y%m%d_%H%M%S).log"

    echo "=== Testing: symb=$symb, peak_adj=$peak_adj, pnr=$pnr ==="

    # Configure CAD parameters
    # First set symbol count
    send_cmd "$symb"
    sleep 0.2

    # Adjust detect_peak (positive = increase, negative = decrease)
    if [ $peak_adj -gt 0 ]; then
        for ((i=0; i<peak_adj; i++)); do
            send_cmd "+"
            sleep 0.05
        done
    elif [ $peak_adj -lt 0 ]; then
        local neg_adj=$((0 - peak_adj))
        for ((i=0; i<neg_adj; i++)); do
            send_cmd "-"
            sleep 0.05
        done
    fi

    # Set PNR delta (toggle if needed)
    # Assume starting with pnr=8 (on), so toggle if we want it off
    if [ "$pnr" = "0" ]; then
        send_cmd "p"
        sleep 0.1
    fi

    # Show current status
    send_cmd "i"
    sleep 0.3

    # Start capture
    echo "Starting CAD scan, logging to $logfile"
    $TIMEOUT_CMD --foreground $((NUM_SWEEPS * 2))s cat "$SERIAL_PORT" > "$logfile" &
    CAT_PID=$!
    sleep 0.5

    # Start scan
    send_cmd "h"

    # Wait for sweeps to complete (estimate ~200ms per sweep at SF6/125kHz)
    local wait_time=$((NUM_SWEEPS / 4 + 2))
    echo "Waiting ${wait_time}s for $NUM_SWEEPS sweeps..."
    sleep $wait_time

    # Stop scan
    send_cmd "h"
    sleep 0.5

    # Stop capture
    kill $CAT_PID 2>/dev/null
    wait $CAT_PID 2>/dev/null

    # Restore PNR if we turned it off
    if [ "$pnr" = "0" ]; then
        send_cmd "p"
        sleep 0.1
    fi

    # Parse results
    local sweeps=$(grep -c "sweep.*no signal" "$logfile" 2>/dev/null || echo 0)
    local detections=$(grep -c "CAD detected" "$logfile" 2>/dev/null || echo 0)

    echo "Results: $sweeps sweeps, $detections false detections"
    echo ""

    # Append to summary
    echo "$test_name,$symb,$peak_adj,$pnr,$sweeps,$detections" >> "$LOGDIR/summary.csv"
}

# Initialize serial port
stty -F "$SERIAL_PORT" $BAUD raw -echo -echoe -echok

# Create summary file header
echo "test_name,symb_nb,peak_adj,pnr_delta,sweeps,false_detections" > "$LOGDIR/summary.csv"

echo "CAD Parameter Test Script"
echo "Serial port: $SERIAL_PORT"
echo "Target sweeps per test: $NUM_SWEEPS"
echo "Log directory: $LOGDIR"
echo ""

# Test matrix - try different combinations
# Format: symb_nb detect_peak_adjustment pnr_delta_on

# Baseline tests with default peak values
echo "=== Testing symbol count variations ==="
run_cad_test 1 0 8
run_cad_test 2 0 8
run_cad_test 4 0 8

# Test with increased detect_peak (more selective, fewer false positives)
echo "=== Testing increased detect_peak ==="
run_cad_test 2 5 8
run_cad_test 2 10 8

# Test with decreased detect_peak (more sensitive)
echo "=== Testing decreased detect_peak ==="
run_cad_test 2 -5 8
run_cad_test 2 -10 8

# Test with PNR delta off
echo "=== Testing PNR delta off ==="
run_cad_test 2 0 0
run_cad_test 2 5 0

echo ""
echo "=== Test Summary ==="
cat "$LOGDIR/summary.csv"
echo ""
echo "Detailed logs in: $LOGDIR"
