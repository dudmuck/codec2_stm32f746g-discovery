#!/bin/bash
# Simple FHSS test script
# Usage: ./test_fhss.sh [duration_seconds] [extra_commands] [delay_before_tx_ms]
#
# Examples:
#   ./test_fhss.sh           # 10 second test
#   ./test_fhss.sh 5         # 5 second test
#   ./test_fhss.sh 10 "" 1000  # 10 second test, 1000ms delay before TX

TX_DEV="/dev/ttyACM1"
RX_DEV="/dev/ttyACM0"
TX_LOG="tx.log"
RX_LOG="rx.log"
DURATION="${1:-10}"
EXTRA_CMD="${2:-}"
DELAY_MS="${3:-3000}"

# Convert delay to seconds for sleep
DELAY_SEC=$(echo "scale=3; $DELAY_MS / 1000" | bc)

echo "FHSS Test Script"
echo "================"
echo "TX: $TX_DEV -> $TX_LOG"
echo "RX: $RX_DEV -> $RX_LOG"
echo "Duration: ${DURATION}s"
echo ""

# Check devices exist
if [ ! -e "$TX_DEV" ]; then
    echo "Error: $TX_DEV not found"
    exit 1
fi
if [ ! -e "$RX_DEV" ]; then
    echo "Error: $RX_DEV not found"
    exit 1
fi

# Configure serial ports
stty -F "$TX_DEV" 115200 raw -echo -echoe -echok
stty -F "$RX_DEV" 115200 raw -echo -echoe -echok

# Kill any existing cat processes on these devices
fuser -k "$TX_DEV" 2>/dev/null
fuser -k "$RX_DEV" 2>/dev/null
sleep 0.2

# Clear old logs
> "$TX_LOG"
> "$RX_LOG"

# Start capturing logs in background
cat "$RX_DEV" >> "$RX_LOG" &
RX_PID=$!
cat "$TX_DEV" > "$TX_LOG" &
TX_PID=$!

cleanup() {
    echo "Cleaning up..."
    kill $RX_PID $TX_PID 2>/dev/null
    wait $RX_PID $TX_PID 2>/dev/null
}
trap cleanup EXIT

# Give cat time to start
sleep 0.3

# Determine rate digit from EXTRA_CMD if it's a numeric rate
# Maps: 3200->0, 2400->1, 1600->2, 1400->3, 1300->4, 1200->5, 700C->8
RATE_DIGIT="0"  # Default to 3200
case "$EXTRA_CMD" in
    3200) RATE_DIGIT="0" ;;
    2400) RATE_DIGIT="1" ;;
    1600) RATE_DIGIT="2" ;;
    1400) RATE_DIGIT="3" ;;
    1300) RATE_DIGIT="4" ;;
    1200) RATE_DIGIT="5" ;;
    700C|700c) RATE_DIGIT="8" ;;
    [0-8]) RATE_DIGIT="$EXTRA_CMD" ;;  # Allow direct digit
esac

# Reset both boards first to ensure clean state
echo "Resetting boards..."
echo -n "R" > "$TX_DEV"
echo -n "R" > "$RX_DEV"
sleep 1.5  # Wait for reset and bootup

# Clear logs after reset to remove any stale output
> "$TX_LOG"
> "$RX_LOG"
sleep 0.1

# Initialize both boards with rate and test mode
echo "Initializing TX with rate $RATE_DIGIT..."
echo -n "$RATE_DIGIT" > "$TX_DEV"
sleep 0.3
echo -n "T" > "$TX_DEV"
sleep 0.2

echo "Initializing RX with rate $RATE_DIGIT..."
echo -n "$RATE_DIGIT" > "$RX_DEV"
sleep 0.3
echo -n "T" > "$RX_DEV"
sleep 0.2

# Wait for RX to start CAD scanning
echo "Waiting ${DELAY_SEC}s for RX scanning..."
sleep "$DELAY_SEC"

# Start TX
echo "Starting TX..."
echo -n "t" > "$TX_DEV"

# Wait for transmission
echo "Transmitting for ${DURATION}s..."
sleep "$DURATION"

# Stop TX
echo "Stopping TX..."
echo -n "r" > "$TX_DEV"

# Wait for RX to return to scan (3 timeouts ~2s) and print stats
sleep 3

# Stop log capture
kill $RX_PID $TX_PID 2>/dev/null
wait $RX_PID $TX_PID 2>/dev/null
trap - EXIT

echo ""
echo "=== TX Log (last 20 lines) ==="
tail -20 "$TX_LOG"
echo ""
echo "=== RX Log (last 40 lines) ==="
tail -40 "$RX_LOG"
echo ""
echo "Logs saved to $TX_LOG and $RX_LOG"
