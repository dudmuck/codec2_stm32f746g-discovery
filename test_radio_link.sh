#!/bin/bash
#
# Radio link sequence test for all codec2 rates
# Tests that encoded frames are correctly sent/received over LoRa link
#
# Usage: ./test_radio_link.sh [duration_seconds] [rate]
#   duration_seconds: test duration per rate (default: 10)
#   rate: specific rate to test (3200,2400,1600,1400,1300,1200,700C)
#         if omitted, tests all rates
#

TX_DEV="/dev/ttyACM0"
RX_DEV="/dev/ttyACM1"
BAUD=115200
TEST_DURATION=${1:-10}  # Default 10 seconds per rate
SINGLE_RATE=${2:-""}    # Optional: test only this rate
LOG_DIR="test_logs"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Codec2 rates to test: index -> name
declare -A RATES
RATES[0]="3200"
RATES[1]="2400"
RATES[2]="1600"
RATES[3]="1400"
RATES[4]="1300"
RATES[5]="1200"
RATES[8]="700C"

# Reverse lookup: name -> index
declare -A RATE_IDX
RATE_IDX["3200"]=0
RATE_IDX["2400"]=1
RATE_IDX["1600"]=2
RATE_IDX["1400"]=3
RATE_IDX["1300"]=4
RATE_IDX["1200"]=5
RATE_IDX["700C"]=8

# Results tracking
declare -A RESULTS
PASS_COUNT=0
FAIL_COUNT=0

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

cleanup() {
    echo "Cleaning up..."
    # Kill background processes
    kill $TX_PID $RX_PID 2>/dev/null
    # Close file descriptors
    exec 3>&- 4>&- 2>/dev/null
    exit
}

trap cleanup EXIT INT TERM

check_devices() {
    if [ ! -e "$TX_DEV" ]; then
        echo -e "${RED}Error: TX device $TX_DEV not found${NC}"
        exit 1
    fi
    if [ ! -e "$RX_DEV" ]; then
        echo -e "${RED}Error: RX device $RX_DEV not found${NC}"
        exit 1
    fi
}

setup_serial() {
    local dev=$1
    stty -F "$dev" $BAUD raw -echo -echoe -echok -echoctl -echoke
}

send_cmd() {
    local dev=$1
    local cmd=$2
    echo -n "$cmd" > "$dev"
}

# Reset board and wait for it to be ready
reset_board() {
    local dev=$1
    local name=$2
    echo "  Resetting $name..."
    send_cmd "$dev" "R"
    sleep 2  # Wait for reset and boot
}

# Select codec2 rate (at startup screen)
select_rate() {
    local dev=$1
    local rate_idx=$2
    send_cmd "$dev" "$rate_idx"
    sleep 1  # Wait for codec2 init
}

run_test() {
    local rate_idx=$1
    local rate_name=${RATES[$rate_idx]}
    local tx_log="$LOG_DIR/tx_${rate_name}_${TIMESTAMP}.log"
    local rx_log="$LOG_DIR/rx_${rate_name}_${TIMESTAMP}.log"

    echo ""
    echo "========================================"
    echo "Testing codec2 rate: $rate_name (index $rate_idx)"
    echo "========================================"

    # Reset both boards
    reset_board "$TX_DEV" "TX"
    reset_board "$RX_DEV" "RX"

    # Select rate on both boards
    echo "  Selecting rate $rate_name on both boards..."
    select_rate "$TX_DEV" "$rate_idx"
    select_rate "$RX_DEV" "$rate_idx"
    sleep 1

    # Enable test mode on both
    echo "  Enabling test mode..."
    send_cmd "$TX_DEV" "T"
    send_cmd "$RX_DEV" "T"
    sleep 0.5

    # Start capturing RX output
    cat "$RX_DEV" > "$rx_log" 2>&1 &
    RX_PID=$!

    # Start capturing TX output
    cat "$TX_DEV" > "$tx_log" 2>&1 &
    TX_PID=$!

    sleep 0.5

    # Start TX
    echo "  Starting TX for $TEST_DURATION seconds..."
    send_cmd "$TX_DEV" "t"

    # Wait for test duration
    sleep $TEST_DURATION

    # Stop TX
    echo "  Stopping TX..."
    send_cmd "$TX_DEV" "r"
    sleep 1

    # Get stats from receiver
    send_cmd "$RX_DEV" "?"
    sleep 0.5

    # Stop capturing
    kill $TX_PID $RX_PID 2>/dev/null
    wait $TX_PID $RX_PID 2>/dev/null

    # Parse results from RX log
    local stats_line=$(grep "Test stats:" "$rx_log" | tail -1)
    local frames=$(echo "$stats_line" | grep -oP 'frames=\K[0-9]+')
    local dropped=$(echo "$stats_line" | grep -oP 'dropped=\K[0-9]+')
    local dup=$(echo "$stats_line" | grep -oP 'dup=\K[0-9]+')

    # Check for errors
    if [ -z "$frames" ]; then
        echo -e "  ${RED}FAIL${NC}: No frames received (check connection)"
        RESULTS[$rate_name]="FAIL: No frames"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        return 1
    elif [ "$dropped" != "0" ] || [ "$dup" != "0" ]; then
        echo -e "  ${RED}FAIL${NC}: frames=$frames dropped=$dropped dup=$dup"
        RESULTS[$rate_name]="FAIL: frames=$frames dropped=$dropped dup=$dup"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        return 1
    else
        echo -e "  ${GREEN}PASS${NC}: frames=$frames dropped=0 dup=0"
        RESULTS[$rate_name]="PASS: frames=$frames"
        PASS_COUNT=$((PASS_COUNT + 1))
        # Remove logs for passing tests
        rm -f "$tx_log" "$rx_log"
        return 0
    fi
}

print_summary() {
    echo ""
    echo "========================================"
    echo "TEST SUMMARY"
    echo "========================================"
    echo "Duration per rate: ${TEST_DURATION}s"
    echo ""

    for rate_idx in 0 1 2 3 4 5 8; do
        local rate_name=${RATES[$rate_idx]}
        local result=${RESULTS[$rate_name]}
        # Only print if this rate was tested
        if [ -n "$result" ]; then
            if [[ $result == PASS* ]]; then
                echo -e "  $rate_name: ${GREEN}$result${NC}"
            else
                echo -e "  $rate_name: ${RED}$result${NC}"
            fi
        fi
    done

    echo ""
    echo "----------------------------------------"
    if [ $FAIL_COUNT -eq 0 ]; then
        echo -e "${GREEN}All tests passed ($PASS_COUNT/$PASS_COUNT)${NC}"
    else
        echo -e "${RED}$FAIL_COUNT test(s) failed${NC}, $PASS_COUNT passed"
        echo "Failed test logs saved in: $LOG_DIR/"
    fi
}

# Main
echo "Radio Link Sequence Test"
echo "========================"
echo "TX device: $TX_DEV"
echo "RX device: $RX_DEV"
echo "Test duration: ${TEST_DURATION}s per rate"

# Validate single rate if specified
if [ -n "$SINGLE_RATE" ]; then
    if [ -z "${RATE_IDX[$SINGLE_RATE]}" ]; then
        echo -e "${RED}Error: Invalid rate '$SINGLE_RATE'${NC}"
        echo "Valid rates: 3200, 2400, 1600, 1400, 1300, 1200, 700C"
        exit 1
    fi
    echo "Testing rate: $SINGLE_RATE only"
fi
echo ""

check_devices
setup_serial "$TX_DEV"
setup_serial "$RX_DEV"

mkdir -p "$LOG_DIR"

# Test rates
if [ -n "$SINGLE_RATE" ]; then
    # Test single rate
    run_test ${RATE_IDX[$SINGLE_RATE]}
else
    # Test all rates
    for rate_idx in 0 1 2 3 4 5 8; do
        run_test $rate_idx
    done
fi

print_summary

# Exit with failure if any tests failed
exit $FAIL_COUNT
