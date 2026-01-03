#!/bin/bash
#
# FHSS radio link test for all codec2 rates
# Tests that encoded frames are correctly sent/received over FHSS link
#
# Usage: ./test_fhss_rates.sh [duration_seconds] [rate]
#   duration_seconds: test duration per rate (default: 15)
#   rate: specific rate to test (3200,2400,1600,1400,1300,1200,700C)
#         if omitted, tests all rates
#

TX_DEV="/dev/ttyACM1"
RX_DEV="/dev/ttyACM0"
BAUD=115200
TEST_DURATION=${1:-15}  # Default 15 seconds per rate (longer for FHSS sync)
SINGLE_RATE=${2:-""}    # Optional: test only this rate
LOG_DIR="test_logs"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Validate TEST_DURATION is numeric
if ! [[ "$TEST_DURATION" =~ ^[0-9]+$ ]]; then
    echo -e "${RED:-\033[0;31m}Error: TEST_DURATION must be a positive integer, got '$TEST_DURATION'${NC:-\033[0m}"
    exit 1
fi

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
    wait $TX_PID $RX_PID 2>/dev/null
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

kill_serial_procs() {
    fuser -k "$TX_DEV" 2>/dev/null
    fuser -k "$RX_DEV" 2>/dev/null
    sleep 0.3
}

send_cmd() {
    local dev=$1
    local cmd=$2
    echo -n "$cmd" > "$dev"
}

# Select codec2 rate (at startup screen)
select_rate() {
    local dev=$1
    local rate_idx=$2
    send_cmd "$dev" "$rate_idx"
    sleep 0.5
}

run_test() {
    local rate_idx=$1
    local rate_name=${RATES[$rate_idx]}
    local tx_log="$LOG_DIR/fhss_tx_${rate_name}_${TIMESTAMP}.log"
    local rx_log="$LOG_DIR/fhss_rx_${rate_name}_${TIMESTAMP}.log"

    echo ""
    echo "========================================"
    echo "Testing FHSS with codec2 rate: $rate_name"
    echo "========================================"

    # Kill any existing serial processes
    kill_serial_procs

    # Configure serial ports
    stty -F "$TX_DEV" $BAUD raw -echo -echoe -echok 2>/dev/null
    stty -F "$RX_DEV" $BAUD raw -echo -echoe -echok 2>/dev/null

    # Start capturing output
    cat "$RX_DEV" > "$rx_log" 2>&1 &
    RX_PID=$!
    cat "$TX_DEV" > "$tx_log" 2>&1 &
    TX_PID=$!
    sleep 0.3

    # Reset MCUs to ensure clean state
    echo "  Resetting MCUs..."
    send_cmd "$TX_DEV" "R"
    send_cmd "$RX_DEV" "R"
    sleep 2

    # Select rate on both boards (this initializes)
    echo "  Selecting rate $rate_name on both boards..."
    select_rate "$TX_DEV" "$rate_idx"
    select_rate "$RX_DEV" "$rate_idx"
    sleep 1

    # Enable test mode on both
    echo "  Enabling test mode..."
    send_cmd "$TX_DEV" "T"
    send_cmd "$RX_DEV" "T"
    sleep 0.5

    # Wait for RX to start scanning (FHSS needs time to initialize and start CAD)
    echo "  Waiting for RX to scan (5s)..."
    sleep 5

    # Start TX
    echo "  Starting TX for $TEST_DURATION seconds..."
    send_cmd "$TX_DEV" "t"

    # Wait for test duration
    sleep $TEST_DURATION

    # Stop TX
    echo "  Stopping TX..."
    send_cmd "$TX_DEV" "r"

    # Wait for RX to timeout and print stats
    echo "  Waiting for RX stats (3s)..."
    sleep 3

    # Stop capturing
    kill $TX_PID $RX_PID 2>/dev/null
    wait $TX_PID $RX_PID 2>/dev/null

    # Parse results
    local synced=$(grep -c "FHSS synchronized" "$rx_log")
    local rx_stats=$(grep "RX stats:" "$rx_log" | tail -1)
    local rx_pkts=$(echo "$rx_stats" | grep -oP 'rx=\K[0-9]+')
    local crc_err=$(echo "$rx_stats" | grep -oP 'crc_err=\K[0-9]+')
    local hdr_err=$(echo "$rx_stats" | grep -oP 'hdr_err=\K[0-9]+')
    local false_sync=$(echo "$rx_stats" | grep -oP 'false_sync=\K[0-9]+')

    # Count "OK" frame reports (1000ms for 50 frames OK)
    local ok_frames=$(grep -c "frames OK" "$rx_log")

    # Get TX packet count
    local tx_pkts=$(grep "FHSS: TX stopped" "$tx_log" | grep -oP 'data_cnt=\K[0-9]+')

    # Default values
    [ -z "$rx_pkts" ] && rx_pkts=0
    [ -z "$crc_err" ] && crc_err=0
    [ -z "$hdr_err" ] && hdr_err=0
    [ -z "$false_sync" ] && false_sync=0
    [ -z "$tx_pkts" ] && tx_pkts=0
    [ -z "$ok_frames" ] && ok_frames=0

    # Determine pass/fail
    if [ "$synced" -eq 0 ]; then
        echo -e "  ${YELLOW}No sync - will retry${NC}"
        return 2  # Special return code for "retry"
    elif [ "$rx_pkts" -eq 0 ]; then
        echo -e "  ${RED}FAIL${NC}: Synced but no packets received"
        RESULTS[$rate_name]="FAIL: No packets"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        return 1
    elif [ "$crc_err" -gt 0 ] || [ "$hdr_err" -gt 0 ]; then
        echo -e "  ${YELLOW}WARN${NC}: rx=$rx_pkts/$tx_pkts crc_err=$crc_err hdr_err=$hdr_err"
        RESULTS[$rate_name]="WARN: rx=$rx_pkts/$tx_pkts crc=$crc_err hdr=$hdr_err"
        # Still count as pass if we got frames
        if [ "$rx_pkts" -gt 0 ]; then
            PASS_COUNT=$((PASS_COUNT + 1))
            return 0
        else
            FAIL_COUNT=$((FAIL_COUNT + 1))
            return 1
        fi
    else
        # Calculate packet loss percentage
        local loss_pct=0
        if [ "$tx_pkts" -gt 0 ]; then
            loss_pct=$(( (tx_pkts - rx_pkts) * 100 / tx_pkts ))
        fi
        echo -e "  ${GREEN}PASS${NC}: rx=$rx_pkts/$tx_pkts (${loss_pct}% loss) ok_frames=$ok_frames"
        RESULTS[$rate_name]="PASS: rx=$rx_pkts/$tx_pkts (${loss_pct}% loss)"
        PASS_COUNT=$((PASS_COUNT + 1))
        return 0
    fi
}

print_summary() {
    echo ""
    echo "========================================"
    echo "FHSS TEST SUMMARY"
    echo "========================================"
    echo "Duration per rate: ${TEST_DURATION}s"
    echo "Bandwidth: 125kHz (FHSS default)"
    echo ""

    for rate_idx in 0 1 2 3 4 5 8; do
        local rate_name=${RATES[$rate_idx]}
        local result=${RESULTS[$rate_name]}
        # Only print if this rate was tested
        if [ -n "$result" ]; then
            if [[ $result == PASS* ]]; then
                echo -e "  $rate_name: ${GREEN}$result${NC}"
            elif [[ $result == WARN* ]]; then
                echo -e "  $rate_name: ${YELLOW}$result${NC}"
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
    fi
    echo "Logs saved in: $LOG_DIR/"
}

# Main
echo "FHSS Radio Link Test"
echo "===================="
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
mkdir -p "$LOG_DIR"

# Run a test with retry on sync failure
run_test_with_retry() {
    local rate_idx=$1
    local rate_name=${RATES[$rate_idx]}

    run_test $rate_idx
    local result=$?

    if [ $result -eq 2 ]; then
        # Sync failed - retry once
        echo -e "  ${YELLOW}Retrying $rate_name...${NC}"
        sleep 2
        run_test $rate_idx
        result=$?
        if [ $result -eq 2 ]; then
            # Still no sync after retry
            echo -e "  ${RED}FAIL${NC}: RX never synced with TX (after retry)"
            RESULTS[$rate_name]="FAIL: No sync"
            FAIL_COUNT=$((FAIL_COUNT + 1))
        fi
    fi
}

# Run tests
if [ -n "$SINGLE_RATE" ]; then
    # Test single rate
    run_test_with_retry ${RATE_IDX[$SINGLE_RATE]}
else
    # Test all rates
    for rate_idx in 0 1 2 3 4 5 8; do
        run_test_with_retry $rate_idx
    done
fi

print_summary

# Exit with failure count
exit $FAIL_COUNT
