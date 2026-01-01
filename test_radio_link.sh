#!/bin/bash
#
# Radio link sequence test for all codec2 rates
# Tests that encoded frames are correctly sent/received over LoRa link
#
# Usage: ./test_radio_link.sh [duration_seconds] [rate] [bandwidth_khz]
#   duration_seconds: test duration per rate (default: 10)
#   rate: specific rate to test (3200,2400,1600,1400,1300,1200,700C)
#         if omitted, tests all rates
#   bandwidth_khz: LoRa bandwidth in kHz (1000,812,500,406,250,203,125,101,83,62,41,31)
#         if omitted, uses default 500kHz
#         use "all" to test all bandwidths
#         Note: both TX and RX will be set to this bandwidth
#

TX_DEV="/dev/ttyACM0"
RX_DEV="/dev/ttyACM1"
BAUD=115200
TEST_DURATION=${1:-10}  # Default 10 seconds per rate
SINGLE_RATE=${2:-""}    # Optional: test only this rate
TARGET_BW=${3:-500}     # Optional: bandwidth in kHz (default 500)
LOG_DIR="test_logs"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Validate TEST_DURATION is numeric
if ! [[ "$TEST_DURATION" =~ ^[0-9]+$ ]]; then
    echo -e "${RED:-\033[0;31m}Error: TEST_DURATION must be a positive integer, got '$TEST_DURATION'${NC:-\033[0m}"
    exit 1
fi

# Bandwidth table: kHz -> steps from 500kHz (positive = up, negative = down)
# Based on lr20xx bw_table: 31 <- ... <- 125 <- 250 <- 406 <- 500 -> 812 -> 1000
declare -A BW_STEPS
BW_STEPS[1000]=2   # 2 steps up from 500
BW_STEPS[812]=1    # 1 step up from 500
BW_STEPS[500]=0
BW_STEPS[406]=-1   # 1 step down from 500
BW_STEPS[250]=-2
BW_STEPS[203]=-3
BW_STEPS[125]=-4
BW_STEPS[101]=-5
BW_STEPS[83]=-6
BW_STEPS[62]=-7
BW_STEPS[41]=-8
BW_STEPS[31]=-9

# List of all bandwidths for "all" mode (ordered high to low)
# The adjust_sf_for_streaming() function automatically reduces SF to ensure
# TOA <= production time. Bandwidths where even SF5 can't achieve this will
# show warnings but can still be tested.
ALL_BWS=(1000 812 500 250 125 62 41)

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

# Set bandwidth by stepping from 500kHz default
# 'b' = step BW down, 'B' = step BW up (SF auto-adjusted for streaming)
set_bandwidth() {
    local dev=$1
    local steps=$2
    local i
    if [ "$steps" -gt 0 ]; then
        # Step up (higher BW)
        for ((i=0; i<steps; i++)); do
            send_cmd "$dev" "B"
            sleep 0.5  # Allow radio to reconfigure
        done
    elif [ "$steps" -lt 0 ]; then
        # Step down (lower BW)
        local abs_steps=$((-steps))
        for ((i=0; i<abs_steps; i++)); do
            send_cmd "$dev" "b"
            sleep 0.5  # Allow radio to reconfigure
        done
    fi
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
    local tx_log="$LOG_DIR/tx_${rate_name}_${TARGET_BW}kHz_${TIMESTAMP}.log"
    local rx_log="$LOG_DIR/rx_${rate_name}_${TARGET_BW}kHz_${TIMESTAMP}.log"

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

    # Set bandwidth if not default - step both boards together
    local bw_steps=${BW_STEPS[$TARGET_BW]}
    if [ "$bw_steps" -ne 0 ]; then
        local dir="down"
        local cmd="b"
        [ "$bw_steps" -gt 0 ] && { dir="up"; cmd="B"; }
        local abs_steps=${bw_steps#-}  # Remove minus sign for display
        echo "  Setting bandwidth to ${TARGET_BW}kHz ($abs_steps steps $dir)..."
        for ((i=0; i<abs_steps; i++)); do
            send_cmd "$TX_DEV" "$cmd"
            send_cmd "$RX_DEV" "$cmd"
            sleep 0.5
        done
        sleep 1  # Allow both boards to stabilize
    fi

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
    local gaps=$(echo "$stats_line" | grep -oP 'gaps=\K[0-9]+')
    # Parse overflow=count/bytes format
    local overflow_count=$(echo "$stats_line" | grep -oP 'overflow=\K[0-9]+')
    local overflow_bytes=$(echo "$stats_line" | grep -oP 'overflow=[0-9]+/\K[0-9]+')

    # Default values for missing stats (old firmware compatibility)
    [ -z "$gaps" ] && gaps=0
    [ -z "$overflow_count" ] && overflow_count=0
    [ -z "$overflow_bytes" ] && overflow_bytes=0

    # Result key includes bandwidth for multi-BW testing
    local result_key="${TARGET_BW}kHz_${rate_name}"

    # Check for errors
    if [ -z "$frames" ]; then
        echo -e "  ${RED}FAIL${NC}: No frames received (check connection)"
        RESULTS[$result_key]="FAIL: No frames"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        return 1
    elif [ "$gaps" != "0" ]; then
        echo -e "  ${RED}FAIL${NC}: frames=$frames gaps=$gaps (streaming gaps detected)"
        RESULTS[$result_key]="FAIL: frames=$frames gaps=$gaps"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        return 1
    elif [ "$dropped" != "0" ] || [ "$dup" != "0" ]; then
        echo -e "  ${RED}FAIL${NC}: frames=$frames dropped=$dropped dup=$dup"
        RESULTS[$result_key]="FAIL: frames=$frames dropped=$dropped dup=$dup"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        return 1
    else
        # Show overflow stats (informational - overflow is handled correctly)
        local overflow_info=""
        if [ "$overflow_count" != "0" ]; then
            overflow_info=" overflow=${overflow_count}pkts/${overflow_bytes}B"
        fi
        echo -e "  ${GREEN}PASS${NC}: frames=$frames dropped=0 dup=0 gaps=0${overflow_info}"
        RESULTS[$result_key]="PASS ($frames frames${overflow_info})"
        PASS_COUNT=$((PASS_COUNT + 1))
        return 0
    fi
}

print_summary() {
    echo ""
    echo "========================================"
    echo "TEST SUMMARY"
    echo "========================================"
    echo "Duration per rate: ${TEST_DURATION}s"
    if [ "$TEST_ALL_BW" = "1" ]; then
        echo "Bandwidths tested: ${ALL_BWS[*]} kHz"
    else
        echo "Bandwidth: ${TARGET_BW}kHz"
    fi
    echo ""

    # Determine which bandwidths to show in summary
    local bw_list
    if [ "$TEST_ALL_BW" = "1" ]; then
        bw_list=("${ALL_BWS[@]}")
    else
        bw_list=($TARGET_BW)
    fi

    for bw in "${bw_list[@]}"; do
        echo "--- ${bw}kHz ---"
        for rate_idx in 0 1 2 3 4 5 8; do
            local rate_name=${RATES[$rate_idx]}
            local result_key="${bw}kHz_${rate_name}"
            local result=${RESULTS[$result_key]}
            # Only print if this rate was tested
            if [ -n "$result" ]; then
                if [[ $result == PASS* ]]; then
                    echo -e "  $rate_name: ${GREEN}$result${NC}"
                else
                    echo -e "  $rate_name: ${RED}$result${NC}"
                fi
            fi
        done
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

# Run tests for a specific bandwidth
run_tests_for_bw() {
    local bw=$1
    TARGET_BW=$bw

    echo ""
    echo "########################################"
    echo "# Testing at ${TARGET_BW}kHz bandwidth"
    echo "########################################"

    if [ -n "$SINGLE_RATE" ]; then
        # Test single rate
        run_test ${RATE_IDX[$SINGLE_RATE]}
    else
        # Test all rates
        for rate_idx in 0 1 2 3 4 5 8; do
            run_test $rate_idx
        done
    fi
}

# Main
echo "Radio Link Sequence Test"
echo "========================"
echo "TX device: $TX_DEV"
echo "RX device: $RX_DEV"
echo "Test duration: ${TEST_DURATION}s per rate"

# Check for "all" bandwidth mode
if [ "$TARGET_BW" = "all" ]; then
    echo "Bandwidth: ALL (${ALL_BWS[*]} kHz)"
    TEST_ALL_BW=1
else
    echo "Bandwidth: ${TARGET_BW}kHz"
    TEST_ALL_BW=0
    # Validate single bandwidth
    if [ -z "${BW_STEPS[$TARGET_BW]}" ]; then
        echo -e "${RED}Error: Invalid bandwidth '${TARGET_BW}'${NC}"
        echo "Valid bandwidths: 1000, 812, 500, 406, 250, 203, 125, 101, 83, 62, 41, 31, or 'all'"
        exit 1
    fi
fi

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

# Run tests
if [ "$TEST_ALL_BW" = "1" ]; then
    # Test all bandwidths
    for bw in "${ALL_BWS[@]}"; do
        run_tests_for_bw $bw
    done
else
    # Test single bandwidth
    run_tests_for_bw $TARGET_BW
fi

print_summary

# Exit with failure if any tests failed
exit $FAIL_COUNT
