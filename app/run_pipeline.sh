#!/bin/bash

############################################################
# Direct Visual LiDAR Calibration — full 4-step pipeline
# (bundled inside the AppImage; invoked by AppRun)
#
# Runs preprocess -> SuperGlue matching -> initial guess ->
# calibrate, each step force-stopped after its own wait time
# (these processes don't exit on their own), same model as
# the native gnome-terminal-tabs workflow.
#
# Author : Azad Kumar Jha
# ROS    : Jazzy
############################################################

set -uo pipefail

usage() {
    cat <<EOF
Usage: $(basename "$0") [--dataset <dir>] [--output <dir>] [options]

Defaults: when launched as a .AppImage, AppRun sets \$BAGS_DIR and
\$OUTPUT_DIR relative to the AppImage file's own location, so a
calibration_bags/ folder placed next to the AppImage is picked up
automatically with no flags needed.

  --dataset <dir>   Directory containing bag1, bag2, bag3...
                     (default: \$BAGS_DIR, normally <appimage-dir>/calibration_bags)
  --output <dir>    Directory to write processed output + calib.json
                     (default: \$OUTPUT_DIR, normally <appimage-dir>/calibration_preprocessed)

Options:
  --preprocess-wait <sec>     default 480
  --superglue-wait <sec>      default 30
  --initial-guess-wait <sec>  default 30
  --calibration-wait <sec>    default 300
  --kill-grace <sec>          default 5
  --visualize                 enable preprocess's live GLFW viewer (-v).
                               Needs a real X11 display. Off by default
                               so this runs headless/in containers.
  --status-interval <sec>     how often to print/update progress while
                               a step is running, default 10
EOF
}

DATASET_DIR="${BAGS_DIR:-}"
OUTPUT_DIR="${OUTPUT_DIR:-}"
PREPROCESS_WAIT="${PREPROCESS_WAIT:-480}"
SUPERGLUE_WAIT="${SUPERGLUE_WAIT:-30}"
INITIAL_GUESS_WAIT="${INITIAL_GUESS_WAIT:-30}"
CALIBRATION_WAIT="${CALIBRATION_WAIT:-300}"
KILL_GRACE="${KILL_GRACE:-5}"
VISUALIZE=0
STATUS_INTERVAL=10

while [ $# -gt 0 ]; do
    case "$1" in
        --dataset) DATASET_DIR="$2"; shift 2 ;;
        --output) OUTPUT_DIR="$2"; shift 2 ;;
        --preprocess-wait) PREPROCESS_WAIT="$2"; shift 2 ;;
        --superglue-wait) SUPERGLUE_WAIT="$2"; shift 2 ;;
        --initial-guess-wait) INITIAL_GUESS_WAIT="$2"; shift 2 ;;
        --calibration-wait) CALIBRATION_WAIT="$2"; shift 2 ;;
        --kill-grace) KILL_GRACE="$2"; shift 2 ;;
        --visualize) VISUALIZE=1; shift ;;
        --status-interval) STATUS_INTERVAL="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown argument: $1"; usage; exit 1 ;;
    esac
done

if [ -z "$DATASET_DIR" ] || [ -z "$OUTPUT_DIR" ]; then
    echo "ERROR: no dataset/output directory resolved."
    usage
    exit 1
fi

if [ ! -d "$DATASET_DIR" ]; then
    echo "ERROR: Dataset directory not found: $DATASET_DIR"
    echo "Place your calibration bag folders (bag1, bag2, ...) there and re-run."
    exit 1
fi

LOG_DIR="$OUTPUT_DIR/logs"
mkdir -p "$OUTPUT_DIR" "$LOG_DIR"
STATUS_FILE="$OUTPUT_DIR/.pipeline_status"

GREEN="\e[32m"; RED="\e[31m"; YELLOW="\e[33m"; BLUE="\e[34m"; CYAN="\e[36m"; NC="\e[0m"

TOTAL_STEPS=4

############################################################
# run_step: launch a long-running/non-terminating process in
# its own process group. While it runs, print/refresh a
# progress line every $STATUS_INTERVAL seconds (and keep
# $STATUS_FILE updated) so it's always clear which step is
# active and how far into its window it is — these steps
# don't print their own progress and don't exit on their own.
# After $wait_secs, SIGTERM (then SIGKILL if needed) the whole
# process group before moving to the next step.
############################################################

run_step() {
    local step_num="$1" title="$2" logfile="$3" wait_secs="$4"; shift 4
    echo
    echo -e "${YELLOW}=== STEP ${step_num}/${TOTAL_STEPS} - ${title} (window: ${wait_secs}s) ===${NC}"
    echo "RUNNING|${step_num}|${TOTAL_STEPS}|${title}|0|${wait_secs}" > "$STATUS_FILE"

    setsid "$@" > "$LOG_DIR/$logfile" 2>&1 &
    local pid=$!

    local waited=0 last_report=0
    while [ $waited -lt "$wait_secs" ]; do
        if ! kill -0 "$pid" 2>/dev/null; then
            wait "$pid"
            local status=$?
            if [ $status -eq 0 ]; then
                echo -e "${GREEN}SUCCESS (exited early):${NC} STEP ${step_num}/${TOTAL_STEPS} - ${title} (after ${waited}s)"
                echo "DONE|${step_num}|${TOTAL_STEPS}|${title}|${waited}|${wait_secs}" > "$STATUS_FILE"
            else
                echo -e "${RED}FAILED:${NC} STEP ${step_num}/${TOTAL_STEPS} - ${title} (exit $status) — see $LOG_DIR/$logfile"
                echo "FAILED|${step_num}|${TOTAL_STEPS}|${title}|${waited}|${wait_secs}" > "$STATUS_FILE"
                exit $status
            fi
            return 0
        fi
        sleep 1
        waited=$((waited + 1))
        if [ $((waited - last_report)) -ge "$STATUS_INTERVAL" ]; then
            echo -e "${BLUE}...${NC} STEP ${step_num}/${TOTAL_STEPS} - ${title}: ${waited}s / ${wait_secs}s elapsed"
            echo "RUNNING|${step_num}|${TOTAL_STEPS}|${title}|${waited}|${wait_secs}" > "$STATUS_FILE"
            last_report=$waited
        fi
    done

    echo -e "${BLUE}Time elapsed, stopping STEP ${step_num}/${TOTAL_STEPS} - ${title}...${NC}"
    kill -TERM -- -"$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
    sleep "$KILL_GRACE"
    if kill -0 "$pid" 2>/dev/null; then
        echo -e "${YELLOW}Still alive after SIGTERM, sending SIGKILL...${NC}"
        kill -KILL -- -"$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null
    fi
    wait "$pid" 2>/dev/null
    echo -e "${GREEN}STOPPED:${NC} STEP ${step_num}/${TOTAL_STEPS} - ${title} — see $LOG_DIR/$logfile"
    echo "DONE|${step_num}|${TOTAL_STEPS}|${title}|${wait_secs}|${wait_secs}" > "$STATUS_FILE"
}

echo -e "${GREEN}"
echo "=============================================================="
echo "      DIRECT VISUAL LIDAR CALIBRATION PIPELINE"
echo "=============================================================="
echo -e "${NC}"
echo -e "${CYAN}Dataset Directory :${NC} $DATASET_DIR"
echo -e "${CYAN}Output Directory  :${NC} $OUTPUT_DIR"
echo -e "${CYAN}Status file       :${NC} $STATUS_FILE  (cat this from another terminal to check progress)"

PREPROCESS_FLAGS="-a -d"
if [ "$VISUALIZE" -eq 1 ]; then
    PREPROCESS_FLAGS="$PREPROCESS_FLAGS -v"
    echo -e "${YELLOW}--visualize set: preprocess will open a GLFW viewer (needs X11)${NC}"
fi

run_step 1 "PREPROCESS" "01_preprocess.log" "$PREPROCESS_WAIT" \
    ros2 run direct_visual_lidar_calibration preprocess "$DATASET_DIR" "$OUTPUT_DIR" $PREPROCESS_FLAGS

run_step 2 "SUPERGLUE MATCHING (CPU)" "02_superglue.log" "$SUPERGLUE_WAIT" \
    ros2 run direct_visual_lidar_calibration find_matches_superglue.py "$OUTPUT_DIR"

run_step 3 "INITIAL GUESS" "03_initial_guess.log" "$INITIAL_GUESS_WAIT" \
    ros2 run direct_visual_lidar_calibration initial_guess_auto "$OUTPUT_DIR"

run_step 4 "CALIBRATION" "04_calibration.log" "$CALIBRATION_WAIT" \
    ros2 run direct_visual_lidar_calibration calibrate "$OUTPUT_DIR"

echo
echo "=============================================================="
echo "VERIFYING CALIBRATION OUTPUT"
echo "=============================================================="

if [ -f "$OUTPUT_DIR/calib.json" ]; then
    echo -e "${GREEN}Calibration completed successfully!${NC}"
    ls -lh "$OUTPUT_DIR/calib.json"
    ls "$OUTPUT_DIR"
else
    echo -e "${RED}Calibration appears incomplete.${NC} calib.json was NOT found."
    echo "Check logs in $LOG_DIR"
    exit 1
fi

echo
echo -e "${GREEN}"
echo "=============================================================="
echo "             PIPELINE FINISHED"
echo "=============================================================="
echo -e "${NC}"
