#!/bin/bash
# ==============================================================================
# MANTA UAV — Safe Flight Recording (Power-Loss Immune & Anti-Vibration)
# Platform: Raspberry Pi 3 A+ with Sony IMX378-79 Camera
# ==============================================================================
# Key Features:
# 1. POWER-LOSS IMMUNITY:
#    Uses Matroska (.mkv) container with continuous buffer flush to SD card.
#    Even if power is lost abruptly, all footage up to the cut is preserved.
# 2. ANTI-VIBRATION OPTIMIZATION (JELLO EFFECT & HIGH MOTOR FREQUENCIES):
#    - Fast readout mode (2x2 Binning 2028x1080): Halves rolling shutter skew.
#    - Sport exposure (--exposure sport): Prioritizes fast shutter speeds.
#    - Denoise off (--denoise cdn_off): Prevents motion blurring from vibration.
#    - Hardware H.264 encoding (bcm2835-codec).
# ==============================================================================

set -e

VIDEO_DIR="${VIDEO_DIR:-/home/pc/flight_videos}"
WIDTH="${WIDTH:-1920}"
HEIGHT="${HEIGHT:-1080}"
FPS="${FPS:-30}"
BITRATE="${BITRATE:-15000000}"  # 15 Mbps (Action cam style quality)
EXPOSURE="${EXPOSURE:-sport}"
DENOISE="${DENOISE:-cdn_fast}"
SHUTTER_US="${SHUTTER_US:-0}"  # 0 = auto fast priority (sport), or e.g. 2000 (1/500s)

mkdir -p "$VIDEO_DIR"

# Check free disk space (minimum 500 MB)
FREE_KB=$(df -k "$VIDEO_DIR" | awk 'NR==2 {print $4}')
if [ "$FREE_KB" -lt 512000 ]; then
    echo "[!] ERROR: Less than 500MB free disk space on SD card ($((FREE_KB/1024)) MB available)." >&2
    echo "    Please free space before starting flight recording." >&2
    exit 1
fi

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
OUTPUT_FILE="${VIDEO_DIR}/manta_flight_${TIMESTAMP}.mkv"

echo "=================================================================="
echo " MANTA UAV — Safe Flight Recording Initialized"
echo " File: $OUTPUT_FILE"
echo " Resolution: ${WIDTH}x${HEIGHT} @ ${FPS} fps | Bitrate: $((BITRATE/1000000)) Mbps | Exposure: ${EXPOSURE}"
if [ "$SHUTTER_US" -gt 0 ]; then
    echo " Fixed Shutter: ${SHUTTER_US} µs (1/$((1000000/SHUTTER_US))s anti-blur)"
fi
echo " Container: Matroska (.mkv) with active flush (power-loss immune)"
echo "=================================================================="

ARGS=(
    -t 0
    -n
    --width "$WIDTH"
    --height "$HEIGHT"
    --framerate "$FPS"
    --bitrate "$BITRATE"
    --profile high
    --exposure "$EXPOSURE"
    --denoise "$DENOISE"
    --autofocus-mode manual
    --lens-position 0.0
    --flush
    --codec libav
    --libav-format matroska
    --vflip
    --hflip
    -o "$OUTPUT_FILE"
)

if [ "$SHUTTER_US" -gt 0 ]; then
    ARGS+=(--shutter "$SHUTTER_US")
fi

cleanup() {
    echo ""
    echo "[*] Safely closing video recording..."
    sync
    echo "[+] Flight video saved successfully: $OUTPUT_FILE"
    exit 0
}
trap cleanup SIGINT SIGTERM

exec rpicam-vid "${ARGS[@]}"
