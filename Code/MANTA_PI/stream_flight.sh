#!/bin/bash
# ==============================================================================
# MANTA UAV — Transmissão de Vídeo em Direto (Live FPV Stream)
# ==============================================================================

WIDTH="${1:-1280}"
HEIGHT="${2:-720}"
FPS="${3:-30}"

echo "[*] A iniciar transmissão rpicam-vid: ${WIDTH}x${HEIGHT} @ ${FPS}fps..." >&2

exec rpicam-vid \
    -t 0 \
    --inline \
    --flush \
    --profile baseline \
    --intra 10 \
    --bitrate 3000000 \
    --width "$WIDTH" \
    --height "$HEIGHT" \
    --framerate "$FPS" \
    --exposure sport \
    --denoise cdn_off \
    --autofocus-mode manual \
    --lens-position 0.0 \
    --vflip \
    --hflip \
    -n \
    -o -
