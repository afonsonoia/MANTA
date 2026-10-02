#!/bin/bash
# ==============================================================================
# MANTA UAV — Autonomous Boot Manager
# Dual-Mode: Development Mode (Bench) vs Flight Mode (Field)
# ==============================================================================
# Behavior:
# 1. During the first 30 seconds after boot, attempts to connect to Wi-Fi.
# 2. IF WI-FI CONNECTED (< 30s):
#    - Enters DEVELOPMENT / BENCH MODE.
#    - Runs 'git pull' to update local repository to latest code.
#    - Keeps Wi-Fi and SSH active for user access.
#    - Does NOT start video recording (camera left free).
# 3. IF NO WI-FI (30s TIMEOUT):
#    - Enters FLIGHT / AUTONOMOUS MODE.
#    - Disables wireless communications (Wi-Fi and Bluetooth) to eliminate
#      2.4 GHz RF interference with RC receiver and LoRa.
#    - Disables HDMI output circuitry to save battery power (~30mA).
#    - Automatically launches safe, anti-vibration flight recording (.mkv).
# ==============================================================================

set -u

WIFI_TIMEOUT_SEC=30
REPO_DIR="/home/pc/MANTA"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_PREFIX="[MANTA-BOOT]"

log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') $LOG_PREFIX $1"
}

log "=========================================================="
log "Starting MANTA UAV Autonomous Boot Manager"
log "Wi-Fi detection window: ${WIFI_TIMEOUT_SEC} seconds"
log "=========================================================="

# Multi-layer Wi-Fi subsystem unblock on startup (prevents persistent rfkill lockouts)
log "[*] Unblocking Wi-Fi radio subsystem..."
sudo rfkill unblock wifi 2>/dev/null || rfkill unblock wifi 2>/dev/null || true
sudo rfkill unblock all 2>/dev/null || rfkill unblock all 2>/dev/null || true

if command -v nmcli >/dev/null 2>&1; then
    sudo nmcli radio wifi on 2>/dev/null || nmcli radio wifi on 2>/dev/null || true
    sudo nmcli networking on 2>/dev/null || nmcli networking on 2>/dev/null || true
fi

for iface in $(ip -o link show 2>/dev/null | awk -F': ' '{print $2}' | grep -E '^wl'); do
    sudo ip link set "$iface" up 2>/dev/null || ip link set "$iface" up 2>/dev/null || true
done

if command -v nmcli >/dev/null 2>&1; then
    sudo nmcli device connect wlan0 2>/dev/null || true
    sudo nmcli device wifi rescan 2>/dev/null || true
elif command -v wpa_cli >/dev/null 2>&1; then
    sudo wpa_cli -i wlan0 reassociate 2>/dev/null || true
fi

sleep 2

check_wifi_connection() {
    # 1. NetworkManager status
    if command -v nmcli >/dev/null 2>&1; then
        if nmcli -t -f TYPE,STATE dev 2>/dev/null | grep -qE '^wifi:connected'; then
            return 0
        fi
    fi

    # 2. Assigned IPv4 on wireless interface
    local IP_WL
    IP_WL=$(ip -4 -o addr show 2>/dev/null | awk '$2 ~ /^wl/ {split($4, a, "/"); print a[1]}' | grep -vE '^(127\.|169\.254\.)' | head -n 1)
    if [ -n "$IP_WL" ]; then
        return 0
    fi

    # 3. Gateway / DNS ping check
    local GATEWAY
    GATEWAY=$(ip route show default 2>/dev/null | awk '/default via/ {print $3; exit}')
    if [ -n "$GATEWAY" ]; then
        if ping -c 1 -W 1 "$GATEWAY" >/dev/null 2>&1 || ping -c 1 -W 1 1.1.1.1 >/dev/null 2>&1 || ping -c 1 -W 1 8.8.8.8 >/dev/null 2>&1; then
            return 0
        fi
    fi

    return 1
}

CONNECTED=0
ELAPSED=0

while [ "$ELAPSED" -lt "$WIFI_TIMEOUT_SEC" ]; do
    if check_wifi_connection; then
        CONNECTED=1
        break
    fi

    sleep 1
    ELAPSED=$((ELAPSED + 1))
    
    if [ $((ELAPSED % 5)) -eq 0 ]; then
        log "Waiting for Wi-Fi connection... (${ELAPSED}/${WIFI_TIMEOUT_SEC}s)"
    fi
done

if [ "$CONNECTED" -eq 1 ]; then
    # Scenario A: Development / Bench Mode
    WIFI_IP=$(ip -4 -o addr show 2>/dev/null | awk '$2 ~ /^wl/ {split($4, a, "/"); print a[1]}' | head -n 1)
    log "----------------------------------------------------------"
    log "[+] Wi-Fi DETECTED after ${ELAPSED}s! IP: ${WIFI_IP:-N/A}"
    log "[+] ENTERING DEVELOPMENT / BENCH MODE"
    log "----------------------------------------------------------"
    
    if [ -d "$REPO_DIR" ]; then
        log "Updating local repository at $REPO_DIR..."
        cd "$REPO_DIR" || exit 1
        
        CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "untested")
        log "Active branch: $CURRENT_BRANCH"
        
        if git pull origin "$CURRENT_BRANCH"; then
            log "[+] Repository updated successfully!"
        else
            log "[!] Warning: git pull failed (external internet may be unreachable)."
        fi
    else
        log "[!] Repository directory not found at $REPO_DIR."
    fi

    log "[+] Raspberry Pi ready for development and testing via SSH."
    log "[+] Camera remains available and radios stay active."
    exit 0

else
    # Scenario B: Flight / Autonomous Mode
    log "----------------------------------------------------------"
    log "[!] TIMEOUT of ${WIFI_TIMEOUT_SEC}s reached without Wi-Fi connection."
    log "[!] ENTERING FLIGHT / FIELD MODE"
    log "----------------------------------------------------------"

    log "1. Disabling wireless radios (Wi-Fi and Bluetooth) for flight mode..."
    sudo rfkill block wifi 2>/dev/null || true
    sudo rfkill block bluetooth 2>/dev/null || true
    sudo systemctl stop bluetooth 2>/dev/null || true

    log "2. Disabling HDMI video circuitry to save battery..."
    sudo vcgencmd display_power 0 2>/dev/null || true

    log "3. Starting continuous flight recording (Matroska .mkv, anti-vibration & power-loss immune)..."
    
    RECORD_SCRIPT="${SCRIPT_DIR}/record_flight.sh"
    if [ -f "$RECORD_SCRIPT" ]; then
        chmod +x "$RECORD_SCRIPT"
        exec /bin/bash "$RECORD_SCRIPT"
    else
        log "[!] CRITICAL ERROR: Script $RECORD_SCRIPT not found!"
        exit 1
    fi
fi
