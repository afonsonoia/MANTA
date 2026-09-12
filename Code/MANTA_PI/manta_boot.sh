#!/bin/bash
# ==============================================================================
# MANTA UAV — Gestor Autónomo de Arranque (Boot Manager)
# Dual-Mode: Modo Programação (Bancada) vs Modo Voo (Campo)
# ==============================================================================
# Comportamento:
# 1. Durante os primeiros 30 segundos após o boot, tenta obter ligação Wi-Fi.
# 2. SE CONECTAR AO WI-FI (< 30s):
#    - Entra em MODO PROGRAMAÇÃO / BANCADA.
#    - Executa 'git pull' para atualizar o repositório com o código mais recente.
#    - Mantém o Wi-Fi e o SSH ativos para trabalho do utilizador.
#    - NÃO inicia gravação de vídeo (deixa a câmara livre).
# 3. SE NÃO CONECTAR AO WI-FI (TIMEOUT 30s):
#    - Entra em MODO VOO / AUTÓNOMO.
#    - Desliga todas as comunicações sem fios (Wi-Fi e Bluetooth) para eliminar
#      interferências de RF nos 2.4 GHz com o rádio de controlo RC e LoRa.
#    - Desliga o circuito HDMI para poupar energia (~30mA).
#    - Inicia automaticamente a gravação de voo segura e anti-vibração (.mkv).
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
log "A iniciar Gestor Autónomo de Arranque MANTA UAV"
log "Janela de deteção de Wi-Fi: ${WIFI_TIMEOUT_SEC} segundos"
log "=========================================================="

# ------------------------------------------------------------------------------
# GARANTIA DE REATIVAÇÃO DE WI-FI NO ARRANQUE (Prevenção de Ciclo Infinito)
# ------------------------------------------------------------------------------
# Se no boot anterior o Pi entrou em Modo Voo (executou rfkill block wifi/all),
# o kernel, o systemd-rfkill e o NetworkManager guardam esse estado desligado.
# Aqui forçamos a reativação ativa e profunda em todas as camadas do sistema:
log "[*] A reativar e desbloquear subsistema de rádio Wi-Fi..."

# 1. Desbloquear rfkill no kernel/driver
sudo rfkill unblock wifi 2>/dev/null || rfkill unblock wifi 2>/dev/null || true
sudo rfkill unblock all 2>/dev/null || rfkill unblock all 2>/dev/null || true

# 2. Reativar rádio e networking no NetworkManager (Raspberry Pi OS Bookworm)
if command -v nmcli >/dev/null 2>&1; then
    sudo nmcli radio wifi on 2>/dev/null || nmcli radio wifi on 2>/dev/null || true
    sudo nmcli networking on 2>/dev/null || nmcli networking on 2>/dev/null || true
fi

# 3. Forçar todas as interfaces wireless (wlan0, etc.) para estado UP
for iface in $(ip -o link show 2>/dev/null | awk -F': ' '{print $2}' | grep -E '^wl'); do
    sudo ip link set "$iface" up 2>/dev/null || ip link set "$iface" up 2>/dev/null || true
done

# 4. Disparar pedido ativo de varrimento de redes para associação imediata
if command -v nmcli >/dev/null 2>&1; then
    sudo nmcli device wifi rescan 2>/dev/null || nmcli device wifi rescan 2>/dev/null || true
elif command -v wpa_cli >/dev/null 2>&1; then
    sudo wpa_cli -i wlan0 reassociate 2>/dev/null || wpa_cli -i wlan0 reassociate 2>/dev/null || true
fi

# Pequena pausa para o firmware e rádio estabilizarem após o unblock
sleep 2

# Função inteligente de deteção de conectividade Wi-Fi (sem falsos negativos)
check_wifi_connection() {
    # 1. Validação via NetworkManager
    if command -v nmcli >/dev/null 2>&1; then
        if nmcli -t -f TYPE,STATE dev 2>/dev/null | grep -qE '^wifi:connected'; then
            return 0
        fi
    fi

    # 2. Validação por IPv4 atribuído na interface wireless (fora de 127.x e 169.254.x)
    local IP_WL
    IP_WL=$(ip -4 -o addr show 2>/dev/null | awk '$2 ~ /^wl/ {split($4, a, "/"); print a[1]}' | grep -vE '^(127\.|169\.254\.)' | head -n 1)
    if [ -n "$IP_WL" ]; then
        return 0
    fi

    # 3. Validação clássica via ping ao Gateway ou DNS externo
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

# Ciclo de verificação de ligação à rede por até 30 segundos
while [ "$ELAPSED" -lt "$WIFI_TIMEOUT_SEC" ]; do
    if check_wifi_connection; then
        CONNECTED=1
        break
    fi

    sleep 1
    ELAPSED=$((ELAPSED + 1))
    
    # Notificação periódica a cada 5 segundos
    if [ $((ELAPSED % 5)) -eq 0 ]; then
        log "A aguardar ligação Wi-Fi... (${ELAPSED}/${WIFI_TIMEOUT_SEC}s)"
    fi
done

# ==============================================================================
# DECISÃO DE MODO
# ==============================================================================

if [ "$CONNECTED" -eq 1 ]; then
    # --------------------------------------------------------------------------
    # CENÁRIO A: MODO PROGRAMAÇÃO / BANCADA
    # --------------------------------------------------------------------------
    WIFI_IP=$(ip -4 -o addr show 2>/dev/null | awk '$2 ~ /^wl/ {split($4, a, "/"); print a[1]}' | head -n 1)
    log "----------------------------------------------------------"
    log "[+] Wi-Fi DETETADO após ${ELAPSED}s! IP: ${WIFI_IP:-N/A}"
    log "[+] A ENTRAR EM MODO PROGRAMAÇÃO / BANCADA"
    log "----------------------------------------------------------"
    
    if [ -d "$REPO_DIR" ]; then
        log "A atualizar repositório local em $REPO_DIR..."
        cd "$REPO_DIR" || exit 1
        
        CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "untested")
        log "Ramo ativo: $CURRENT_BRANCH"
        
        # Executa git pull da versão mais recente
        if git pull origin "$CURRENT_BRANCH"; then
            log "[+] Repositório atualizado com sucesso!"
        else
            log "[!] Aviso: Falha no git pull (possível falta de internet externa)."
        fi
    else
        log "[!] Diretório do repositório não encontrado em $REPO_DIR."
    fi

    log "[+] Raspberry Pi pronto para programação e testes via SSH."
    log "[+] A câmara permanece livre e os rádios ligados."
    exit 0

else
    # --------------------------------------------------------------------------
    # CENÁRIO B: MODO VOO / AUTÓNOMO
    # --------------------------------------------------------------------------
    log "----------------------------------------------------------"
    log "[!] TIMEOUT de ${WIFI_TIMEOUT_SEC}s atingido sem ligação Wi-Fi."
    log "[!] A ENTRAR EM MODO VOO / CAMPO"
    log "----------------------------------------------------------"

    log "1. A desativar emissões de rádio (Wi-Fi e Bluetooth) para eliminar interferências de RF..."
    sudo rfkill block wifi 2>/dev/null || true
    sudo rfkill block bluetooth 2>/dev/null || true
    sudo systemctl stop bluetooth 2>/dev/null || true

    log "2. A desativar saída de vídeo HDMI para poupança de bateria..."
    sudo vcgencmd display_power 0 2>/dev/null || true

    log "3. A iniciar gravação contínua de voo (Matroska .mkv, anti-vibração e seguro contra quebra de energia)..."
    
    RECORD_SCRIPT="${SCRIPT_DIR}/record_flight.sh"
    if [ -f "$RECORD_SCRIPT" ]; then
        chmod +x "$RECORD_SCRIPT"
        # Substitui o processo atual pelo script de gravação para monitorização pelo systemd
        exec /bin/bash "$RECORD_SCRIPT"
    else
        log "[!] ERRO CRÍTICO: Script $RECORD_SCRIPT não encontrado!"
        exit 1
    fi
fi
