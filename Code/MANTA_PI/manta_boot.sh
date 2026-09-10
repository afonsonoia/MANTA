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

# Garantir que o Wi-Fi não está bloqueado para permitir tentativa de ligação
sudo rfkill unblock wifi 2>/dev/null || true

CONNECTED=0
ELAPSED=0

# Ciclo de verificação de ligação à rede por até 30 segundos
while [ "$ELAPSED" -lt "$WIFI_TIMEOUT_SEC" ]; do
    # Verifica se existe gateway por defeito no wlan0
    GATEWAY=$(ip route show default 2>/dev/null | awk '/default via/ {print $3; exit}')
    
    if [ -n "$GATEWAY" ]; then
        # Testa se consegue comunicar com o router ou com a internet
        if ping -c 1 -W 1 "$GATEWAY" >/dev/null 2>&1 || ping -c 1 -W 1 1.1.1.1 >/dev/null 2>&1; then
            CONNECTED=1
            break
        fi
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
    log "----------------------------------------------------------"
    log "[+] Wi-Fi DETETADO após ${ELAPSED}s!"
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
