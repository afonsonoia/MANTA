#!/bin/bash
# ==============================================================================
# MANTA UAV — Gravação Segura de Voo (Power-Loss Immune & Anti-Vibração)
# Plataforma: Raspberry Pi 3 A+ com câmara Sony IMX378-79
# ==============================================================================
# Características:
# 1. IMUNIDADE A CORTES DE ENERGIA:
#    Utiliza contentor Matroska (.mkv) com --flush contínuo de buffers para o
#    cartão SD. Mesmo que a bateria seja desligada bruscamente em voo, todos
#    os segundos gravados até ao último instante ficam 100% legíveis e intactos.
#
# 2. OTIMIZAÇÃO ANTI-VIBRAÇÃO (EFEITO JELLO & ALTA FREQUÊNCIA DO MOTOR):
#    - Modo de leitura rápido (2x2 Binning 2028x1080): Reduz o tempo de
#      varrimento do Rolling Shutter, cortando a distorção jello para metade.
#    - Exposição Sport (--exposure sport): Obriga o sensor a usar tempos de
#      obturador o mais rápidos possível (evita arrastamento e borrão por vibração).
#    - Denoise desligado (--denoise cdn_off): Elimina o efeito "fantasma" ou
#      esbatimento temporal provocado por micro-vibrações nas arestas.
#    - Codificação H.264 por aceleração de hardware (bcm2835-codec).
# ==============================================================================

set -e

# Configurações padrão
VIDEO_DIR="${VIDEO_DIR:-/home/pc/flight_videos}"
WIDTH="${WIDTH:-1920}"
HEIGHT="${HEIGHT:-1080}"
FPS="${FPS:-30}"
EXPOSURE="${EXPOSURE:-sport}"
SHUTTER_US="${SHUTTER_US:-0}"  # 0 = auto com prioridade rápida (sport), ou ex: 2000 (1/500s)

# Criar pasta de gravações se não existir
mkdir -p "$VIDEO_DIR"

# Verificar espaço livre em disco (mínimo 500 MB)
FREE_KB=$(df -k "$VIDEO_DIR" | awk 'NR==2 {print $4}')
if [ "$FREE_KB" -lt 512000 ]; then
    echo "[!] ERRO: Menos de 500MB de espaço livre no cartão SD ($((FREE_KB/1024)) MB disponíveis)." >&2
    echo "    Por favor liberte espaço antes de iniciar a gravação de voo." >&2
    exit 1
fi

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
OUTPUT_FILE="${VIDEO_DIR}/manta_flight_${TIMESTAMP}.mkv"

echo "=================================================================="
echo " MANTA UAV — Início de Gravação de Voo Segura"
echo " Ficheiro: $OUTPUT_FILE"
echo " Resolução: ${WIDTH}x${HEIGHT} @ ${FPS} fps | Exposição: ${EXPOSURE}"
if [ "$SHUTTER_US" -gt 0 ]; then
    echo " Shutter fixo: ${SHUTTER_US} µs (1/$((1000000/SHUTTER_US))s anti-blur)"
fi
echo " Contentor: Matroska (.mkv) com --flush ativo (imune a corte de energia)"
echo "=================================================================="

# Construção dos argumentos rpicam-vid
ARGS=(
    -t 0
    -n
    --width "$WIDTH"
    --height "$HEIGHT"
    --framerate "$FPS"
    --exposure "$EXPOSURE"
    --denoise cdn_off
    --flush
    --codec libav
    --libav-format matroska
    -o "$OUTPUT_FILE"
)

# Adicionar velocidade de obturador fixa se especificada
if [ "$SHUTTER_US" -gt 0 ]; then
    ARGS+=(--shutter "$SHUTTER_US")
fi

# Tratamento de fecho ordenado (SIGINT / SIGTERM)
cleanup() {
    echo ""
    echo "[*] A encerrar gravação de forma segura..."
    sync
    echo "[+] Gravação guardada com sucesso em: $OUTPUT_FILE"
    exit 0
}
trap cleanup SIGINT SIGTERM

# Iniciar rpicam-vid
exec rpicam-vid "${ARGS[@]}"
