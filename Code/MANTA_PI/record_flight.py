"""
MANTA Companion Computer — Gravação Segura de Voo (Python Controller)
Permite iniciar e monitorizar a gravação à prova de quebras de energia com
otimizações anti-vibração para o sensor Sony IMX378-79.
"""

import os
import sys
import time
import signal
import shutil
import argparse
import subprocess
from datetime import datetime

DEFAULT_OUTPUT_DIR = "/home/pc/flight_videos"

def get_free_disk_mb(path):
    stat = shutil.disk_usage(path)
    return stat.free // (1024 * 1024)

def parse_args():
    parser = argparse.ArgumentParser(description="MANTA Safe Video Recorder")
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR, help="Diretório onde guardar os vídeos")
    parser.add_argument("--width", type=int, default=1920, help="Largura do vídeo (padrão: 1920)")
    parser.add_argument("--height", type=int, default=1080, help="Altura do vídeo (padrão: 1080)")
    parser.add_argument("--fps", type=int, default=30, help="Taxa de fotogramas por segundo (padrão: 30)")
    parser.add_argument("--exposure", default="sport", choices=["normal", "sport"],
                        help="Modo de exposição (sport = prioridade obturador rápido anti-vibração)")
    parser.add_argument("--shutter", type=int, default=0,
                        help="Velocidade de obturador fixa em microssegundos (ex: 2000 = 1/500s; 0 = auto)")
    parser.add_argument("--duration", type=int, default=0,
                        help="Duração em segundos (0 = infinito até parar ou desligar energia)")
    return parser.parse_args()

def main():
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    free_mb = get_free_disk_mb(args.output_dir)
    if free_mb < 500:
        print(f"[!] ERRO: Apenas {free_mb} MB livres em {args.output_dir}. Mínimo 500 MB.")
        sys.exit(1)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_filename = os.path.join(args.output_dir, f"manta_flight_{timestamp}.mkv")

    cmd = [
        "rpicam-vid",
        "-t", str(args.duration * 1000 if args.duration > 0 else 0),
        "-n",
        "--width", str(args.width),
        "--height", str(args.height),
        "--framerate", str(args.fps),
        "--bitrate", "15000000",
        "--profile", "high",
        "--exposure", args.exposure,
        "--denoise", "cdn_fast",
        "--autofocus-mode", "manual",
        "--lens-position", "0.0",
        "--flush",
        "--codec", "libav",
        "--libav-format", "matroska",
        "--vflip",
        "--hflip",
        "-o", out_filename
    ]

    if args.shutter > 0:
        cmd.extend(["--shutter", str(args.shutter)])

    print("=" * 65)
    print(" MANTA UAV — Gravação Segura de Voo (Anti-Vibração & Power-Loss Safe)")
    print(f" Destino: {out_filename}")
    print(f" Resolução: {args.width}x{args.height} @ {args.fps}fps | Exposição: {args.exposure}")
    if args.shutter > 0:
        print(f" Shutter fixo: {args.shutter} µs (1/{1_000_000 // args.shutter}s)")
    print(f" Espaço em disco disponível: {free_mb} MB")
    print(" Formato: Matroska (.mkv) com flush síncrono para imunidade a corte")
    print("=" * 65)

    proc = subprocess.Popen(cmd)

    def sig_handler(signum, frame):
        print("\n[*] Sinal de encerramento recebido. A fechar gravação...")
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
        subprocess.run(["sync"])
        print(f"[+] Vídeo gravado com sucesso: {out_filename}")
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    try:
        proc.wait()
    except KeyboardInterrupt:
        sig_handler(None, None)

    subprocess.run(["sync"])
    print(f"[+] Gravação terminada: {out_filename}")

if __name__ == "__main__":
    main()
