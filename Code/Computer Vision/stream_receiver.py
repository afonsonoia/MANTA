"""
MANTA Companion Computer — Ultra Low-Latency SSH Video Stream Receiver
Recebe e reproduz em direto no PC o fluxo de vídeo H.264 do Raspberry Pi 3 A+
Permite inverter a imagem verticalmente (V-Flip) e horizontalmente (H-Flip).
"""

import os
import sys
import time
import json
import shutil
import argparse
import subprocess
import paramiko

RPI_HOST = "manta.local"
RPI_USER = "pc"
RPI_PASS = "134679"

WIDTH = 1280
HEIGHT = 720
FPS = 30

CONFIG_FILE = os.path.join(os.path.dirname(__file__), "fpv_config.json")

def load_fpv_config():
    """Carrega as opções de orientação guardadas ou devolve predefinição."""
    cfg = {"vflip": True, "hflip": True}
    if os.path.isfile(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                saved = json.load(f)
                cfg.update(saved)
        except Exception:
            pass
    return cfg

def check_ffplay():
    ffplay_path = shutil.which("ffplay")
    if not ffplay_path:
        print("[!] Erro: ffplay não foi encontrado no PATH do Windows.")
        print("    Certifique-se de que o FFmpeg está instalado.")
        return None
    return ffplay_path

def parse_args():
    cfg = load_fpv_config()
    parser = argparse.ArgumentParser(description="MANTA UAV — Live FPV Video Stream Receiver")
    parser.add_argument("--host", default=RPI_HOST, help="Endereço IP ou hostname do Raspberry Pi")
    parser.add_argument("--width", type=int, default=WIDTH, help="Largura do vídeo")
    parser.add_argument("--height", type=int, default=HEIGHT, help="Altura do vídeo")
    parser.add_argument("--fps", type=int, default=FPS, help="Taxa de fotogramas por segundo")

    # Inversão de imagem (vertical e horizontal)
    parser.add_argument("--vflip", dest="vflip", action="store_true", default=None,
                        help="Ativar inversão vertical (V-Flip)")
    parser.add_argument("--no-vflip", dest="vflip", action="store_false",
                        help="Desativar inversão vertical")
    parser.add_argument("--hflip", dest="hflip", action="store_true", default=None,
                        help="Ativar inversão horizontal (H-Flip)")
    parser.add_argument("--no-hflip", dest="hflip", action="store_false",
                        help="Desativar inversão horizontal")

    args = parser.parse_args()

    # Preencher com configuração guardada caso o argumento não tenha sido especificado
    if args.vflip is None:
        args.vflip = cfg.get("vflip", True)
    if args.hflip is None:
        args.hflip = cfg.get("hflip", True)

    return args

def main():
    args = parse_args()

    ffplay_bin = check_ffplay()
    if not ffplay_bin:
        sys.exit(1)

    target_host = args.host

    flip_tags = []
    if args.vflip:
        flip_tags.append("V-Flip")
    if args.hflip:
        flip_tags.append("H-Flip")
    orient_str = f" [{', '.join(flip_tags)}]" if flip_tags else " [Normal]"

    print("=" * 60)
    print(" MANTA UAV — Transmissão de Vídeo em Direto por SSH")
    print(f" Alvo: {RPI_USER}@{target_host} | Resolução: {args.width}x{args.height} @ {args.fps}fps")
    print(f" Orientação de Imagem:{orient_str}")
    print("=" * 60)

    print(f"[*] A estabelecer ligação SSH a {target_host}...")
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    try:
        client.connect(target_host, username=RPI_USER, password=RPI_PASS, timeout=8)
    except Exception as e:
        print(f"[!] Erro ao ligar ao Raspberry Pi: {e}")
        print("    Verifique se o Raspberry Pi já arrancou e está conectado ao Wi-Fi.")
        sys.exit(1)

    print("[+] Ligação SSH estabelecida com sucesso!")

    # Verificar se a câmara é detetada
    stdin, stdout, stderr = client.exec_command("rpicam-hello --list-cameras")
    cam_out = stdout.read().decode('utf-8')
    print("[*] Verificação de hardware da câmara:")
    print(cam_out.strip() if cam_out.strip() else "    (Nenhuma câmara listada automaticamente. Se for módulo de terceiros, pode necessitar de dtoverlay).")

    # Construir argumentos de inversão por hardware no sensor
    flip_flags = []
    if args.vflip:
        flip_flags.append("--vflip")
    if args.hflip:
        flip_flags.append("--hflip")
    flip_cmd_str = f" {' '.join(flip_flags)}" if flip_flags else ""

    # Comando remoto de captura e stream H.264 Ultra Low-Latency:
    # 1. --flush: descarrega imediatamente os buffers de vídeo sem acumulação em stdout.
    # 2. --profile baseline: elimina B-frames e reordenação temporal (zero latência de descodificação).
    # 3. --intra 10: I-frame a cada 330ms para sincronização imediata do descodificador.
    # 4. --bitrate 3000000: limita a 3 Mbps para eliminar bufferbloat na rede Wi-Fi.
    # 5. Inversão opcional de hardware: --vflip e/ou --hflip
    remote_cmd = (
        f"rpicam-vid -t 0 --inline --flush --profile baseline --intra 10 "
        f"--width {args.width} --height {args.height} --framerate {args.fps} --bitrate 3000000 "
        f"--exposure sport --denoise cdn_off --autofocus-mode manual --lens-position 0.0 "
        f"--nopreview --codec h264{flip_cmd_str} -o -"
    )

    print(f"[*] A iniciar comando remoto: {remote_cmd}")
    stdin, stdout, stderr = client.exec_command(remote_cmd, bufsize=0)

    # Comando local ffplay para latência mínima em tempo real (tempo de resposta < 100-150ms)
    ffplay_cmd = [
        ffplay_bin,
        "-probesize", "32",
        "-analyzeduration", "0",
        "-sync", "ext",              # Sincroniza com relógio do sistema (elimina espera por áudio)
        "-fflags", "nobuffer",       # Desativa buffer interno do ffplay
        "-flags", "low_delay",       # Força modo de latência ultrabaixa no codec
        "-framedrop",                # Descarta fotogramas atrasados para nunca acumular delay
        "-strict", "experimental",
        "-avioflags", "direct",
        "-window_title", f"MANTA FPV — Live Video Stream{orient_str}",
        "-i", "-"
    ]

    print("[*] A abrir leitor ffplay...")
    player = subprocess.Popen(ffplay_cmd, stdin=subprocess.PIPE)

    channel = stdout.channel
    channel.setblocking(True)

    try:
        while True:
            data = channel.recv(32768)
            if not data:
                break
            try:
                player.stdin.write(data)
                player.stdin.flush()
            except (BrokenPipeError, OSError):
                break
    except KeyboardInterrupt:
        print("\n[*] Transmissão interrompida pelo utilizador.")
    finally:
        print("[*] A encerrar processos...")
        try:
            player.stdin.close()
        except Exception:
            pass
        player.terminate()
        try:
            client.exec_command("pkill -SIGINT -f 'rpicam-vid.*baseline'")
        except Exception:
            pass
        client.close()
        print("[+] Sessão terminada com sucesso.")

if __name__ == "__main__":
    main()
