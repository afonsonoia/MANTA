"""
MANTA Companion Computer — Ultra Low-Latency SSH Video Stream Receiver
Recebe e reproduz em direto no PC o fluxo de vídeo H.264 do Raspberry Pi 3 A+
"""

import subprocess
import sys
import time
import shutil
import paramiko

RPI_HOST = "manta.local"
RPI_USER = "pc"
RPI_PASS = "134679"

WIDTH = 1280
HEIGHT = 720
FPS = 30

def check_ffplay():
    ffplay_path = shutil.which("ffplay")
    if not ffplay_path:
        print("[!] Erro: ffplay não foi encontrado no PATH do Windows.")
        print("    Certifique-se de que o FFmpeg está instalado.")
        return None
    return ffplay_path

def main():
    ffplay_bin = check_ffplay()
    if not ffplay_bin:
        sys.exit(1)

    print("=" * 60)
    print(" MANTA UAV — Transmissão de Vídeo em Direto por SSH")
    print(f" Alvo: {RPI_USER}@{RPI_HOST} | Resolução: {WIDTH}x{HEIGHT} @ {FPS}fps")
    print("=" * 60)

    print(f"[*] A estabelecer ligação SSH a {RPI_HOST}...")
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
        client.connect(RPI_HOST, username=RPI_USER, password=RPI_PASS, timeout=8)
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

    # Comando remoto de captura e stream H.264
    remote_cmd = (
        f"rpicam-vid -t 0 --inline --width {WIDTH} --height {HEIGHT} "
        f"--framerate {FPS} --nopreview --codec h264 -o -"
    )

    print(f"[*] A iniciar comando remoto: {remote_cmd}")
    stdin, stdout, stderr = client.exec_command(remote_cmd, bufsize=0)

    # Comando local ffplay para latência mínima
    ffplay_cmd = [
        ffplay_bin,
        "-probesize", "32",
        "-analyzeduration", "0",
        "-fflags", "nobuffer",
        "-flags", "low_delay",
        "-framedrop",
        "-window_title", "MANTA FPV — Live Video Stream (SSH)",
        "-i", "-"
    ]

    print("[*] A abrir leitor ffplay...")
    player = subprocess.Popen(ffplay_cmd, stdin=subprocess.PIPE)

    try:
        while True:
            data = stdout.channel.recv(4096)
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
        client.close()
        print("[+] Sessão terminada com sucesso.")

if __name__ == "__main__":
    main()
