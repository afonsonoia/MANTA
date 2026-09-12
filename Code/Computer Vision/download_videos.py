#!/usr/bin/env python3
"""
MANTA UAV — Script Resiliente de Descarregamento de Vídeos do Raspberry Pi
==========================================================================
Características:
1. Deteção & Espera Automática: Se o Pi estiver a arrancar ou temporariamente
   offline, aguarda ativamente até que fique acessível por ping/SSH.
2. Transferência Resiliente (Chunked Streaming): NÃO usa prefetch(), evitando
   sobrecarregar a RAM (512MB) do RPi 3 A+ e o chip de Wi-Fi 2.4GHz.
3. Retoma Automática (Resume / Partial Download): Se a ligação cair a meio
   de um vídeo de 1GB aos 85%, reconecta e retoma exatamente nos 85% sem
   perder nada do que já foi transferido.
4. Auto-Reconexão com Backoff: Se o Wi-Fi cair, tenta reconectar até 10 vezes
   e prossegue a partir do byte exato.
5. Verificação de Integridade: Confirma o tamanho final de cada vídeo.
6. Conversão Automática para MP4: Converte vídeos .mkv para .mp4 de forma
   sem perdas (lossless remux) imediatamente após o descarregamento.
7. Limpeza Segura do .mkv Local: Apaga o .mkv local após validação do .mp4
   para libertar espaço em disco. Os vídeos no Raspberry Pi NUNCA são apagados
   automaticamente (o utilizador apaga manualmente se/quando quiser).
"""

import os
import sys
import io
import time
import socket
import paramiko
from datetime import datetime

try:
    if sys.platform == "win32":
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
    else:
        sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

# Configurações de Conexão
DEFAULT_HOSTS = ["10.32.198.35", "manta.local"]
RPI_USER = "pc"
RPI_PASS = "134679"
REMOTE_VIDEO_DIR = "/home/pc/flight_videos"
LOCAL_DEST_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "flight_videos")
CHUNK_SIZE = 256 * 1024  # 256 KB chunk (estável e rápido)
MAX_RETRIES_PER_FILE = 20
RETRY_DELAY_SEC = 2


from concurrent.futures import ThreadPoolExecutor
import argparse

try:
    from convert_videos import convert_mkv_to_mp4, is_ffmpeg_available
except ImportError:
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from convert_videos import convert_mkv_to_mp4, is_ffmpeg_available


def test_ssh_auth(ip):
    """Testa se as credenciais do Raspberry Pi funcionam no IP."""
    try:
        c = paramiko.SSHClient()
        c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        c.connect(ip, username=RPI_USER, password=RPI_PASS, timeout=2.0, banner_timeout=4)
        c.close()
        return True
    except Exception:
        return False


def check_ip_candidate(ip):
    """Verifica porta 22 e autenticação rápida."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.3)
        res = s.connect_ex((ip, 22))
        s.close()
        if res == 0:
            if test_ssh_auth(ip):
                return ip
    except Exception:
        pass
    return None


def resolve_host():
    """Tenta resolver e contactar o IP do Raspberry Pi."""
    for host in DEFAULT_HOSTS:
        try:
            ip = socket.gethostbyname(host)
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(0.8)
            res = s.connect_ex((ip, 22))
            s.close()
            if res == 0 and test_ssh_auth(ip):
                return ip
        except Exception:
            continue

    # Varrimento rápido da subnet caso o DHCP tenha atribuído outro IP
    try:
        with ThreadPoolExecutor(max_workers=50) as ex:
            ips = [f"10.32.198.{i}" for i in range(1, 255)]
            found = list(filter(None, ex.map(check_ip_candidate, ips)))
            if found:
                return found[0]
    except Exception:
        pass

    return None


def wait_for_pi(max_wait_sec=600):
    """Aguarda ativamente até que o Raspberry Pi fique online na rede (até 10 minutos)."""
    print("[*] À procura do Raspberry Pi na rede...")
    start_time = time.time()
    last_print = 0
    while time.time() - start_time < max_wait_sec:
        host = resolve_host()
        if host:
            print(f"[+] Raspberry Pi encontrado e acessível em: {host}!")
            return host
        
        now = time.time()
        if now - last_print >= 5:
            elapsed = int(now - start_time)
            print(f"    A aguardar ligação Wi-Fi do Raspberry Pi... ({elapsed}s decorridos)")
            last_print = now
        time.sleep(1.5)

    return None


def create_sftp_client(host):
    """Cria cliente SSH/SFTP com Keep-Alive ativo para prevenir timeouts."""
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(
        host,
        username=RPI_USER,
        password=RPI_PASS,
        timeout=10,
        banner_timeout=15,
        auth_timeout=10
    )
    transport = ssh.get_transport()
    if transport:
        transport.set_keepalive(5)  # Envia pacote keepalive a cada 5 segundos
    sftp = ssh.open_sftp()
    return ssh, sftp


def format_bytes(b):
    """Converte bytes em string legível (KB, MB, GB)."""
    if b < 1024:
        return f"{b} B"
    elif b < 1024 * 1024:
        return f"{b / 1024:.1f} KB"
    elif b < 1024 * 1024 * 1024:
        return f"{b / (1024 * 1024):.1f} MB"
    else:
        return f"{b / (1024 * 1024 * 1024):.2f} GB"


def download_single_file(host_getter, remote_path, local_path, file_size):
    """
    Descarrega um ficheiro remoto com suporte para retoma (resume) e auto-reconexão.
    """
    retries = 0
    os.makedirs(os.path.dirname(local_path), exist_ok=True)

    while retries < MAX_RETRIES_PER_FILE:
        # Verificar quantos bytes já existem localmente
        existing_bytes = 0
        if os.path.exists(local_path):
            existing_bytes = os.path.getsize(local_path)
            if existing_bytes == file_size and file_size > 0:
                print(f"    [OK] Ficheiro já completo localmente ({format_bytes(file_size)}). Ignorado.")
                return True
            elif existing_bytes > file_size:
                # Ficheiro local maior que o remoto, recomeçar
                os.remove(local_path)
                existing_bytes = 0

        host = host_getter()
        if not host:
            print(f"    [!] Pi temporariamente inacessível. Nova tentativa em {RETRY_DELAY_SEC}s...")
            time.sleep(RETRY_DELAY_SEC)
            retries += 1
            continue

        ssh = None
        sftp = None
        try:
            ssh, sftp = create_sftp_client(host)
            
            with sftp.open(remote_path, 'rb') as rfile:
                # Não chamamos rfile.prefetch()! O prefetch causa memory leak e timeouts no RPi.
                if existing_bytes > 0:
                    rfile.seek(existing_bytes)
                    print(f"    [+] A retomar download a partir de {format_bytes(existing_bytes)} / {format_bytes(file_size)} ({int(existing_bytes/file_size*100)}%)...")
                    mode = 'ab'
                else:
                    mode = 'wb'

                transferred = existing_bytes
                start_time = time.time()
                last_ui_time = 0
                bytes_since_start = 0

                with open(local_path, mode) as lfile:
                    while transferred < file_size:
                        to_read = min(CHUNK_SIZE, file_size - transferred)
                        chunk = rfile.read(to_read)
                        if not chunk:
                            break
                        lfile.write(chunk)
                        transferred += len(chunk)
                        bytes_since_start += len(chunk)

                        now = time.time()
                        if now - last_ui_time >= 0.3 or transferred >= file_size:
                            elapsed = now - start_time
                            speed = bytes_since_start / elapsed if elapsed > 0 else 0
                            pct = (transferred / file_size) * 100 if file_size > 0 else 100
                            rem_bytes = file_size - transferred
                            rem_sec = int(rem_bytes / speed) if speed > 0 else 0
                            rem_str = f"{rem_sec}s" if rem_sec < 60 else f"{rem_sec//60}m{rem_sec%60:02d}s"
                            
                            bar_len = 25
                            filled = int(bar_len * pct / 100)
                            bar = '=' * filled + '-' * (bar_len - filled)
                            
                            sys.stdout.write(
                                f"\r    [{bar}] {pct:5.1f}% | {format_bytes(transferred)}/{format_bytes(file_size)} | "
                                f"{format_bytes(speed)}/s | Resta: {rem_str}   "
                            )
                            sys.stdout.flush()
                            last_ui_time = now

                print()
                if transferred >= file_size:
                    print(f"    [+] Download concluído com sucesso: {os.path.basename(local_path)}")
                    return True
                else:
                    print(f"    [!] Transferência incompleta ({transferred}/{file_size} bytes). A reconectar para retomar...")

        except Exception as e:
            print(f"\n    [!] Desconexão durante a transferência: {e}")
            retries += 1
            print(f"    [*] A aguardar reconexão (tentativa {retries}/{MAX_RETRIES_PER_FILE})...")
            time.sleep(RETRY_DELAY_SEC)
        finally:
            if sftp:
                try:
                    sftp.close()
                except Exception:
                    pass
            if ssh:
                try:
                    ssh.close()
                except Exception:
                    pass

    print(f"    [!] ERRO: Esgotadas as tentativas para descarregar {os.path.basename(remote_path)}")
    return False


def main():
    parser = argparse.ArgumentParser(description="MANTA UAV — Descarregamento Seguro e Conversão de Vídeos de Voo")
    parser.add_argument("--no-convert", action="store_true", help="Não converter automaticamente de .mkv para .mp4")
    parser.add_argument("--keep-mkv", action="store_true", help="Manter o ficheiro .mkv local após a conversão para .mp4")
    parser.add_argument("--dest", default=LOCAL_DEST_DIR, help=f"Diretório local de destino (padrão: {LOCAL_DEST_DIR})")
    args = parser.parse_args()

    dest_dir = os.path.abspath(args.dest)
    os.makedirs(dest_dir, exist_ok=True)

    print("=" * 70)
    print(" MANTA UAV — Descarregamento Seguro e Resiliente de Vídeos de Voo")
    print("=" * 70)
    print(f" Pasta de destino no PC: {dest_dir}")
    if not args.no_convert:
        if is_ffmpeg_available():
            print(" Conversão automática MP4: ATIVA (Lossless Remuxing)")
            print(f" Gestão de ficheiros locais: .mkv local será {'MANTIDO' if args.keep_mkv else 'APAGADO após validação do MP4'}")
            print(" Gravações no Raspberry Pi: INTOCADAS (nunca são apagadas remotamente)")
        else:
            print(" [!] Aviso: ffmpeg não encontrado no sistema. Conversão para MP4 indisponível.")

    # 1. Esperar pelo Raspberry Pi
    host = wait_for_pi(max_wait_sec=600)
    if not host:
        print("[!] Não foi possível encontrar o Raspberry Pi na rede após 10 minutos.")
        print("    Certifique-se de que:")
        print("    1. O Raspberry Pi tem alimentação ligada.")
        print("    2. O router/hotspot 'DELTA' está ativo.")
        print("    3. Se o Pi entrou em Modo Voo (LED Wi-Fi apagado), reinicie a alimentação.")
        return 1

    # 2. Listar ficheiros de vídeo remotos
    print(f"[*] A obter lista de gravações em {REMOTE_VIDEO_DIR}...")
    try:
        ssh, sftp = create_sftp_client(host)
    except Exception as e:
        print(f"[!] Erro ao ligar ao SSH: {e}")
        return 1

    try:
        remote_files = sftp.listdir_attr(REMOTE_VIDEO_DIR)
    except Exception as e:
        print(f"[!] Erro ao aceder à pasta {REMOTE_VIDEO_DIR}: {e}")
        sftp.close()
        ssh.close()
        return 1

    # Filtrar ficheiros de vídeo (.mkv, .mp4, .h264)
    video_files = []
    for f in remote_files:
        name = f.filename
        if name.lower().endswith(('.mkv', '.mp4', '.h264')):
            video_files.append((name, f.st_size))

    sftp.close()
    ssh.close()

    if not video_files:
        print(f"[+] Nenhum ficheiro de vídeo encontrado em {REMOTE_VIDEO_DIR}.")
        return 0

    total_size = sum(sz for _, sz in video_files)
    print(f"[+] Encontrados {len(video_files)} vídeos (Total: {format_bytes(total_size)}):")
    for idx, (vname, sz) in enumerate(video_files, 1):
        print(f"    {idx}. {vname} ({format_bytes(sz)})")

    print("-" * 70)
    print("[*] A iniciar transferência com proteção anti-queda e retoma contínua...")

    def get_active_host():
        h = resolve_host()
        if not h:
            h = wait_for_pi(max_wait_sec=45)
        return h

    success_count = 0
    for idx, (vname, sz) in enumerate(video_files, 1):
        remote_file_path = f"{REMOTE_VIDEO_DIR}/{vname}"
        local_file_path = os.path.join(dest_dir, vname)
        base_name, ext = os.path.splitext(vname)

        # Se for um .mkv e a conversão estiver ativa, verificar se já temos o .mp4 local correspondente
        if ext.lower() == ".mkv" and not args.no_convert:
            local_mp4_path = os.path.join(dest_dir, f"{base_name}.mp4")
            if os.path.exists(local_mp4_path) and os.path.getsize(local_mp4_path) > 1024:
                print(f"\n[{idx}/{len(video_files)}] [OK] {vname} já descarregado e convertido (.mp4 existente: {os.path.basename(local_mp4_path)}). Ignorado.")
                success_count += 1
                continue

        print(f"\n[{idx}/{len(video_files)}] A transferir: {vname} ({format_bytes(sz)})")

        if download_single_file(get_active_host, remote_file_path, local_file_path, sz):
            success_count += 1

            # Pós-processamento automático: Conversão para MP4 e eliminação do .mkv LOCAL
            if ext.lower() == ".mkv" and not args.no_convert and is_ffmpeg_available():
                print(f"    [*] A converter {vname} para MP4...")
                conv_ok, mp4_file = convert_mkv_to_mp4(local_file_path, delete_original=not args.keep_mkv, verbose=True)
                if conv_ok:
                    print(f"    [+] Vídeo pronto em: {os.path.basename(mp4_file)}")
                else:
                    print(f"    [!] Aviso: Falha na conversão para MP4. O ficheiro .mkv local foi preservado.")

    print("\n" + "=" * 70)
    print(f" PROCESSO CONCLUÍDO: {success_count}/{len(video_files)} vídeos descarregados e preparados.")
    print(f" Destino local: {dest_dir}")
    print("=" * 70)

    # Abrir a pasta no Explorer
    try:
        os.startfile(dest_dir)
    except Exception:
        pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
