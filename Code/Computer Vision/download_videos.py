#!/usr/bin/env python3
"""
MANTA UAV — Resilient Flight Video Downloader
Downloads flight recordings from Raspberry Pi via SFTP with chunked streaming,
automatic resume on reconnection, and lossless MP4 conversion.
"""

import os
import sys
import io
import time
import socket
import paramiko
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor
import argparse

try:
    if sys.platform == "win32":
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
    else:
        sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

DEFAULT_HOSTS = ["10.32.198.35", "manta.local"]
RPI_USER = "pc"
RPI_PASS = "134679"
REMOTE_VIDEO_DIR = "/home/pc/flight_videos"
LOCAL_DEST_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "flight_videos")
CHUNK_SIZE = 256 * 1024  # 256 KB chunk
MAX_RETRIES_PER_FILE = 20
RETRY_DELAY_SEC = 2

try:
    from convert_videos import convert_mkv_to_mp4, is_ffmpeg_available
except ImportError:
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from convert_videos import convert_mkv_to_mp4, is_ffmpeg_available


def test_ssh_auth(ip):
    try:
        c = paramiko.SSHClient()
        c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        c.connect(ip, username=RPI_USER, password=RPI_PASS, timeout=2.0, banner_timeout=4)
        c.close()
        return True
    except Exception:
        return False


def check_ip_candidate(ip):
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
    print("[*] Searching for Raspberry Pi on the network...")
    start_time = time.time()
    last_print = 0
    while time.time() - start_time < max_wait_sec:
        host = resolve_host()
        if host:
            print(f"[+] Raspberry Pi found and reachable at: {host}!")
            return host
        
        now = time.time()
        if now - last_print >= 5:
            elapsed = int(now - start_time)
            print(f"    Waiting for Raspberry Pi Wi-Fi connection... ({elapsed}s elapsed)")
            last_print = now
        time.sleep(1.5)

    return None


def create_sftp_client(host):
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
        transport.set_keepalive(5)
    sftp = ssh.open_sftp()
    return ssh, sftp


def format_bytes(b):
    if b < 1024:
        return f"{b} B"
    elif b < 1024 * 1024:
        return f"{b / 1024:.1f} KB"
    elif b < 1024 * 1024 * 1024:
        return f"{b / (1024 * 1024):.1f} MB"
    else:
        return f"{b / (1024 * 1024 * 1024):.2f} GB"


def download_single_file(host_getter, remote_path, local_path, file_size):
    retries = 0
    os.makedirs(os.path.dirname(local_path), exist_ok=True)

    while retries < MAX_RETRIES_PER_FILE:
        existing_bytes = 0
        if os.path.exists(local_path):
            existing_bytes = os.path.getsize(local_path)
            if existing_bytes == file_size and file_size > 0:
                print(f"    [OK] File already complete locally ({format_bytes(file_size)}). Skipped.")
                return True
            elif existing_bytes > file_size:
                os.remove(local_path)
                existing_bytes = 0

        host = host_getter()
        if not host:
            print(f"    [!] Pi temporarily unreachable. Retrying in {RETRY_DELAY_SEC}s...")
            time.sleep(RETRY_DELAY_SEC)
            retries += 1
            continue

        ssh = None
        sftp = None
        try:
            ssh, sftp = create_sftp_client(host)
            
            with sftp.open(remote_path, 'rb') as rfile:
                # Do not use rfile.prefetch() to prevent memory exhaustion on RPi 3 A+
                if existing_bytes > 0:
                    rfile.seek(existing_bytes)
                    print(f"    [+] Resuming download from {format_bytes(existing_bytes)} / {format_bytes(file_size)} ({int(existing_bytes/file_size*100)}%)...")
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
                                f"{format_bytes(speed)}/s | Remaining: {rem_str}   "
                            )
                            sys.stdout.flush()
                            last_ui_time = now

                print()
                if transferred >= file_size:
                    print(f"    [+] Download completed successfully: {os.path.basename(local_path)}")
                    return True
                else:
                    print(f"    [!] Transfer incomplete ({transferred}/{file_size} bytes). Reconnecting to resume...")

        except Exception as e:
            print(f"\n    [!] Disconnection during transfer: {e}")
            retries += 1
            print(f"    [*] Awaiting reconnection (attempt {retries}/{MAX_RETRIES_PER_FILE})...")
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

    print(f"    [!] ERROR: Exhausted retries downloading {os.path.basename(remote_path)}")
    return False


def main():
    parser = argparse.ArgumentParser(description="MANTA UAV — Safe Flight Video Downloader & Converter")
    parser.add_argument("--no-convert", action="store_true", help="Do not automatically convert .mkv to .mp4")
    parser.add_argument("--keep-mkv", action="store_true", help="Keep local .mkv file after .mp4 conversion")
    parser.add_argument("--dest", default=LOCAL_DEST_DIR, help=f"Local destination directory (default: {LOCAL_DEST_DIR})")
    args = parser.parse_args()

    dest_dir = os.path.abspath(args.dest)
    os.makedirs(dest_dir, exist_ok=True)

    print("=" * 70)
    print(" MANTA UAV — Resilient Flight Video Downloader")
    print("=" * 70)
    print(f" Local destination: {dest_dir}")
    if not args.no_convert:
        if is_ffmpeg_available():
            print(" Automatic MP4 conversion: ACTIVE (Lossless Remuxing)")
            print(f" Local file handling: Local .mkv will be {'KEPT' if args.keep_mkv else 'DELETED after MP4 verification'}")
            print(" Remote files on Raspberry Pi: UNTOUCHED (never deleted remotely)")
        else:
            print(" [!] Warning: ffmpeg not found on system. MP4 conversion unavailable.")

    host = wait_for_pi(max_wait_sec=600)
    if not host:
        print("[!] Raspberry Pi not found on the network after 10 minutes.")
        print("    Ensure:")
        print("    1. Raspberry Pi is powered.")
        print("    2. 'DELTA' hotspot/router is active.")
        print("    3. If Pi entered Flight Mode (Wi-Fi LED off), power cycle it.")
        return 1

    print(f"[*] Fetching recording list from {REMOTE_VIDEO_DIR}...")
    try:
        ssh, sftp = create_sftp_client(host)
    except Exception as e:
        print(f"[!] SSH connection error: {e}")
        return 1

    try:
        remote_files = sftp.listdir_attr(REMOTE_VIDEO_DIR)
    except Exception as e:
        print(f"[!] Error accessing {REMOTE_VIDEO_DIR}: {e}")
        sftp.close()
        ssh.close()
        return 1

    video_files = []
    for f in remote_files:
        name = f.filename
        if name.lower().endswith(('.mkv', '.mp4', '.h264')):
            video_files.append((name, f.st_size))

    sftp.close()
    ssh.close()

    if not video_files:
        print(f"[+] No video files found in {REMOTE_VIDEO_DIR}.")
        return 0

    total_size = sum(sz for _, sz in video_files)
    print(f"[+] Found {len(video_files)} videos (Total: {format_bytes(total_size)}):")
    for idx, (vname, sz) in enumerate(video_files, 1):
        print(f"    {idx}. {vname} ({format_bytes(sz)})")

    print("-" * 70)
    print("[*] Starting resilient download...")

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

        if ext.lower() == ".mkv" and not args.no_convert:
            local_mp4_path = os.path.join(dest_dir, f"{base_name}.mp4")
            if os.path.exists(local_mp4_path) and os.path.getsize(local_mp4_path) > 1024:
                print(f"\n[{idx}/{len(video_files)}] [OK] {vname} already downloaded and converted (.mp4 exists: {os.path.basename(local_mp4_path)}). Skipped.")
                success_count += 1
                continue

        print(f"\n[{idx}/{len(video_files)}] Downloading: {vname} ({format_bytes(sz)})")

        if download_single_file(get_active_host, remote_file_path, local_file_path, sz):
            success_count += 1

            if ext.lower() == ".mkv" and not args.no_convert and is_ffmpeg_available():
                print(f"    [*] Converting {vname} to MP4...")
                conv_ok, mp4_file = convert_mkv_to_mp4(local_file_path, delete_original=not args.keep_mkv, verbose=True)
                if conv_ok:
                    print(f"    [+] Video ready at: {os.path.basename(mp4_file)}")
                else:
                    print("    [!] Warning: Failed MP4 conversion. Local .mkv preserved.")

    print("\n" + "=" * 70)
    print(f" PROCESS COMPLETE: {success_count}/{len(video_files)} videos downloaded and prepared.")
    print(f" Local destination: {dest_dir}")
    print("=" * 70)

    try:
        os.startfile(dest_dir)
    except Exception:
        pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
