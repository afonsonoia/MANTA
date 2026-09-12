#!/usr/bin/env python3
"""
MANTA UAV — Conversor Automático de Vídeos MKV para MP4
======================================================
Converte gravações de voo (.mkv com H.264 gerado pelo Raspberry Pi)
para formato .mp4 compatível com browsers, editores e leitores de vídeo
através de remuxing sem perdas (lossless stream copy, ultrarrápido).

Após conversão e validação com sucesso, apaga o ficheiro .mkv LOCAL
para poupar espaço em disco.
NUNCA altera nem apaga ficheiros remotos no Raspberry Pi.
"""

import os
import sys
import io
import subprocess
import shutil
import time

try:
    if sys.platform == "win32":
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
    else:
        sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

def is_ffmpeg_available():
    """Verifica se o ffmpeg está instalado e acessível no sistema."""
    return shutil.which("ffmpeg") is not None


def format_bytes(b):
    """Converte bytes em formato legível."""
    if b < 1024:
        return f"{b} B"
    elif b < 1024 * 1024:
        return f"{b / 1024:.1f} KB"
    elif b < 1024 * 1024 * 1024:
        return f"{b / (1024 * 1024):.1f} MB"
    else:
        return f"{b / (1024 * 1024 * 1024):.2f} GB"


def verify_mp4(mp4_path):
    """Verifica a integridade do ficheiro MP4 gerado usando ffprobe."""
    if not os.path.exists(mp4_path) or os.path.getsize(mp4_path) < 1024:
        return False, 0.0

    if not shutil.which("ffprobe"):
        # Se ffprobe não estiver disponível, validação básica pelo tamanho
        return True, 0.0

    cmd = [
        "ffprobe", "-v", "error",
        "-show_entries", "format=duration",
        "-of", "default=noprint_wrappers=1:nokey=1",
        mp4_path
    ]
    try:
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
        dur_str = res.stdout.strip()
        duration = float(dur_str) if dur_str else 0.0
        return True, duration
    except Exception:
        return False, 0.0


def convert_mkv_to_mp4(mkv_path, delete_original=True, verbose=True):
    """
    Converte um ficheiro .mkv local para .mp4 via lossless remuxing (-c copy).
    
    Parâmetros:
      - mkv_path: Caminho completo para o ficheiro .mkv local.
      - delete_original: Se True, apaga o .mkv local após validação do .mp4.
      - verbose: Se True, imprime detalhes no terminal.
      
    Retorna:
      - (sucesso: bool, caminho_mp4: str)
    """
    if not os.path.isfile(mkv_path):
        if verbose:
            print(f"[!] Ficheiro não encontrado: {mkv_path}")
        return False, None

    if not is_ffmpeg_available():
        if verbose:
            print("[!] ERRO: ffmpeg não encontrado no PATH do sistema.")
        return False, None

    dir_name, file_name = os.path.split(mkv_path)
    base_name, _ = os.path.splitext(file_name)
    final_mp4_path = os.path.join(dir_name, f"{base_name}.mp4")
    temp_mp4_path = os.path.join(dir_name, f"{base_name}.tmp_converting.mp4")

    # Se já existir o mp4 final e for válido, não re-converter
    if os.path.exists(final_mp4_path) and os.path.getsize(final_mp4_path) > 1024:
        valid, dur = verify_mp4(final_mp4_path)
        if valid:
            if verbose:
                print(f"    [OK] Ficheiro MP4 já existe e está íntegro: {os.path.basename(final_mp4_path)} ({format_bytes(os.path.getsize(final_mp4_path))})")
            if delete_original and os.path.exists(mkv_path):
                try:
                    os.remove(mkv_path)
                    if verbose:
                        print(f"    [*] Removido ficheiro .mkv local redundante: {file_name}")
                except Exception as e:
                    if verbose:
                        print(f"    [!] Aviso ao remover .mkv local: {e}")
            return True, final_mp4_path

    mkv_size = os.path.getsize(mkv_path)
    if verbose:
        print(f"    [*] A converter para MP4 (Lossless Remux): {file_name} ({format_bytes(mkv_size)})...")

    # Limpar ficheiro temporário antigo se existir
    if os.path.exists(temp_mp4_path):
        try:
            os.remove(temp_mp4_path)
        except Exception:
            pass

    cmd = [
        "ffmpeg", "-y",
        "-i", mkv_path,
        "-c", "copy",
        "-movflags", "+faststart",
        temp_mp4_path
    ]

    start_time = time.time()
    try:
        proc = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
    except Exception as e:
        if verbose:
            print(f"    [!] Falha ao executar ffmpeg: {e}")
        if os.path.exists(temp_mp4_path):
            try:
                os.remove(temp_mp4_path)
            except Exception:
                pass
        return False, None

    if proc.returncode != 0:
        if verbose:
            print(f"    [!] Erro durante conversão ffmpeg (código {proc.returncode}).")
            err_lines = proc.stderr.strip().splitlines()[-5:]
            for line in err_lines:
                print(f"        {line}")
        if os.path.exists(temp_mp4_path):
            try:
                os.remove(temp_mp4_path)
            except Exception:
                pass
        return False, None

    # Validar ficheiro temporário gerado
    valid, duration = verify_mp4(temp_mp4_path)
    if not valid:
        if verbose:
            print("    [!] Ficheiro MP4 gerado falhou na verificação de integridade.")
        if os.path.exists(temp_mp4_path):
            try:
                os.remove(temp_mp4_path)
            except Exception:
                pass
        return False, None

    # Mover temporário para o destino final
    try:
        if os.path.exists(final_mp4_path):
            os.remove(final_mp4_path)
        os.rename(temp_mp4_path, final_mp4_path)
    except Exception as e:
        if verbose:
            print(f"    [!] Erro ao finalizar ficheiro MP4: {e}")
        return False, None

    elapsed = time.time() - start_time
    mp4_size = os.path.getsize(final_mp4_path)
    dur_str = f" ({int(duration // 60)}m{int(duration % 60):02d}s)" if duration > 0 else ""

    if verbose:
        print(f"    [+] MP4 gerado com sucesso{dur_str}: {os.path.basename(final_mp4_path)} ({format_bytes(mp4_size)}) em {elapsed:.1f}s")

    # Apagar o ficheiro .mkv LOCAL se solicitado
    if delete_original and os.path.exists(mkv_path):
        try:
            os.remove(mkv_path)
            if verbose:
                print(f"    [*] Ficheiro .mkv local apagado para libertar {format_bytes(mkv_size)}.")
        except Exception as e:
            if verbose:
                print(f"    [!] Aviso: Não foi possível apagar o .mkv local: {e}")

    return True, final_mp4_path


def convert_directory(dir_path, delete_original=True):
    """Converte todos os ficheiros .mkv num diretório para .mp4."""
    if not os.path.isdir(dir_path):
        print(f"[!] Diretório não encontrado: {dir_path}")
        return 0, 0

    mkv_files = [f for f in os.listdir(dir_path) if f.lower().endswith(".mkv")]
    if not mkv_files:
        print(f"[*] Nenhum ficheiro .mkv encontrado em: {dir_path}")
        return 0, 0

    print("=" * 70)
    print(f" MANTA UAV — Conversão de Vídeos MKV -> MP4")
    print(f" Pasta: {os.path.abspath(dir_path)}")
    print(f" Ficheiros encontrados: {len(mkv_files)}")
    print("=" * 70)

    success_count = 0
    for idx, f in enumerate(mkv_files, 1):
        mkv_full_path = os.path.join(dir_path, f)
        print(f"\n[{idx}/{len(mkv_files)}] Processando: {f}")
        ok, _ = convert_mkv_to_mp4(mkv_full_path, delete_original=delete_original, verbose=True)
        if ok:
            success_count += 1

    print("\n" + "=" * 70)
    print(f" CONVERSÃO FINALIZADA: {success_count}/{len(mkv_files)} vídeos convertidos com sucesso.")
    print("=" * 70)
    return success_count, len(mkv_files)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="MANTA UAV — Conversor MKV para MP4")
    parser.add_argument("path", nargs="?", default="", help="Caminho para ficheiro .mkv ou pasta com ficheiros .mkv")
    parser.add_argument("--keep-mkv", action="store_true", help="Não apagar o ficheiro .mkv local após a conversão")
    args = parser.parse_args()

    target_path = args.path
    if not target_path:
        # Padrão: pasta flight_videos do projeto
        default_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "flight_videos")
        if os.path.isdir(default_dir):
            target_path = default_dir
        else:
            parser.print_help()
            sys.exit(1)

    if os.path.isfile(target_path):
        convert_mkv_to_mp4(target_path, delete_original=not args.keep_mkv)
    elif os.path.isdir(target_path):
        convert_directory(target_path, delete_original=not args.keep_mkv)
    else:
        print(f"[!] Caminho inválido: {target_path}")
        sys.exit(1)
