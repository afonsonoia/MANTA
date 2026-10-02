#!/usr/bin/env python3
"""
MANTA UAV — Automatic MKV to MP4 Video Converter
Lossless stream remuxing from H.264 MKV to browser/editor compatible MP4.
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
    return shutil.which("ffmpeg") is not None


def format_bytes(b):
    if b < 1024:
        return f"{b} B"
    elif b < 1024 * 1024:
        return f"{b / 1024:.1f} KB"
    elif b < 1024 * 1024 * 1024:
        return f"{b / (1024 * 1024):.1f} MB"
    else:
        return f"{b / (1024 * 1024 * 1024):.2f} GB"


def verify_mp4(mp4_path):
    if not os.path.exists(mp4_path) or os.path.getsize(mp4_path) < 1024:
        return False, 0.0

    if not shutil.which("ffprobe"):
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
    Converts a local .mkv file to .mp4 via lossless remuxing (-c copy).
    Returns (success: bool, mp4_path: str).
    """
    if not os.path.isfile(mkv_path):
        if verbose:
            print(f"[!] File not found: {mkv_path}")
        return False, None

    if not is_ffmpeg_available():
        if verbose:
            print("[!] ERROR: ffmpeg not found in system PATH.")
        return False, None

    dir_name, file_name = os.path.split(mkv_path)
    base_name, _ = os.path.splitext(file_name)
    final_mp4_path = os.path.join(dir_name, f"{base_name}.mp4")
    temp_mp4_path = os.path.join(dir_name, f"{base_name}.tmp_converting.mp4")

    if os.path.exists(final_mp4_path) and os.path.getsize(final_mp4_path) > 1024:
        valid, dur = verify_mp4(final_mp4_path)
        if valid:
            if verbose:
                print(f"    [OK] MP4 file already exists and is intact: {os.path.basename(final_mp4_path)} ({format_bytes(os.path.getsize(final_mp4_path))})")
            if delete_original and os.path.exists(mkv_path):
                try:
                    os.remove(mkv_path)
                    if verbose:
                        print(f"    [*] Removed redundant local .mkv file: {file_name}")
                except Exception as e:
                    if verbose:
                        print(f"    [!] Warning removing local .mkv: {e}")
            return True, final_mp4_path

    mkv_size = os.path.getsize(mkv_path)
    if verbose:
        print(f"    [*] Converting to MP4 (Lossless Remux): {file_name} ({format_bytes(mkv_size)})...")

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
            print(f"    [!] Failed to execute ffmpeg: {e}")
        if os.path.exists(temp_mp4_path):
            try:
                os.remove(temp_mp4_path)
            except Exception:
                pass
        return False, None

    if proc.returncode != 0:
        if verbose:
            print(f"    [!] Error during ffmpeg conversion (code {proc.returncode}).")
            err_lines = proc.stderr.strip().splitlines()[-5:]
            for line in err_lines:
                print(f"        {line}")
        if os.path.exists(temp_mp4_path):
            try:
                os.remove(temp_mp4_path)
            except Exception:
                pass
        return False, None

    valid, duration = verify_mp4(temp_mp4_path)
    if not valid:
        if verbose:
            print("    [!] Generated MP4 failed integrity check.")
        if os.path.exists(temp_mp4_path):
            try:
                os.remove(temp_mp4_path)
            except Exception:
                pass
        return False, None

    try:
        if os.path.exists(final_mp4_path):
            os.remove(final_mp4_path)
        os.rename(temp_mp4_path, final_mp4_path)
    except Exception as e:
        if verbose:
            print(f"    [!] Error finalizing MP4 file: {e}")
        return False, None

    elapsed = time.time() - start_time
    mp4_size = os.path.getsize(final_mp4_path)
    dur_str = f" ({int(duration // 60)}m{int(duration % 60):02d}s)" if duration > 0 else ""

    if verbose:
        print(f"    [+] MP4 successfully generated{dur_str}: {os.path.basename(final_mp4_path)} ({format_bytes(mp4_size)}) in {elapsed:.1f}s")

    if delete_original and os.path.exists(mkv_path):
        try:
            os.remove(mkv_path)
            if verbose:
                print(f"    [*] Deleted local .mkv to free {format_bytes(mkv_size)}.")
        except Exception as e:
            if verbose:
                print(f"    [!] Warning: Could not delete local .mkv: {e}")

    return True, final_mp4_path


def convert_directory(dir_path, delete_original=True):
    """Converts all .mkv files in a directory to .mp4."""
    if not os.path.isdir(dir_path):
        print(f"[!] Directory not found: {dir_path}")
        return 0, 0

    mkv_files = [f for f in os.listdir(dir_path) if f.lower().endswith(".mkv")]
    if not mkv_files:
        print(f"[*] No .mkv files found in: {dir_path}")
        return 0, 0

    print("=" * 70)
    print(" MANTA UAV — Video Conversion MKV -> MP4")
    print(f" Folder: {os.path.abspath(dir_path)}")
    print(f" Files found: {len(mkv_files)}")
    print("=" * 70)

    success_count = 0
    for idx, f in enumerate(mkv_files, 1):
        mkv_full_path = os.path.join(dir_path, f)
        print(f"\n[{idx}/{len(mkv_files)}] Processing: {f}")
        ok, _ = convert_mkv_to_mp4(mkv_full_path, delete_original=delete_original, verbose=True)
        if ok:
            success_count += 1

    print("\n" + "=" * 70)
    print(f" CONVERSION FINISHED: {success_count}/{len(mkv_files)} videos converted successfully.")
    print("=" * 70)
    return success_count, len(mkv_files)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="MANTA UAV — MKV to MP4 Converter")
    parser.add_argument("path", nargs="?", default="", help="Path to .mkv file or folder containing .mkv files")
    parser.add_argument("--keep-mkv", action="store_true", help="Do not delete local .mkv file after conversion")
    args = parser.parse_args()

    target_path = args.path
    if not target_path:
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
        print(f"[!] Invalid path: {target_path}")
        sys.exit(1)
