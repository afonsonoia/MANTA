"""
MANTA Companion Computer — Safe Flight Recorder (Python Controller)
Manages power-loss immune flight video recording with anti-vibration
optimizations for the Sony IMX378-79 sensor.
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
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR, help="Directory to save video files")
    parser.add_argument("--width", type=int, default=1920, help="Video width (default: 1920)")
    parser.add_argument("--height", type=int, default=1080, help="Video height (default: 1080)")
    parser.add_argument("--fps", type=int, default=30, help="Frame rate in fps (default: 30)")
    parser.add_argument("--exposure", default="sport", choices=["normal", "sport"],
                        help="Exposure mode (sport = fast shutter anti-vibration priority)")
    parser.add_argument("--shutter", type=int, default=0,
                        help="Fixed shutter speed in microseconds (e.g. 2000 = 1/500s; 0 = auto)")
    parser.add_argument("--duration", type=int, default=0,
                        help="Recording duration in seconds (0 = infinite until stopped or power cut)")
    return parser.parse_args()

def main():
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    free_mb = get_free_disk_mb(args.output_dir)
    if free_mb < 500:
        print(f"[!] ERROR: Only {free_mb} MB available in {args.output_dir}. Minimum 500 MB required.")
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
    print(" MANTA UAV — Safe Flight Recording (Anti-Vibration & Power-Loss Safe)")
    print(f" Target: {out_filename}")
    print(f" Resolution: {args.width}x{args.height} @ {args.fps}fps | Exposure: {args.exposure}")
    if args.shutter > 0:
        print(f" Fixed Shutter: {args.shutter} µs (1/{1_000_000 // args.shutter}s)")
    print(f" Free disk space: {free_mb} MB")
    print(" Format: Matroska (.mkv) with synchronous buffer flush")
    print("=" * 65)

    proc = subprocess.Popen(cmd)

    def sig_handler(signum, frame):
        print("\n[*] Shutdown signal received. Closing recording safely...")
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
        subprocess.run(["sync"])
        print(f"[+] Video saved successfully: {out_filename}")
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    try:
        proc.wait()
    except KeyboardInterrupt:
        sig_handler(None, None)

    subprocess.run(["sync"])
    print(f"[+] Recording finished: {out_filename}")

if __name__ == "__main__":
    main()
