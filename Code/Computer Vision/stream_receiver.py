"""
MANTA Companion Computer — Ultra Low-Latency SSH Video Stream Receiver
Receives and displays live H.264 video stream from Raspberry Pi 3 A+.
Calculates real-time focus metric (Laplacian variance) and displays HUD focus score.
"""

import os
import sys
import time
import json
import shutil
import argparse
import threading
import subprocess
import paramiko
import cv2
import numpy as np

RPI_HOST = "manta.local"
RPI_USER = "pc"
RPI_PASS = "134679"

WIDTH = 1280
HEIGHT = 720
FPS = 30

CONFIG_FILE = os.path.join(os.path.dirname(__file__), "fpv_config.json")

def load_fpv_config():
    cfg = {"vflip": True, "hflip": True}
    if os.path.isfile(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                saved = json.load(f)
                cfg.update(saved)
        except Exception:
            pass
    return cfg

def check_ffmpeg():
    ffmpeg_path = shutil.which("ffmpeg")
    if not ffmpeg_path:
        print("[!] Error: ffmpeg was not found in Windows PATH.")
        print("    Please ensure FFmpeg is installed.")
        return None
    return ffmpeg_path

def parse_args():
    cfg = load_fpv_config()
    parser = argparse.ArgumentParser(description="MANTA UAV — Live FPV Video Stream Receiver with Focus Meter")
    parser.add_argument("--host", default=RPI_HOST, help="Raspberry Pi IP address or hostname")
    parser.add_argument("--width", type=int, default=WIDTH, help="Video width")
    parser.add_argument("--height", type=int, default=HEIGHT, help="Video height")
    parser.add_argument("--fps", type=int, default=FPS, help="Frame rate in fps")

    # Image orientation
    parser.add_argument("--vflip", dest="vflip", action="store_true", default=None,
                        help="Enable vertical flip (V-Flip)")
    parser.add_argument("--no-vflip", dest="vflip", action="store_false",
                        help="Disable vertical flip")
    parser.add_argument("--hflip", dest="hflip", action="store_true", default=None,
                        help="Enable horizontal flip (H-Flip)")
    parser.add_argument("--no-hflip", dest="hflip", action="store_false",
                        help="Disable horizontal flip")

    # Focus HUD options
    parser.add_argument("--no-focus", dest="show_focus", action="store_false", default=True,
                        help="Disable focus meter HUD overlay")

    args = parser.parse_args()

    if args.vflip is None:
        args.vflip = cfg.get("vflip", True)
    if args.hflip is None:
        args.hflip = cfg.get("hflip", True)

    return args

def main():
    args = parse_args()

    ffmpeg_bin = check_ffmpeg()
    if not ffmpeg_bin:
        sys.exit(1)

    target_host = args.host

    flip_tags = []
    if args.vflip:
        flip_tags.append("V-Flip")
    if args.hflip:
        flip_tags.append("H-Flip")
    orient_str = f" [{', '.join(flip_tags)}]" if flip_tags else " [Normal]"

    print("=" * 60)
    print(" MANTA UAV — Live Video Stream with Focus Meter")
    print(f" Target: {RPI_USER}@{target_host} | Resolution: {args.width}x{args.height} @ {args.fps}fps")
    print(f" Image Orientation:{orient_str}")
    print(" Shortcuts: [Q] or [ESC] to quit | [R] to reset peak focus score")
    print("=" * 60)

    print(f"[*] Establishing SSH connection to {target_host}...")
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    try:
        client.connect(target_host, username=RPI_USER, password=RPI_PASS, timeout=8)
    except Exception as e:
        print(f"[!] Error connecting to Raspberry Pi: {e}")
        print("    Verify that Raspberry Pi is powered and connected to Wi-Fi.")
        sys.exit(1)

    print("[+] SSH connection established successfully!")

    stdin, stdout, stderr = client.exec_command("rpicam-hello --list-cameras")
    cam_out = stdout.read().decode('utf-8')
    print("[*] Camera hardware verification:")
    print(cam_out.strip() if cam_out.strip() else "    (No camera listed automatically).")

    flip_flags = []
    if args.vflip:
        flip_flags.append("--vflip")
    if args.hflip:
        flip_flags.append("--hflip")
    flip_cmd_str = f" {' '.join(flip_flags)}" if flip_flags else ""

    remote_cmd = (
        f"rpicam-vid -t 0 --inline --flush --profile baseline --intra 10 "
        f"--width {args.width} --height {args.height} --framerate {args.fps} --bitrate 3000000 "
        f"--exposure sport --denoise cdn_off --autofocus-mode manual --lens-position 0.0 "
        f"--nopreview --codec h264{flip_cmd_str} -o -"
    )

    print("[*] Terminating stale camera processes on Raspberry Pi...")
    client.exec_command("pkill -9 -f rpicam 2>/dev/null; pkill -9 -f v4l2 2>/dev/null")
    time.sleep(0.4)

    print(f"[*] Launching remote stream command: {remote_cmd}")
    stdin, stdout, stderr = client.exec_command(remote_cmd, bufsize=0)

    def _read_remote_stderr():
        try:
            for line in iter(stderr.readline, ""):
                if line:
                    line_str = line.strip()
                    if "ERROR" in line_str or "WARN" in line_str or "failed" in line_str or "timed out" in line_str:
                        print(f"[RPi CAMERA] {line_str}", file=sys.stderr)
        except Exception:
            pass

    err_thread = threading.Thread(target=_read_remote_stderr, daemon=True)
    err_thread.start()

    ffmpeg_dec_cmd = [
        ffmpeg_bin,
        "-probesize", "32",
        "-analyzeduration", "0",
        "-fflags", "nobuffer",
        "-flags", "low_delay",
        "-f", "h264",
        "-i", "pipe:0",
        "-f", "rawvideo",
        "-pix_fmt", "bgr24",
        "-"
    ]

    print("[*] Starting FFmpeg video decoder and display window...")
    decoder = subprocess.Popen(
        ffmpeg_dec_cmd,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        bufsize=0
    )

    channel = stdout.channel
    channel.setblocking(True)

    is_running = threading.Event()
    is_running.set()

    def _feeder():
        try:
            while is_running.is_set():
                chunk = channel.recv(32768)
                if not chunk:
                    break
                decoder.stdin.write(chunk)
                decoder.stdin.flush()
        except Exception:
            pass
        finally:
            try:
                decoder.stdin.close()
            except Exception:
                pass
            is_running.clear()

    feed_thread = threading.Thread(target=_feeder, daemon=True)
    feed_thread.start()

    frame_bytes = args.width * args.height * 3
    window_name = f"MANTA FPV — Live Video Stream (Focus Meter){orient_str}"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_EXPANDED)
    cv2.resizeWindow(window_name, args.width, args.height)

    roi_h = int(args.height * 0.5)
    roi_w = int(args.width * 0.5)
    y1 = (args.height - roi_h) // 2
    y2 = y1 + roi_h
    x1 = (args.width - roi_w) // 2
    x2 = x1 + roi_w

    max_focus_score = 1.0
    smooth_score = 0.0
    alpha = 0.25

    try:
        while is_running.is_set():
            raw_data = bytearray()
            while len(raw_data) < frame_bytes and is_running.is_set():
                needed = frame_bytes - len(raw_data)
                part = decoder.stdout.read(needed)
                if not part:
                    break
                raw_data.extend(part)

            if len(raw_data) < frame_bytes:
                break

            frame = np.frombuffer(raw_data, dtype=np.uint8).reshape((args.height, args.width, 3)).copy()

            if args.show_focus:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                roi_gray = gray[y1:y2, x1:x2]

                # Focus measurement: Laplacian variance
                raw_score = float(cv2.Laplacian(roi_gray, cv2.CV_64F).var())
                smooth_score = (alpha * raw_score) + ((1.0 - alpha) * smooth_score)
                if smooth_score > max_focus_score:
                    max_focus_score = smooth_score

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 220, 255), 1)
                cx, cy = args.width // 2, args.height // 2
                cv2.line(frame, (cx - 15, cy), (cx + 15, cy), (0, 220, 255), 1)
                cv2.line(frame, (cx, cy - 15), (cx, cy + 15), (0, 220, 255), 1)

                hud_x, hud_y = 20, 20
                hud_w, hud_h = 360, 80
                sub_img = frame[hud_y:hud_y+hud_h, hud_x:hud_x+hud_w]
                dark_rect = np.zeros_like(sub_img)
                cv2.addWeighted(sub_img, 0.4, dark_rect, 0.6, 0, sub_img)
                cv2.rectangle(frame, (hud_x, hud_y), (hud_x+hud_w, hud_y+hud_h), (0, 180, 216), 1)

                pct_max = min(1.0, smooth_score / max(max_focus_score, 10.0))
                color_b = int(255 * (1.0 - pct_max))
                color_g = int(255 * pct_max)
                color_r = int(50 * (1.0 - pct_max))

                score_text = f"FOCUS: {smooth_score:6.1f}"
                cv2.putText(frame, score_text, (hud_x + 12, hud_y + 35),
                            cv2.FONT_HERSHEY_DUPLEX, 0.9, (color_b, color_g, color_r), 2, cv2.LINE_AA)

                bar_x1 = hud_x + 12
                bar_y1 = hud_y + 50
                bar_w = hud_w - 24
                bar_h = 16
                cv2.rectangle(frame, (bar_x1, bar_y1), (bar_x1 + bar_w, bar_y1 + bar_h), (80, 80, 80), -1)
                filled_w = int(bar_w * pct_max)
                if filled_w > 0:
                    cv2.rectangle(frame, (bar_x1, bar_y1), (bar_x1 + filled_w, bar_y1 + bar_h),
                                  (color_b, color_g, color_r), -1)
                cv2.rectangle(frame, (bar_x1, bar_y1), (bar_x1 + bar_w, bar_y1 + bar_h), (200, 200, 200), 1)

                cv2.putText(frame, f"PEAK: {max_focus_score:.1f} (R=Reset)", (hud_x + 12, hud_y + 73),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1, cv2.LINE_AA)

            cv2.imshow(window_name, frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break
            elif key == ord('r') or key == ord('R'):
                max_focus_score = max(smooth_score, 1.0)

    except KeyboardInterrupt:
        print("\n[*] Stream interrupted by user.")
    finally:
        print("[*] Terminating processes...")
        is_running.clear()
        try:
            decoder.terminate()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        try:
            client.exec_command("pkill -9 -f rpicam 2>/dev/null")
        except Exception:
            pass
        client.close()
        print("[+] Session closed cleanly.")

if __name__ == "__main__":
    main()
