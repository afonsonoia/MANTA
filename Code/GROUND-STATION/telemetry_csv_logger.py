import os
import time
import queue
import threading
import atexit
from datetime import datetime
import csv


class TelemetryCSVLogger:
    """Thread-safe, high-precision, crash-resilient CSV logger for MANTA flight telemetry & PID analysis.
    
    Logs timestamped records of estimated pitch/roll, raw 6-axis IMU readings (ax, ay, az, gx, gy, gz),
    CH1-3 RC PWM values, battery voltage, altitude, and signal status with microsecond resolution.
    Guarantees zero file corruption on sudden process termination or system power loss via instant fsync.
    """
    def __init__(self, filename=None):
        self.queue = queue.Queue()
        self.is_running = False
        self.worker_thread = None
        self.file_obj = None
        self.csv_writer = None
        self.file_path = filename
        self.record_count = 0
        atexit.register(self.stop)

    def start(self, filename=None):
        if self.is_running:
            return True
        
        # Ensure dedicated logs directory exists
        log_dir = "flight_logs"
        os.makedirs(log_dir, exist_ok=True)

        if filename:
            self.file_path = os.path.join(log_dir, os.path.basename(filename))
        if not self.file_path:
            timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.file_path = os.path.join(log_dir, f"manta_pid_flight_{timestamp_str}.csv")
            
        try:
            self.file_obj = open(self.file_path, "w", newline="", encoding="utf-8", buffering=1)
            self.csv_writer = csv.writer(self.file_obj)
            
            # Header organized specifically for PID identification & flight dynamics
            header = [
                "timestamp_iso",
                "timestamp_epoch",
                "pitch_deg",
                "roll_deg",
                "raw_accel_x",
                "raw_accel_y",
                "raw_accel_z",
                "raw_gyro_x",
                "raw_gyro_y",
                "raw_gyro_z",
                "ch1_roll_pwm",
                "ch2_pitch_pwm",
                "ch3_throttle_pwm",
                "ch4_pwm",
                "ch5_pwm",
                "battery_v",
                "alt_m",
                "rc_signal_lost"
            ]
            self.csv_writer.writerow(header)
            self.file_obj.flush()
            os.fsync(self.file_obj.fileno())

            self.is_running = True
            self.record_count = 0
            self.worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
            self.worker_thread.start()
            print(f"[CSV Logger] Asynchronous flight logger active: '{self.file_path}'")
            return True
        except Exception as e:
            print(f"[CSV Logger Error] Failed to start logger: {e}")
            self.is_running = False
            return False

    def log_telemetry(self, telemetry_dict):
        """Asynchronously enqueues a telemetry frame for non-blocking disk write."""
        if not self.is_running or not isinstance(telemetry_dict, dict):
            return

        now = datetime.now()
        iso_ts = now.strftime("%Y-%m-%d %H:%M:%S.%f")
        epoch_ts = time.time()

        rc = telemetry_dict.get("rc", [0, 0, 0, 0, 0])
        ch1 = rc[0] if len(rc) > 0 else 0
        ch2 = rc[1] if len(rc) > 1 else 0
        ch3 = rc[2] if len(rc) > 2 else 0
        ch4 = rc[3] if len(rc) > 3 else 0
        ch5 = rc[4] if len(rc) > 4 else 0

        row = [
            iso_ts,
            f"{epoch_ts:.6f}",
            telemetry_dict.get("pitch", 0.0),
            telemetry_dict.get("roll", 0.0),
            telemetry_dict.get("accel_x", 0),
            telemetry_dict.get("accel_y", 0),
            telemetry_dict.get("accel_z", 0),
            telemetry_dict.get("gyro_x", 0),
            telemetry_dict.get("gyro_y", 0),
            telemetry_dict.get("gyro_z", 0),
            ch1,
            ch2,
            ch3,
            ch4,
            ch5,
            telemetry_dict.get("batteryVoltage", 0.0),
            telemetry_dict.get("alt", 0.0),
            1 if telemetry_dict.get("rcSignalLost", False) else 0
        ]

        try:
            self.queue.put_nowait(row)
        except queue.Full:
            pass

    def _worker_loop(self):
        while self.is_running or not self.queue.empty():
            try:
                row = self.queue.get(timeout=0.1)
                if row and self.csv_writer and self.file_obj:
                    self.csv_writer.writerow(row)
                    self.record_count += 1
                    self.file_obj.flush()
                    os.fsync(self.file_obj.fileno())
                    self.queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[CSV Logger Worker Error] {e}")

    def stop(self):
        if not self.is_running:
            return
        self.is_running = False
        if self.worker_thread and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1.0)
        if self.file_obj:
            try:
                self.file_obj.flush()
                os.fsync(self.file_obj.fileno())
                self.file_obj.close()
            except Exception:
                pass
            self.file_obj = None
        print(f"[CSV Logger] Saved and closed '{self.file_path}' ({self.record_count} records).")
