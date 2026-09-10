import serial
import serial.tools.list_ports
import time
import os
import math
import re
import json
import subprocess
import socket
import queue
import threading
import sys
import csv
from collections import deque

try:
    from pymavlink import mavutil
    HAS_PYMAVLINK = True
except ImportError:
    HAS_PYMAVLINK = False

from telemetry_codec import decode_telemetry, encode_telemetry, decode_ch5_mode, PACKET_SIZE, SUPPORTED_PACKET_SIZES, SUPPORTED_PACKET_SIZES_DESC

# Config
DEFAULT_BAUD = 115200
LOG_DIR = 'flight_logs'
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT_DIR = os.path.abspath(os.path.join(_SCRIPT_DIR, '..', '..'))
if os.path.exists(os.path.join(_ROOT_DIR, 'imu_calibration.json')):
    CALIB_FILE = os.path.join(_ROOT_DIR, 'imu_calibration.json')
elif os.path.exists('imu_calibration.json'):
    CALIB_FILE = os.path.abspath('imu_calibration.json')
else:
    CALIB_FILE = os.path.join(_ROOT_DIR, 'imu_calibration.json')

def get_next_flight_log_filename(log_dir=LOG_DIR):
    """Scans flight_logs/ directory and returns the next sequential flight log CSV filename."""
    os.makedirs(log_dir, exist_ok=True)
    existing_files = os.listdir(log_dir)
    
    max_idx = 0
    pattern = re.compile(r'manta_flight_(\d+)', re.IGNORECASE)
    for fname in existing_files:
        match = pattern.search(fname)
        if match:
            try:
                idx = int(match.group(1))
                if idx > max_idx:
                    max_idx = idx
            except ValueError:
                pass

    next_idx = max_idx + 1
    timestamp_str = time.strftime("%Y%m%d_%H%M%S")
    filename = f"manta_flight_{next_idx:04d}_{timestamp_str}.csv"
    return os.path.join(log_dir, filename)

# Global state
active_serial_conn = None
last_rssi = None
last_snr = None
latest_estimated_voltage = 12.50
battery_voltage_history = deque(maxlen=20)

def calculate_battery_pct(voltage: float) -> int:
    if voltage <= 0.0:
        return 0
    if voltage > 13.0:
        # 4S LiPo: 13.6V (0%) to 16.8V (100%)
        return int(max(0, min(100, (voltage - 13.6) / (16.8 - 13.6) * 100)))
    else:
        # 3S LiPo: 10.2V (0%) to 12.6V (100%)
        return int(max(0, min(100, (voltage - 10.2) / (12.6 - 10.2) * 100)))

# Current IMU, GPS & Baro state
latest_pitch = 0.0
latest_roll = 0.0
latest_yaw = 0.0
latest_gx = 0
latest_gy = 0
latest_gz = 0
latest_lat = 0.0
latest_lon = 0.0
latest_alt = 0.0
latest_temp = 25.0
latest_satellites = 0
latest_fix_type = 0
pitch_offset = 0.0
roll_offset = 0.0

rc_signal_lost = False
latest_rc = [1500, 1500, 1000, 1500, 1500]
rc_margin_deadband = 18
alert_voltage_threshold = 12.50

mission_planner_proc = None
excel_logger = None

def ensure_mission_planner_autoconnect():
    """Ensures Mission Planner's config.xml has the default UDP AutoConnect rule enabled for port 14550."""
    try:
        config_path = os.path.expandvars(r'%USERPROFILE%\Documents\Mission Planner\config.xml')
        if os.path.exists(config_path):
            with open(config_path, 'r', encoding='utf-8') as f:
                content = f.read()

            start_tag = '<AutoConnect>'
            end_tag = '</AutoConnect>'
            if start_tag in content and end_tag in content:
                idx1 = content.find(start_tag) + len(start_tag)
                idx2 = content.find(end_tag)
                json_str = content[idx1:idx2]
                autoconnect_list = json.loads(json_str)
                
                updated = False
                has_udp = False
                for item in autoconnect_list:
                    if item.get('Port') == 14550 and item.get('Protocol') == 'Udp' and item.get('Direction') == 'Inbound':
                        has_udp = True
                        if not item.get('Enabled'):
                            item['Enabled'] = True
                            updated = True
                    elif item.get('Protocol') in ['Serial', 'ComPort', 'Com']:
                        if item.get('Enabled'):
                            item['Enabled'] = False
                            updated = True
                if not has_udp:
                    autoconnect_list.insert(0, {
                        'Label': 'Mavlink default port',
                        'Enabled': True,
                        'Port': 14550,
                        'Protocol': 'Udp',
                        'Format': 'MAVLink',
                        'Direction': 'Inbound',
                        'ConfigString': ''
                    })
                    updated = True
                
                if updated:
                    new_json_str = json.dumps(autoconnect_list, indent=2)
                    new_content = content[:idx1] + new_json_str + content[idx2:]
                    with open(config_path, 'w', encoding='utf-8') as f:
                        f.write(new_content)
                    print("[Mission Planner Config] Configured UDP 14550 AutoConnect and disabled Serial COM scan.")
    except Exception as e:
        print(f"[Mission Planner Config Warning] Could not update config.xml: {e}")

class AsyncTelemetryLogger:
    """High-performance, non-blocking asynchronous telemetry logger writing pure real-time CSV."""
    def __init__(self, filename=None):
        if filename is None:
            self.filename = get_next_flight_log_filename()
        else:
            self.filename = filename if filename.endswith('.csv') else f"{filename}.csv"
            
        self.queue = queue.Queue(maxsize=10000)
        self.is_running = False
        self.worker_thread = None
        self.record_count = 0
        self.csv_file = None
        self.csv_writer = None
        self.headers = [
            "Record Number", "ESP32 Timestamp (ms)", "Elapsed Time (s)", "Packet Seq",
            "Pitch (deg)", "Roll (deg)", "Yaw (deg)",
            "Accel X (LSB)", "Accel Y (LSB)", "Accel Z (LSB)",
            "Gyro X (LSB)", "Gyro Y (LSB)", "Gyro Z (LSB)",
            "RC1 Roll (us)", "RC2 Pitch (us)", "RC3 Throttle (us)", "RC5 Mode (us)",
            "Servo BR (us)", "Servo BL (us)", "Servo FR (us)", "Servo FL (us)", "ESC Throttle (us)",
            "Battery Voltage (V)", "Altitude (m)", "RC Signal Lost", "Flaperons Active",
            "Latitude", "Longitude", "Satellites", "Fix Type", "RSSI (dBm)", "SNR (dB)",
            "Flight Mode", "ESC Active",
            "Pitch Kp", "Pitch Ki", "Pitch Kd",
            "Roll Kp", "Roll Ki", "Roll Kd"
        ]

    def start(self):
        if self.is_running:
            return
        os.makedirs(os.path.dirname(self.filename), exist_ok=True)
        self.csv_file = open(self.filename, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(self.headers)
        self.csv_file.flush()
        print(f"[Telemetry Logger] Novo ficheiro de voo iniciado em: '{self.filename}'.")

        self.is_running = True
        self.worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker_thread.start()

    def log_row(self, row):
        if self.is_running:
            try:
                self.queue.put_nowait(row)
            except queue.Full:
                pass

    def _worker_loop(self):
        last_flush = time.time()
        while self.is_running:
            try:
                row = self.queue.get(timeout=0.1)
                if row is not None and self.csv_writer is not None:
                    self.csv_writer.writerow(row)
                    self.record_count += 1
                self.queue.task_done()
            except queue.Empty:
                pass
            except Exception:
                pass

            # Drain batch
            while not self.queue.empty():
                try:
                    row = self.queue.get_nowait()
                    if row is not None and self.csv_writer is not None:
                        self.csv_writer.writerow(row)
                        self.record_count += 1
                    self.queue.task_done()
                except queue.Empty:
                    break
                except Exception:
                    pass

            now = time.time()
            if (now - last_flush) >= 1.0 and self.csv_file:
                last_flush = now
                try:
                    self.csv_file.flush()
                except Exception:
                    pass

    def stop(self):
        if not self.is_running:
            return
        self.is_running = False
        if self.worker_thread and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1.0)
        while not self.queue.empty():
            try:
                row = self.queue.get_nowait()
                if row is not None and self.csv_writer is not None:
                    self.csv_writer.writerow(row)
                    self.record_count += 1
                self.queue.task_done()
            except Exception:
                break
        if self.csv_file:
            try:
                self.csv_file.flush()
                self.csv_file.close()
            except Exception:
                pass
        print(f"[MANTA Ground Station] Registo final guardado em {self.filename} ({self.record_count} registos).")

def kill_mission_planner():
    """Terminates Mission Planner process cleanly and kills any running instances."""
    global mission_planner_proc
    if mission_planner_proc is not None:
        try:
            mission_planner_proc.terminate()
        except Exception:
            pass
    try:
        subprocess.run(["taskkill", "/f", "/im", "MissionPlanner.exe"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass

def global_shutdown(signum=None, frame=None):
    """Cleanly terminates Ground Station, saves Excel, closes Mission Planner, and exits immediately."""
    global active_serial_conn, excel_logger
    print("\n[MANTA Ground Station] A encerrar processos...")
    
    if active_serial_conn and active_serial_conn.is_open:
        try:
            active_serial_conn.close()
        except Exception:
            pass

    if excel_logger is not None:
        try:
            excel_logger.stop()
        except Exception:
            pass

    kill_mission_planner()
    print("[MANTA Ground Station] Terminado com sucesso.")
    os._exit(0)

def launch_mission_planner():
    """Ensures auto-connect settings and launches Mission Planner."""
    global mission_planner_proc
    ensure_mission_planner_autoconnect()
    mp_paths = [
        r'C:\Program Files (x86)\Mission Planner\MissionPlanner.exe',
        r'C:\Program Files\Mission Planner\MissionPlanner.exe',
        os.path.expandvars(r'%LOCALAPPDATA%\Apps\Mission Planner\MissionPlanner.exe'),
        os.path.expandvars(r'%USERPROFILE%\Desktop\Mission Planner.lnk')
    ]
    for p in mp_paths:
        if os.path.exists(p):
            try:
                mission_planner_proc = subprocess.Popen([p], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                print(f"[Mission Planner] Mission Planner aberto automaticamente a partir de: '{p}'")
                return True
            except Exception as e:
                print(f"[Mission Planner Launch Error] Falha ao abrir Mission Planner: {e}")

    print("[Mission Planner] Caminho do MissionPlanner.exe não foi encontrado automaticamente.")
    return False

def load_calibration():
    """Loads persistent local Ground Station parameters (horizon offsets, deadband alert, voltage cutoff) from JSON."""
    global pitch_offset, roll_offset, alert_voltage_threshold, rc_margin_deadband
    if os.path.exists(CALIB_FILE):
        try:
            with open(CALIB_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
                pitch_offset = float(data.get('pitch_offset', 0.0))
                roll_offset = float(data.get('roll_offset', 0.0))
                if 'deadband' in data:
                    rc_margin_deadband = int(data.get('deadband', 18))
                if 'cutoff' in data:
                    alert_voltage_threshold = float(data.get('cutoff', 12.50))
        except Exception as e:
            print(f"[IMU Calibration Error] Failed to load calibration file: {e}")

def save_calibration():
    """Saves persistent local Ground Station parameters to JSON file (Simplex Downlink architecture)."""
    global pitch_offset, roll_offset, alert_voltage_threshold, rc_margin_deadband
    try:
        data = {}
        if os.path.exists(CALIB_FILE):
            try:
                with open(CALIB_FILE, 'r', encoding='utf-8') as f:
                    data = json.load(f)
            except Exception:
                data = {}
        data['pitch_offset'] = round(pitch_offset, 2)
        data['roll_offset'] = round(roll_offset, 2)
        data['deadband'] = int(rc_margin_deadband)
        data['cutoff'] = round(alert_voltage_threshold, 2)
        with open(CALIB_FILE, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2)
    except Exception as e:
        print(f"[IMU Calibration Error] Failed to save calibration file: {e}")

def auto_find_com_port(preferred_port=None):
    """Detects available COM ports and selects the Ground Station ESP32 port."""
    if preferred_port:
        return preferred_port
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None
    for p in ports:
        if "COM4" in p.device:
            return "COM4"
    for p in ports:
        if "COM6" in p.device:
            return "COM6"
    for p in ports:
        if "COM5" not in p.device:
            return p.device
    return ports[0].device


# ==============================================================================
#                       TELEMETRY & MAVLINK BRIDGE LOOP
# ==============================================================================

def run_bridge(port_name=None, launch_mp=True):
    """Main CLI execution loop for LoRa Telemetry & MAVLink Mission Planner Bridge."""
    global active_serial_conn, latest_estimated_voltage, latest_pitch, latest_roll, latest_yaw
    global latest_alt, latest_lat, latest_lon, latest_temp, latest_satellites, latest_fix_type, latest_rc
    global last_rssi, last_snr, rc_signal_lost, excel_logger

    load_calibration()
    port = auto_find_com_port(port_name)
    if not port:
        print("[Erro] Nenhuma porta COM detetada! Conecte o ESP32 da Ground Station.")
        return

    print("=" * 60)
    print("      MANTA 20 Hz TELEMETRY & MISSION PLANNER BRIDGE")
    print("=" * 60)
    print(f"Porta COM Ground Station : {port} @ {DEFAULT_BAUD} baud")
    print(f"Ponte MAVLink UDP        : 127.0.0.1:14550")
    print("-" * 60)

    excel_logger = AsyncTelemetryLogger()
    excel_logger.start()

    try:
        ser = serial.Serial(port, DEFAULT_BAUD, timeout=0.01)
        active_serial_conn = ser
        print(f"[Conexão] Conectado com sucesso a {port}!")
    except Exception as e:
        print(f"[Erro Conexão] Falha ao abrir {port}: {e}")
        return

    mav_conn = None
    if HAS_PYMAVLINK:
        try:
            mav_conn = mavutil.mavlink_connection('udpout:127.0.0.1:14550', source_system=1, source_component=1)
            print("[MAVLink] Ligação UDP iniciada para 127.0.0.1:14550.")
        except Exception as e:
            print(f"[MAVLink Warning] Falha ao criar ligação MAVLink: {e}")

    if launch_mp:
        launch_mission_planner()

    raw_bytes_buffer = bytearray()
    buffer = ""
    last_mav_heartbeat = 0.0
    last_cli_print = 0.0
    last_system_time_tx = 0.0
    last_packet_rx_time = time.time()
    record_number = 1
    start_time = time.time()

    # Flight timer and state tracking
    cumulative_flight_time = 0.0
    last_flight_tick = start_time
    last_alt_calc = 0.0
    last_alt_time = start_time
    filtered_climb_rate = 0.0
    estimated_airspeed = 0.0
    latest_flight_mode = 1
    latest_flaperon_active = False
    last_notified_mode = None
    last_notified_flaperon_active = None

    print("\n[Telemetria Ativa - Simplex Downlink] A receber pacotes LoRa da MANTA... (Pressione Ctrl+C para sair)\n")

    try:
        while True:
            # Safe serial read with error handling and auto-reconnect
            try:
                num_bytes = ser.in_waiting
                if num_bytes > 0:
                    raw_data = ser.read(num_bytes)
                    raw_bytes_buffer.extend(raw_data)

                    # Bound binary buffer to prevent unbounded memory growth
                    if len(raw_bytes_buffer) > 2048:
                        raw_bytes_buffer = raw_bytes_buffer[-512:]

                    # Extract binary telemetry packets (Magic header: 'MT' -> 0x4D, 0x54)
                    while len(raw_bytes_buffer) >= min(SUPPORTED_PACKET_SIZES):
                        header_idx = raw_bytes_buffer.find(b'MT')
                        if header_idx == -1:
                            if len(raw_bytes_buffer) > 0 and raw_bytes_buffer[-1] == 0x4D:
                                raw_bytes_buffer = raw_bytes_buffer[-1:]
                            else:
                                raw_bytes_buffer.clear()
                            break
                        elif header_idx > 0:
                            raw_bytes_buffer = raw_bytes_buffer[header_idx:]

                        # Try candidate sizes starting from longest supported packet down
                        decoded_pkt = None
                        matched_size = 0
                        for cand_size in SUPPORTED_PACKET_SIZES_DESC:
                            if len(raw_bytes_buffer) >= cand_size:
                                candidate = bytes(raw_bytes_buffer[:cand_size])
                                res = decode_telemetry(candidate)
                                if res is not None:
                                    decoded_pkt = res
                                    matched_size = cand_size
                                    break

                        if decoded_pkt is not None:
                            last_packet_rx_time = time.time()
                            raw_bytes_buffer = raw_bytes_buffer[matched_size:]
                            latest_pitch = decoded_pkt.get("pitch", latest_pitch)
                            latest_roll = decoded_pkt.get("roll", latest_roll)
                            raw_bat = decoded_pkt.get("batteryVoltage", decoded_pkt.get("battery_v", None))
                            if raw_bat is not None and raw_bat > 0:
                                battery_voltage_history.append(raw_bat)
                                latest_estimated_voltage = round(sum(battery_voltage_history) / len(battery_voltage_history), 2)
                            latest_alt = decoded_pkt.get("alt", latest_alt)
                            latest_lat = decoded_pkt.get("latitude", decoded_pkt.get("lat", latest_lat))
                            latest_lon = decoded_pkt.get("longitude", decoded_pkt.get("lon", latest_lon))
                            latest_satellites = decoded_pkt.get("satellites", decoded_pkt.get("sats", latest_satellites))
                            latest_fix_type = decoded_pkt.get("fix_type", decoded_pkt.get("fixType", latest_fix_type))
                            
                            rc_vals = decoded_pkt.get("rc", None)
                            if rc_vals and len(rc_vals) >= 4:
                                latest_rc = [rc_vals[0], rc_vals[1], rc_vals[2], 1500, rc_vals[3]]
                            else:
                                latest_rc = [
                                    decoded_pkt.get("rc1", latest_rc[0]),
                                    decoded_pkt.get("rc2", latest_rc[1]),
                                    decoded_pkt.get("rc3", latest_rc[2]),
                                    1500,
                                    decoded_pkt.get("rc5", latest_rc[4])
                                ]

                            latest_flight_mode = decoded_pkt.get("flight_mode", decoded_pkt.get("flightMode", 1))
                            rc_signal_lost = decoded_pkt.get("rcSignalLost", decoded_pkt.get("rc_signal_lost", False))
                            is_flaperon_active = decoded_pkt.get("flaperon_active", decoded_pkt.get("flaperonActive", decoded_pkt.get("isAssistMode", False)))
                            latest_flaperon_active = is_flaperon_active

                            # Detect flight mode transition and display confirmation notice
                            if not rc_signal_lost:
                                if last_notified_mode is None:
                                    last_notified_mode = latest_flight_mode
                                    last_notified_flaperon_active = latest_flaperon_active
                                elif (latest_flight_mode != last_notified_mode or latest_flaperon_active != last_notified_flaperon_active):
                                    if latest_flight_mode == 1:
                                        m_tag = "M1:MAN" if not latest_flaperon_active else "M1:FLAP"
                                    elif latest_flight_mode == 2:
                                        m_tag = "M2:FBW" if not latest_flaperon_active else "M2:FLAP"
                                    elif latest_flight_mode == 3:
                                        m_tag = "M3:ESC"
                                    else:
                                        m_tag = f"M{latest_flight_mode}" + ("+FLAP" if latest_flaperon_active else "")
                                    sys.stdout.write(f"\n[PILOTO - MODO ALTERADO] -> {m_tag} (Feedback Sonoro 0.7s ativo)\n")
                                    sys.stdout.flush()
                                    last_notified_mode = latest_flight_mode
                                    last_notified_flaperon_active = latest_flaperon_active

                            now = time.time()
                            elapsed_sec = round(now - start_time, 2)
                            if excel_logger:
                                excel_logger.log_row([
                                    record_number,
                                    decoded_pkt.get("timestamp_ms", 0),
                                    elapsed_sec,
                                    decoded_pkt.get("pkt_seq", 0),
                                    round(latest_pitch, 2),
                                    round(latest_roll, 2),
                                    round(latest_yaw, 2),
                                    decoded_pkt.get("accel_x", 0),
                                    decoded_pkt.get("accel_y", 0),
                                    decoded_pkt.get("accel_z", 0),
                                    decoded_pkt.get("gyro_x", 0),
                                    decoded_pkt.get("gyro_y", 0),
                                    decoded_pkt.get("gyro_z", 0),
                                    latest_rc[0],
                                    latest_rc[1],
                                    latest_rc[2],
                                    latest_rc[4],
                                    decoded_pkt.get("servo_br", 1500),
                                    decoded_pkt.get("servo_bl", 1500),
                                    decoded_pkt.get("servo_fr", 1500),
                                    decoded_pkt.get("servo_fl", 1500),
                                    decoded_pkt.get("esc_throttle", 1000),
                                    latest_estimated_voltage,
                                    latest_alt,
                                    1 if rc_signal_lost else 0,
                                    1 if is_flaperon_active else 0,
                                    latest_lat,
                                    latest_lon,
                                    latest_satellites,
                                    latest_fix_type,
                                    last_rssi if last_rssi is not None else "",
                                    last_snr if last_snr is not None else "",
                                    decoded_pkt.get("flight_mode", decoded_pkt.get("flightMode", 1)),
                                    1 if decoded_pkt.get("isEscActive", False) else 0,
                                    decoded_pkt.get("pitch_kp", 9.35),
                                    decoded_pkt.get("pitch_ki", 5.00),
                                    decoded_pkt.get("pitch_kd", 0.623),
                                    decoded_pkt.get("roll_kp", 15.00),
                                    decoded_pkt.get("roll_ki", 5.00),
                                    decoded_pkt.get("roll_kd", 1.500)
                                ])
                                record_number += 1
                        else:
                            # Not a valid packet at current position
                            if len(raw_bytes_buffer) >= max(SUPPORTED_PACKET_SIZES):
                                raw_bytes_buffer = raw_bytes_buffer[1:]
                            else:
                                break

                    # Decode ASCII string messages (RSSI / SNR) with bounded string buffer
                    try:
                        text_chunk = raw_data.decode('latin-1', errors='ignore')
                        buffer = (buffer + text_chunk)[-256:]
                        while '\n' in buffer:
                            line_str, buffer = buffer.split('\n', 1)
                            line_str = line_str.strip()
                            if "RSSI:" in line_str:
                                m = re.search(r'RSSI:([-\d]+)', line_str)
                                if m:
                                    last_rssi = int(m.group(1))
                            if "SNR:" in line_str:
                                m = re.search(r'SNR:([-\d\.]+)', line_str)
                                if m:
                                    last_snr = float(m.group(1))
                    except Exception:
                        pass
            except (serial.SerialException, OSError) as e:
                print(f"\n[Aviso Porta Série] Erro de I/O em {port}: {e}. A tentar reconectar...")
                try:
                    ser.close()
                except Exception:
                    pass
                time.sleep(1.0)
                try:
                    ser = serial.Serial(port, DEFAULT_BAUD, timeout=0.01)
                    active_serial_conn = ser
                    print(f"[Conexão] Reconectado com sucesso a {port}!")
                except Exception:
                    pass
                continue

            now_time = time.time()

            # Watchdog: If no packet received for > 3s, clear stale serial buffer
            if (now_time - last_packet_rx_time) > 3.0:
                try:
                    if ser.in_waiting > 256:
                        ser.reset_input_buffer()
                        raw_bytes_buffer.clear()
                        buffer = ""
                except Exception:
                    pass

            now_time = time.time()

            # Flight state & dynamics calculations (Airspeed, Climb Rate, Flight Time)
            dt_flight = now_time - last_flight_tick
            last_flight_tick = now_time
            throttle_pct = int(max(0, min(100, (latest_rc[2] - 1000) / 10))) if len(latest_rc) > 2 else 0

            # Dynamic flight speed estimation (Mission Planner requires airspeed > 3 m/s to count Time in Air)
            estimated_airspeed = max(5.0, (throttle_pct / 100.0) * 18.0)

            # Accumulate flight time when throttle is active (> 5%)
            if throttle_pct > 5:
                cumulative_flight_time += dt_flight

            # Climb rate from barometric altitude derivative (low-pass filtered)
            dt_alt = now_time - last_alt_time
            if dt_alt >= 0.1:
                raw_climb = (latest_alt - last_alt_calc) / dt_alt
                filtered_climb_rate = 0.7 * filtered_climb_rate + 0.3 * raw_climb
                last_alt_calc = latest_alt
                last_alt_time = now_time

            # MAVLink 20 Hz Streaming
            if mav_conn and (now_time - last_mav_heartbeat) >= 0.05:
                last_mav_heartbeat = now_time
                try:
                    mav_state = mavutil.mavlink.MAV_STATE_EMERGENCY if rc_signal_lost else mavutil.mavlink.MAV_STATE_ACTIVE
                    mav_conn.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_FIXED_WING,
                        mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                        mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED | mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
                        0,
                        mav_state
                    )

                    # Explicitly inform Mission Planner that the aircraft is IN_AIR to count Time in Air
                    mav_conn.mav.extended_sys_state_send(
                        mavutil.mavlink.MAV_VTOL_STATE_UNDEFINED,
                        mavutil.mavlink.MAV_LANDED_STATE_IN_AIR
                    )

                    batt_mv = int(max(0.0, latest_estimated_voltage) * 1000)
                    batt_pct = calculate_battery_pct(latest_estimated_voltage)
                    sensors_mask = (mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO | mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL | mavutil.mavlink.MAV_SYS_STATUS_SENSOR_BATTERY)
                    mav_conn.mav.sys_status_send(sensors_mask, sensors_mask, sensors_mask, 500, batt_mv, -1, batt_pct, 0, 0, 0, 0, 0, 0)
                    mav_conn.mav.battery_status_send(
                        0,
                        mavutil.mavlink.MAV_BATTERY_FUNCTION_ALL,
                        mavutil.mavlink.MAV_BATTERY_TYPE_LIPO,
                        int(latest_temp * 100),
                        [batt_mv, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535],
                        -1,
                        -1,
                        -1,
                        batt_pct
                    )

                    # Monotonic milliseconds elapsed since boot
                    time_boot_ms = int((now_time - start_time) * 1000) & 0xFFFFFFFF
                    time_usec = int(now_time * 1e6) & 0xFFFFFFFFFFFFFFFF
                    alt_m = float(latest_alt)
                    alt_mm = int(alt_m * 1000)

                    mav_conn.mav.attitude_send(
                        time_boot_ms,
                        math.radians(latest_roll),
                        math.radians(latest_pitch),
                        math.radians(latest_yaw),
                        0.0, 0.0, 0.0
                    )
                    mav_conn.mav.vfr_hud_send(
                        float(estimated_airspeed),   # Estimated Airspeed (> 3.0 m/s to trigger MP Time in Air)
                        float(estimated_airspeed),   # Estimated Groundspeed
                        int(latest_yaw % 360),
                        throttle_pct,
                        alt_m,
                        float(filtered_climb_rate)
                    )

                    vz_cms = int(-filtered_climb_rate * 100)  # NED: down is positive

                    mav_conn.mav.global_position_int_send(
                        time_boot_ms,
                        int(latest_lat * 1e7),
                        int(latest_lon * 1e7),
                        alt_mm,
                        alt_mm,
                        0, 0, vz_cms,
                        int((latest_yaw % 360) * 100)
                    )

                    mav_conn.mav.gps_raw_int_send(
                        time_usec,
                        int(latest_fix_type),
                        int(latest_lat * 1e7),
                        int(latest_lon * 1e7),
                        alt_mm,
                        65535,  # eph
                        65535,  # epv
                        int(estimated_airspeed * 100),  # vel in cm/s
                        int((latest_yaw % 360) * 100),
                        int(latest_satellites)
                    )
                    mav_conn.mav.altitude_send(
                        time_usec,
                        alt_m,
                        alt_m,
                        alt_m,
                        alt_m,
                        alt_m,
                        alt_m
                    )
                    mav_conn.mav.scaled_pressure_send(
                        time_boot_ms,
                        1013.25,
                        0.0,
                        int(latest_temp * 100)
                    )
                    mav_conn.mav.rc_channels_raw_send(
                        time_boot_ms,
                        0,
                        latest_rc[0], latest_rc[1], latest_rc[2], latest_rc[3],
                        latest_rc[4], 0, 0, 0, 255
                    )
                except Exception:
                    pass

                # Periodic 1 Hz SYSTEM_TIME message
                if (now_time - last_system_time_tx) >= 1.0:
                    last_system_time_tx = now_time
                    try:
                        mav_conn.mav.system_time_send(time_usec, time_boot_ms)
                    except Exception:
                        pass

                # Drain incoming requests from Mission Planner (Parameters, Commands)
                try:
                    while True:
                        msg = mav_conn.recv_msg()
                        if msg is None:
                            break
                        m_type = msg.get_type()
                        
                        runtime_sec = float(now_time - start_time)
                        flttime_sec = float(cumulative_flight_time)
                        param_dict = {
                            b"SYSID_THISMAV": (1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
                            b"STAT_RUNTIME": (runtime_sec, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
                            b"STAT_FLTTIME": (flttime_sec, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
                            b"BATT_CAPACITY": (2200.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
                            b"BATT_ARM_VOLT": (11.1, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
                            b"ARMING_CHECK": (0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
                            b"FRAME_CLASS": (1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
                        }
                        param_keys = list(param_dict.keys())
                        total_params = len(param_keys)

                        if m_type == 'PARAM_REQUEST_LIST':
                            for idx, p_name in enumerate(param_keys):
                                val, p_type = param_dict[p_name]
                                mav_conn.mav.param_value_send(
                                    p_name,
                                    val,
                                    p_type,
                                    total_params,
                                    idx
                                )
                        elif m_type == 'PARAM_REQUEST_READ':
                            req_param_id = getattr(msg, 'param_id', b'')
                            if isinstance(req_param_id, str):
                                req_param_id = req_param_id.encode('latin-1')
                            req_idx = getattr(msg, 'param_index', -1)
                            
                            matched = False
                            if 0 <= req_idx < total_params:
                                p_name = param_keys[req_idx]
                                val, p_type = param_dict[p_name]
                                mav_conn.mav.param_value_send(p_name, val, p_type, total_params, req_idx)
                                matched = True
                            else:
                                for idx, p_name in enumerate(param_keys):
                                    if p_name in req_param_id or req_param_id in p_name:
                                        val, p_type = param_dict[p_name]
                                        mav_conn.mav.param_value_send(p_name, val, p_type, total_params, idx)
                                        matched = True
                                        break
                            if not matched:
                                mav_conn.mav.param_value_send(b"STAT_RUNTIME", runtime_sec, mavutil.mavlink.MAV_PARAM_TYPE_REAL32, total_params, 0)
                        elif m_type == 'COMMAND_LONG':
                            mav_conn.mav.command_ack_send(msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED)
                except Exception:
                    pass

            # Terminal Status line at 4 Hz
            if (now_time - last_cli_print) >= 0.25:
                last_cli_print = now_time
                batt_pct = calculate_battery_pct(latest_estimated_voltage)
                rssi_display = f"{last_rssi}dBm" if last_rssi is not None else "--"
                snr_display = f"{last_snr}dB" if last_snr is not None else "--"
                flt_min = int(cumulative_flight_time // 60)
                flt_sec = int(cumulative_flight_time % 60)
                # Flight mode display: prefer confirmed telemetry from aircraft with fallback to RC5
                if latest_flight_mode in (1, 2, 3):
                    mode_num = latest_flight_mode
                    flaperons_on = latest_flaperon_active
                else:
                    mode_num, flaperons_on, _ = decode_ch5_mode(latest_rc[4])

                if mode_num == 1:
                    mode_tag = "M1:MAN" if not flaperons_on else "M1:FLAP"
                elif mode_num == 2:
                    mode_tag = "M2:FBW" if not flaperons_on else "M2:FLAP"
                elif mode_num == 3:
                    mode_tag = "M3:ESC"
                else:
                    mode_tag = f"M{mode_num}" + ("+FLAP" if flaperons_on else "")

                climb_display = f"{filtered_climb_rate:+4.1f}m/s"
                sys.stdout.write(f"\r[MANTA] Modo: {mode_tag:<11} | Voo: {flt_min:02d}:{flt_sec:02d} | Bat: {latest_estimated_voltage:.2f}V ({batt_pct}%) | Alt: {latest_alt:+5.1f}m ({climb_display}) | LoRa: {rssi_display} / {snr_display}   ")
                sys.stdout.flush()

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n\n[Bridge] Terminado pelo utilizador.")
    except Exception as e:
        import traceback
        print(f"\n\n[Bridge Error] Erro inesperado: {e}")
        traceback.print_exc()
    finally:
        global_shutdown()



def main():
    import argparse
    parser = argparse.ArgumentParser(description="MANTA 20 Hz LoRa Telemetry & Mission Planner Bridge")
    parser.add_argument("--port", "-p", type=str, default=None, help="Porta Serial COM (ex: COM4)")
    parser.add_argument("--no-mp", action="store_true", help="Não iniciar o Mission Planner automaticamente")
    args = parser.parse_args()

    run_bridge(port_name=args.port, launch_mp=not args.no_mp)


if __name__ == "__main__":
    main()
