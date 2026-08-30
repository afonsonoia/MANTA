import serial
import serial.tools.list_ports
import time
import openpyxl
from openpyxl import Workbook
import os
import math
import re
import json
import subprocess
import socket
import queue
import threading
import sys

try:
    from pymavlink import mavutil
    HAS_PYMAVLINK = True
except ImportError:
    HAS_PYMAVLINK = False

from telemetry_codec import decode_telemetry, encode_telemetry, PACKET_SIZE

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
    """Scans flight_logs/ directory and returns the next sequential flight log filename."""
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
    filename = f"manta_flight_{next_idx:04d}_{timestamp_str}.xlsx"
    return os.path.join(log_dir, filename)

# Global state
active_serial_conn = None
last_rssi = None
last_snr = None
latest_estimated_voltage = 12.50

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

class AsyncExcelTelemetryLogger:
    """High-performance, non-blocking asynchronous Excel telemetry logger."""
    def __init__(self, filename=None, auto_save_interval=10.0):
        if filename is None:
            self.filename = get_next_flight_log_filename()
        else:
            self.filename = filename
        self.auto_save_interval = auto_save_interval
        self.queue = queue.Queue()
        self.is_running = False
        self.worker_thread = None
        self.wb = None
        self.ws = None
        self.record_count = 0
        self.last_save_time = 0.0

    def start(self):
        if self.is_running:
            return
        os.makedirs(os.path.dirname(self.filename), exist_ok=True)

        self.wb = Workbook()
        self.ws = self.wb.active
        self.ws.title = "MANTA Mission Telemetry Log"
        self.ws.append(["Record Number", "Elapsed Time (s)", "Accel Z (LSB)", "Battery Voltage (V)", "Pitch (deg)", "Roll (deg)", "Yaw (deg)", "Latitude", "Longitude", "Altitude (m)", "Satellites", "Fix Type", "RSSI", "SNR"])
        try:
            self.wb.save(self.filename)
            print(f"[Telemetry Logger] Novo ficheiro de voo iniciado em: '{self.filename}'.")
        except Exception as e:
            print(f"[Warning] Initial save of '{self.filename}' failed: {e}")

        self.is_running = True
        self.last_save_time = time.time()
        self.worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker_thread.start()

    def log_row(self, row):
        if self.is_running:
            try:
                self.queue.put_nowait(row)
            except queue.Full:
                pass

    def _worker_loop(self):
        while self.is_running:
            try:
                row = self.queue.get(timeout=0.2)
                if row is not None and self.ws is not None:
                    self.ws.append(row)
                    self.record_count += 1
                self.queue.task_done()
            except queue.Empty:
                pass
            except Exception:
                pass

            while not self.queue.empty():
                try:
                    row = self.queue.get_nowait()
                    if row is not None and self.ws is not None:
                        self.ws.append(row)
                        self.record_count += 1
                    self.queue.task_done()
                except queue.Empty:
                    break
                except Exception:
                    pass

            now = time.time()
            if (now - self.last_save_time) >= self.auto_save_interval and self.record_count > 0:
                self.last_save_time = now
                try:
                    self.wb.save(self.filename)
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
                if row is not None and self.ws is not None:
                    self.ws.append(row)
                    self.record_count += 1
                self.queue.task_done()
            except Exception:
                break
        if self.wb is not None:
            try:
                self.wb.save(self.filename)
                print(f"[MANTA Ground Station] Registo final guardado em {self.filename} ({self.record_count} registos).")
            except Exception as e:
                print(f"[Warning] Falha ao guardar {self.filename} no encerramento: {e}")

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
    """Loads persistent zero-horizon calibration offsets from JSON file."""
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
    """Saves persistent calibration offsets to JSON file."""
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

    excel_logger = AsyncExcelTelemetryLogger()
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
    record_number = 1
    start_time = time.time()

    print("\n[Telemetria Ativa] A receber pacotes LoRa da MANTA... (Pressione Ctrl+C para sair)\n")

    try:
        while True:
            num_bytes = ser.in_waiting
            if num_bytes > 0:
                raw_data = ser.read(num_bytes)
                raw_bytes_buffer.extend(raw_data)

                # Extract binary telemetry packets (Magic header: 'MT' -> 0x4D, 0x54)
                while len(raw_bytes_buffer) >= PACKET_SIZE:
                    header_idx = -1
                    for i in range(len(raw_bytes_buffer) - 1):
                        if raw_bytes_buffer[i] == 0x4D and raw_bytes_buffer[i+1] == 0x54:
                            header_idx = i
                            break
                    
                    if header_idx == -1:
                        if len(raw_bytes_buffer) > 0 and raw_bytes_buffer[-1] == 0x4D:
                            raw_bytes_buffer = raw_bytes_buffer[-1:]
                        else:
                            raw_bytes_buffer.clear()
                        break
                    elif header_idx > 0:
                        raw_bytes_buffer = raw_bytes_buffer[header_idx:]

                    if len(raw_bytes_buffer) < PACKET_SIZE:
                        break

                    candidate = bytes(raw_bytes_buffer[:PACKET_SIZE])
                    decoded_pkt = decode_telemetry(candidate)
                    if decoded_pkt is not None:
                        raw_bytes_buffer = raw_bytes_buffer[PACKET_SIZE:]
                        latest_pitch = decoded_pkt.get("pitch", latest_pitch)
                        latest_roll = decoded_pkt.get("roll", latest_roll)
                        latest_estimated_voltage = decoded_pkt.get("batteryVoltage", decoded_pkt.get("battery_v", latest_estimated_voltage))
                        latest_alt = decoded_pkt.get("alt", latest_alt)
                        
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

                        rc_signal_lost = decoded_pkt.get("rcSignalLost", decoded_pkt.get("rc_signal_lost", False))

                        now = time.time()
                        elapsed_sec = round(now - start_time, 2)
                        if excel_logger:
                            excel_logger.log_row([
                                record_number, elapsed_sec, decoded_pkt.get("accel_z", 0), latest_estimated_voltage,
                                round(latest_pitch, 2), round(latest_roll, 2), round(latest_yaw, 2),
                                latest_lat, latest_lon, latest_alt, latest_satellites, latest_fix_type,
                                last_rssi if last_rssi else "", last_snr if last_snr else ""
                            ])
                            record_number += 1
                    else:
                        raw_bytes_buffer = raw_bytes_buffer[1:]

                # Decode ASCII string messages (RSSI / SNR)
                try:
                    text_chunk = raw_data.decode('latin-1')
                    buffer += text_chunk
                    while '\n' in buffer:
                        line_str, buffer = buffer.split('\n', 1)
                        line_str = line_str.strip()
                        if not line_str:
                            continue

                        if "RSSI:" in line_str:
                            m = re.search(r'RSSI:([-\d]+)', line_str)
                            if m: last_rssi = int(m.group(1))
                        if "SNR:" in line_str:
                            m = re.search(r'SNR:([-\d\.]+)', line_str)
                            if m: last_snr = float(m.group(1))
                except Exception:
                    pass

            now_time = time.time()

            # MAVLink 20 Hz Streaming
            if mav_conn and (now_time - last_mav_heartbeat) >= 0.05:
                last_mav_heartbeat = now_time
                try:
                    mav_state = mavutil.mavlink.MAV_STATE_EMERGENCY if rc_signal_lost else mavutil.mavlink.MAV_STATE_ACTIVE
                    mav_conn.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_FIXED_WING,
                        mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                        mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED | mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        0,
                        mav_state
                    )
                    batt_mv = int(max(0.0, latest_estimated_voltage) * 1000)
                    batt_pct = int(max(0, min(100, (latest_estimated_voltage - 10.5) / (12.6 - 10.5) * 100)))
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
                    mav_conn.mav.attitude_send(
                        int(now_time * 1000) & 0xFFFFFFFF,
                        math.radians(latest_roll),
                        math.radians(latest_pitch),
                        math.radians(latest_yaw),
                        0.0, 0.0, 0.0
                    )
                    mav_conn.mav.vfr_hud_send(0.0, 0.0, int(latest_yaw % 360), 50, 100.0, 0.0)
                    mav_conn.mav.rc_channels_raw_send(
                        int(now_time * 1000) & 0xFFFFFFFF,
                        0,
                        latest_rc[0], latest_rc[1], latest_rc[2], latest_rc[3],
                        latest_rc[4], 0, 0, 0, 255
                    )
                except Exception:
                    pass

            # Terminal Status line at 4 Hz
            if (now_time - last_cli_print) >= 0.25:
                last_cli_print = now_time
                batt_pct = int(max(0, min(100, (latest_estimated_voltage - 10.5) / (12.6 - 10.5) * 100)))
                rssi_display = f"{last_rssi}dBm" if last_rssi is not None else "--"
                snr_display = f"{last_snr}dB" if last_snr is not None else "--"
                sys.stdout.write(f"\r[MANTA] Bat: {latest_estimated_voltage:.2f}V ({batt_pct}%) | Pitch: {latest_pitch:+5.1f}° | Roll: {latest_roll:+5.1f}° | RC: CH1={latest_rc[0]} CH2={latest_rc[1]} CH3={latest_rc[2]} CH5={latest_rc[4]} | LoRa: {rssi_display} / {snr_display}   ")
                sys.stdout.flush()

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n\n[Bridge] Terminado pelo utilizador.")
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
