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

try:
    from pymavlink import mavutil
    HAS_PYMAVLINK = True
except ImportError:
    HAS_PYMAVLINK = False

from telemetry_codec import decode_telemetry, encode_telemetry

# Config
DEFAULT_BAUD = 115200
EXCEL_FILE = 'registo_manta_telemetry.xlsx'
CALIB_FILE = 'imu_calibration.json'

# Global state
gui_app = None
current_throttle_pulse = 1000  # Default off/armed pulse (1000us)
last_sent_pulse = None
last_sent_cutoff = None
active_serial_conn = None
low_voltage_cutoff_active = False
alarm_active = None  # None force-sends initial state on connect
last_rssi = None
last_snr = None
last_manta_confirmed_cutoff = None
last_manta_confirmed_deadband = None
latest_estimated_voltage = 12.50

# Current IMU, GPS & Baro state
latest_pitch = 0.0
latest_roll = 0.0
latest_yaw = 0.0
latest_lat = 0.0
latest_lon = 0.0
latest_alt = 0.0
latest_temp = 25.0
latest_satellites = 0
latest_fix_type = 0
pitch_offset = 0.0
roll_offset = 0.0

is_manta_calib_mode = False
rc_signal_lost = False
latest_rc = [1500, 1500, 1000, 1500, 1500]

# Configurable Low Voltage Alert Threshold (Default: 12.50V)
alert_voltage_threshold = 12.50

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


def launch_mission_planner():
    """Ensures auto-connect settings and launches Mission Planner."""
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
                subprocess.Popen([p], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                print(f"[Mission Planner] Mission Planner aberto automaticamente a partir de: '{p}'")
                return True
            except Exception as e:
                print(f"[Mission Planner Launch Error] Falha ao abrir Mission Planner: {e}")

    print("[Mission Planner] Caminho do MissionPlanner.exe não foi encontrado automaticamente.")
    return False

def send_version_and_banner(mav_conn):
    """Sends ArduPilot firmware version and banner to satisfy Mission Planner VersionDetection."""
    if not mav_conn:
        return
    try:
        mav_conn.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_INFO,
            b"ArduPlane v4.5.0 (bd324d31)"
        )
        mav_conn.mav.autopilot_version_send(
            capabilities=mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT | mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT,
            flight_sw_version=0x04050000,
            middleware_sw_version=0,
            os_sw_version=0,
            board_version=0,
            flight_custom_version=b"4.5.0\x00\x00\x00",
            middleware_custom_version=b"\x00"*8,
            os_custom_version=b"\x00"*8,
            vendor_id=0,
            product_id=0,
            uid=0
        )
    except Exception:
        pass

def send_battery_parameters(mav_conn, req_param_id=None, req_param_index=-1):
    """Sends battery monitor parameters to Mission Planner so the voltage gauge displays on the HUD."""
    if not mav_conn:
        return
    try:
        params = [
            (b"BATT_MONITOR", 4.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (b"BATT_VOLT_MULT", 1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (b"BATT_CAPACITY", 2200.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (b"BATT_ARM_VOLT", 10.5, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (b"STAT_RUNTIME", 1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
        ]
        total = len(params)
        if req_param_index >= 0 and req_param_index < total:
            p_name, p_val, p_type = params[req_param_index]
            mav_conn.mav.param_value_send(p_name, p_val, p_type, total, req_param_index)
        elif req_param_id:
            req_str = req_param_id.decode('utf-8', errors='ignore').rstrip('\x00') if isinstance(req_param_id, bytes) else str(req_param_id).rstrip('\x00')
            for idx, (p_name, p_val, p_type) in enumerate(params):
                if p_name.decode('utf-8').rstrip('\x00') == req_str:
                    mav_conn.mav.param_value_send(p_name, p_val, p_type, total, idx)
                    return
            mav_conn.mav.param_value_send(params[0][0], params[0][1], params[0][2], total, 0)
        else:
            for idx, (p_name, p_val, p_type) in enumerate(params):
                mav_conn.mav.param_value_send(p_name, p_val, p_type, total, idx)
    except Exception:
        pass

def load_calibration():
    """Loads persistent zero-horizon calibration offsets from JSON file."""
    global pitch_offset, roll_offset
    if os.path.exists(CALIB_FILE):
        try:
            with open(CALIB_FILE, 'r') as f:
                data = json.load(f)
                pitch_offset = float(data.get('pitch_offset', 0.0))
                roll_offset = float(data.get('roll_offset', 0.0))
                print(f"[IMU Calibration] Persistent calibration loaded: Pitch Offset={pitch_offset:.2f} deg, Roll Offset={roll_offset:.2f} deg")
        except Exception as e:
            print(f"[IMU Calibration Error] Failed to load calibration file: {e}")

def get_calibrated_angles(raw_p, raw_r):
    """Computes zero-calibrated pitch and roll, automatically handling upright and upside-down sensor mountings."""
    if abs(raw_p) < 0.001 and abs(raw_r) < 0.001:
        return 0.0, 0.0
    if abs(roll_offset) > 90.0:
        diff_p = raw_p - pitch_offset
        diff_r = (raw_r - roll_offset + 180.0) % 360.0 - 180.0
        c_pitch = diff_p
        c_roll  = diff_r
    else:
        c_pitch = -((raw_p - pitch_offset + 180.0) % 360.0 - 180.0)
        c_roll  = -((raw_r - roll_offset  + 180.0) % 360.0 - 180.0)
    return c_pitch, c_roll

pending_config_commands = []

def send_short_beep():
    """Emits 1 short beep (60ms) on the Ground Station ESP32 buzzer."""
    global active_serial_conn
    if active_serial_conn and active_serial_conn.is_open:
        try:
            active_serial_conn.write(b"BEEP:SHORT\n")
            active_serial_conn.flush()
        except Exception:
            pass

def check_and_flush_pending_commands():
    """Flushes queued configuration commands as soon as MANTA enters Calibration Mode (CH5 > 1900)."""
    global pending_config_commands, active_serial_conn
    if is_debug_mode_active() and pending_config_commands:
        if active_serial_conn and active_serial_conn.is_open:
            print(f"\n[Pending Queue Flushed] MANTA Calibration Mode (CH5 > 1900) activated! Transmitting {len(pending_config_commands)} queued commands...")
            for cmd in list(pending_config_commands):
                try:
                    active_serial_conn.write(cmd.encode('utf-8'))
                    active_serial_conn.flush()
                    print(f"  └─> [Queued Command Sent] {cmd.strip()}")
                    time.sleep(0.02)
                except Exception as e:
                    print(f"  └─> [Error Sending Queued Command] {e}")
            pending_config_commands.clear()

def is_debug_mode_active():
    """Always returns True for immediate direct command transmission to MANTA."""
    return True

def send_alarm_command(state_intermittent, force=False):
    """Sends intermittent alarm state (50% 2kHz) to Ground Station over Serial."""
    global alarm_active, active_serial_conn
    if active_serial_conn and active_serial_conn.is_open:
        try:
            if state_intermittent and (alarm_active != True or force):
                active_serial_conn.write(b"BEEP:INTERMITTENT\n")
                active_serial_conn.flush()
                alarm_active = True
                print("[LoRa Logger] Sent Alarm ON -> BEEP:INTERMITTENT (50% 2kHz)")
            elif not state_intermittent and (alarm_active != False or force):
                active_serial_conn.write(b"BEEP:OFF\n")
                active_serial_conn.flush()
                alarm_active = False
                print("[LoRa Logger] Sent Alarm OFF -> BEEP:OFF")
        except Exception as e:
            print(f"[Error Alarm] {e}")

def send_throttle_command(pulse, force=False):
    """Sends a throttle PWM command (1000us - 2000us) over LoRa Serial ONLY if in debug mode (CH5 > 1900)."""
    global current_throttle_pulse, last_sent_pulse, active_serial_conn, low_voltage_cutoff_active
    
    if not is_debug_mode_active():
        return  # Block outbound command in flight mode (CH5 <= 1900) -> strictly 1-way MANTA -> GS

    if low_voltage_cutoff_active:
        pulse = 1000

    current_throttle_pulse = int(pulse)
    if active_serial_conn and active_serial_conn.is_open and (current_throttle_pulse != last_sent_pulse or force):
        try:
            cmd = f"THROTTLE:{current_throttle_pulse}\n"
            active_serial_conn.write(cmd.encode('utf-8'))
            active_serial_conn.flush()
            last_sent_pulse = current_throttle_pulse
            print(f"[LoRa TX] Sent ESC Throttle Command: {current_throttle_pulse} us")
        except Exception as e:
            print(f"[LoRa TX Error] Failed to send throttle command: {e}")

def send_cutoff_command(val, force=False):
    """Sends low-voltage cutoff threshold command to MANTA. Queues command if in flight mode (CH5 <= 1900)."""
    global last_sent_cutoff, active_serial_conn, pending_config_commands
    cutoff_v = round(float(val), 2)
    cmd = f"CUTOFF:{cutoff_v:.2f}\n"

    if not is_debug_mode_active():
        if cmd not in pending_config_commands:
            pending_config_commands.append(cmd)
            send_short_beep()
            print(f"[Pending Queue] 1 Short Beep emitted! Queued '{cmd.strip()}' until MANTA Calibration Mode (CH5 > 1900) is activated.")
        return

    try:
        if active_serial_conn and active_serial_conn.is_open and (cutoff_v != last_sent_cutoff or force):
            active_serial_conn.write(cmd.encode('utf-8'))
            active_serial_conn.flush()
            last_sent_cutoff = cutoff_v
            print(f"[LoRa TX] Sent Cutoff Threshold Command to MANTA: {cutoff_v:.2f} V")
    except Exception as e:
        print(f"[LoRa TX Error] Failed to send cutoff command: {e}")

last_sent_deadband = None

def send_deadband_command(val, force=False):
    """Sends RC deadband command to MANTA. Queues command if in flight mode (CH5 <= 1900)."""
    global last_sent_deadband, active_serial_conn, pending_config_commands
    db_val = int(val)
    cmd = f"SET_DEADBAND:{db_val}\n"

    if not is_debug_mode_active():
        if cmd not in pending_config_commands:
            pending_config_commands.append(cmd)
            send_short_beep()
            print(f"[Pending Queue] 1 Short Beep emitted! Queued '{cmd.strip()}' until MANTA Calibration Mode (CH5 > 1900) is activated.")
        return

    try:
        if active_serial_conn and active_serial_conn.is_open and (db_val != last_sent_deadband or force):
            active_serial_conn.write(cmd.encode('utf-8'))
            active_serial_conn.flush()
            last_sent_deadband = db_val
            print(f"[LoRa TX] Sent RC Deadband Command to MANTA: {db_val} us")
    except Exception as e:
        print(f"[LoRa TX Error] Failed to send deadband command: {e}")

last_adaptive_power = None
last_adaptive_power_change_time = 0.0

def evaluate_adaptive_manta_power(rssi):
    """Dynamically adjusts MANTA LoRa Tx Power based on real-time RSSI signal quality (ADR Adaptive Data Rate)."""
    global last_adaptive_power, last_adaptive_power_change_time, active_serial_conn
    if not is_debug_mode_active():
        return  # In flight mode, MANTA self-manages Tx power (20 dBm max) without Ground Station commands

    if rssi is None:
        return
    now = time.time()
    # Hysteresis timer: avoid rapidly toggling Tx power (wait at least 4 seconds between adjustments)
    if (now - last_adaptive_power_change_time) < 4.0:
        return

    target_power = None
    if rssi <= -95:
        target_power = 20  # Weak signal: boost to maximum 20 dBm (PA_BOOST)
    elif rssi <= -82:
        target_power = 17  # Moderate signal: standard 17 dBm
    elif rssi > -75:
        target_power = 14  # Strong signal: conserve power at 14 dBm

    if target_power is not None and target_power != last_adaptive_power:
        if active_serial_conn and active_serial_conn.is_open:
            try:
                cmd = f"SET_LORA_POWER:{target_power}\n"
                active_serial_conn.write(cmd.encode('utf-8'))
                active_serial_conn.flush()
                old_pwr_str = f"{last_adaptive_power}dBm" if last_adaptive_power else "DEFAULT"
                print(f"[ADR ADAPTIVE LINK] RSSI is {rssi} dBm. Dynamic LoRa Tx Power adjusted: [{old_pwr_str}] -> [{target_power} dBm]")
                last_adaptive_power = target_power
                last_adaptive_power_change_time = now
            except Exception as e:
                print(f"[ADR Error] Failed to send adaptive power command: {e}")

def send_trim_calibration_command():
    """Sends calibration command (CALIB_TRIM) to MANTA over LoRa to store neutral centers in NVS Flash."""
    global active_serial_conn
    if not is_debug_mode_active():
        print("[LoRa TX Blocked] Calibration command blocked: MANTA is in Flight Mode (CH5 <= 1900). Enable Debug Mode (CH5 > 1900).")
        return False

    if active_serial_conn and active_serial_conn.is_open:
        try:
            active_serial_conn.write(b"CALIB_TRIM\n")
            active_serial_conn.flush()
            print("[LoRa TX] Sent Neutral Calibration Command (CALIB_TRIM) to MANTA ESP32!")
            return True
        except Exception as e:
            print(f"[LoRa TX Error] Failed to send neutral calibration command: {e}")
    else:
        print("[LoRa TX Warning] Cannot send calibration command: Serial connection not open!")
    return False

def auto_find_com_port():
    """Detects available COM ports and selects the ESP32 port."""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None
    for p in ports:
        if "COM4" in p.device:
            return "COM4"
    return ports[0].device

def main():
    global active_serial_conn, current_throttle_pulse, low_voltage_cutoff_active, alert_voltage_threshold, last_rssi, last_snr
    global latest_pitch, latest_roll, latest_yaw, latest_lat, latest_lon, latest_alt, latest_satellites, latest_fix_type, latest_rc
    global last_manta_confirmed_cutoff, latest_estimated_voltage

    latest_rc = [0, 0, 0, 0, 0]
    rc_signal_lost = True  # Assume lost until first telemetry received
    last_rc_signal_warn_time = 0.0


    print("==================================================")
    print("        MANTA MISSION PLANNER LORA BRIDGE         ")
    print("==================================================")
    
    load_calibration()

    print(f"Audio alarm configured: <= {alert_voltage_threshold:.2f}V (Intermittent 50% 2kHz)")
    print(f"MANTA Safety Threshold: MAX(12.00V, GS Cutoff Voltage)")
    print(f"Telemetry rate       : 0.2s (5 Hz LoRa Broadcast)")
    print(f"Excel Log File       : {EXCEL_FILE}")
    print("==================================================\n")
    
    port_name = auto_find_com_port()
    if not port_name:
        print("[Error] No COM port found! Connect Ground Station ESP32.")
        return

    print(f"Connecting to Ground Station on port {port_name} @ {DEFAULT_BAUD} baud...")
    
    try:
        ser = serial.Serial(port_name, DEFAULT_BAUD, timeout=0.01)
        active_serial_conn = ser
        print(f"Successfully connected to port {port_name}!\n")
        
        send_alarm_command(False, force=True)
        send_cutoff_command(alert_voltage_threshold, force=True)
        send_deadband_command(25, force=True)
    except Exception as e:
        print(f"[Error] Could not open {port_name}: {e}")
        return

    records = []
    timestamps = []
    raw_adcs = []
    start_time = None
    last_excel_log_time = 0.0
    last_terminal_print_time = 0.0

    if os.path.exists(EXCEL_FILE):
        try:
            os.remove(EXCEL_FILE)
            print(f"Previous log file '{EXCEL_FILE}' deleted successfully.")
        except Exception as e:
            print(f"[Warning] Could not delete previous log file '{EXCEL_FILE}': {e}")

    wb = Workbook()
    ws = wb.active
    ws.title = "MANTA Mission Telemetry Log"
    ws.append(["Record Number", "Elapsed Time (s)", "Raw Sensor (ADC)", "Battery Voltage (V)", "Pitch (deg)", "Roll (deg)", "Yaw (deg)", "Latitude", "Longitude", "Altitude (m)", "Satellites", "Fix Type"])
    record_number = 1
    wb.save(EXCEL_FILE)
    print(f"Created new clean log file '{EXCEL_FILE}'.")

    send_throttle_command(current_throttle_pulse, force=True)
    buffer = ""
    raw_bytes_buffer = bytearray()

    mav_conn = None
    last_mav_heartbeat = 0.0
    mp_launched = False
    last_gps_wait_print = 0.0

    # Mission Planner Home Position & State
    home_lat_int = 0
    home_lon_int = 0
    home_alt_mm = 0
    home_position_set = False

    # Default RC channels and signal loss state
    latest_rc = [1500, 1500, 1000, 1500, 1500]
    rc_signal_lost = False
    last_rc_signal_warn_time = 0.0

    if HAS_PYMAVLINK:
        try:
            mav_conn = mavutil.mavlink_connection('udpout:127.0.0.1:14550', source_system=1, source_component=1)
            if hasattr(socket, 'SIO_UDP_CONNRESET'):
                try:
                    mav_conn.port.ioctl(socket.SIO_UDP_CONNRESET, False)
                except Exception:
                    pass
            print("[MAVLink Bridge] Emissor MAVLink UDP ativo em 127.0.0.1:14550!")
            print("[Mission Planner] A abrir o Mission Planner automaticamente...")
            launch_mission_planner()
            mp_launched = True
        except Exception as e:
            print(f"[MAVLink Bridge Error] Falha ao iniciar ligação MAVLink: {e}")


    print("\n[Mission Planner Bridge] Streaming Telemetry. Press Ctrl+C to terminate.\n")

    while True:
        try:
            if ser is None or not ser.is_open:
                try:
                    p_name = auto_find_com_port()
                    if p_name:
                        ser = serial.Serial(p_name, DEFAULT_BAUD, timeout=0.01)
                        active_serial_conn = ser
                        print(f"[Serial Auto-Reconnect] Ligação restabelecida na porta {p_name}!")
                except Exception:
                    pass

            if ser and ser.is_open:
                try:
                    in_w = ser.in_waiting
                except Exception:
                    in_w = 0
                    try:
                        ser.close()
                    except Exception:
                        pass
                    ser = None
                    active_serial_conn = None


                if in_w > 0:
                    try:
                        data = ser.read(in_w)
                    except Exception:
                        data = None

                    if data:
                        raw_bytes_buffer.extend(data)
                        binary_telemetry_updated = False

                        while len(raw_bytes_buffer) >= 42:
                            idx = raw_bytes_buffer.find(b'MT')
                            if idx == -1:
                                if len(raw_bytes_buffer) > 1:
                                    raw_bytes_buffer = raw_bytes_buffer[-1:]
                                break
                            if idx > 0:
                                raw_bytes_buffer = raw_bytes_buffer[idx:]
                            if len(raw_bytes_buffer) < 42:
                                break

                            pkt_bin = bytes(raw_bytes_buffer[:42])
                            decoded_pkt = decode_telemetry(pkt_bin)
                            if decoded_pkt is not None:
                                latest_estimated_voltage = decoded_pkt["batteryVoltage"]
                                latest_pitch = decoded_pkt["pitch"]
                                latest_roll = decoded_pkt["roll"]
                                latest_yaw = decoded_pkt["yaw"]
                                last_manta_confirmed_cutoff = decoded_pkt["effectiveCutoff"]
                                last_manta_confirmed_deadband = decoded_pkt["deadband"]
                                latest_lat = decoded_pkt["lat"]
                                latest_lon = decoded_pkt["lon"]
                                latest_alt = decoded_pkt["alt"]
                                latest_temp = decoded_pkt["temp"]
                                latest_satellites = decoded_pkt["sats"]
                                latest_fix_type = decoded_pkt["fix"]
                                latest_rc = decoded_pkt["rc"]
                                rc_signal_lost = decoded_pkt["rcSignalLost"]
                                is_manta_calib_mode = decoded_pkt.get("isCalibMode", False)
                                binary_telemetry_updated = True
                                check_and_flush_pending_commands()
                                if gui_app is not None:
                                    try:
                                        gui_app._process_voltage_sample(decoded_pkt["batteryVoltage"], decoded_pkt["rawADC"])
                                    except Exception:
                                        pass
                                raw_bytes_buffer = raw_bytes_buffer[42:]
                            else:
                                raw_bytes_buffer = raw_bytes_buffer[1:]

                        buffer += data.decode('utf-8', errors='ignore')
                        while '\n' in buffer:
                            line_str, buffer = buffer.split('\n', 1)
                            line_str = line_str.strip()
                            if line_str:
                                if "ACK:" in line_str:
                                    try:
                                        ack_content = line_str[line_str.find("ACK:"):].split()[0]
                                        print(f"  [ACK RECEIVED] MANTA confirmed: {ack_content}")
                                        if gui_app is not None:
                                            gui_app.last_received_ack = ack_content
                                            gui_app.lbl_status.config(text=f"ACK: {ack_content}", fg=gui_app.ACCENT_GREEN)
                                    except Exception:
                                        pass

                                if "RSSI:" in line_str:
                                    try:
                                        rssi_match = re.search(r'RSSI:\s*(-?\d+)', line_str)
                                        if rssi_match:
                                            last_rssi = int(rssi_match.group(1))
                                            if gui_app is not None:
                                                gui_app.last_rssi = f"{last_rssi} dBm"
                                            evaluate_adaptive_manta_power(last_rssi)
                                    except Exception:
                                        pass

                                if "SNR:" in line_str:
                                    try:
                                        snr_match = re.search(r'SNR:\s*([\d\.-]+)', line_str)
                                        if snr_match:
                                            last_snr = float(snr_match.group(1))
                                            if gui_app is not None:
                                                gui_app.last_snr = f"{last_snr} dB"
                                    except Exception:
                                        pass

                                if "CUT:" in line_str:
                                    try:
                                        last_manta_confirmed_cutoff = float(re.search(r'CUT:\s*([\d\.]+)', line_str).group(1))
                                    except Exception:
                                        pass

                                if "DB:" in line_str:
                                    try:
                                        last_manta_confirmed_deadband = int(re.search(r'DB:\s*(\d+)', line_str).group(1))
                                    except Exception:
                                        pass

                                raw_adc = None
                                received_voltage = None

                                if "BAT_V:" in line_str:
                                    try:
                                        received_voltage = float(re.search(r'BAT_V:\s*([\d\.]+)', line_str).group(1))
                                    except Exception:
                                        pass

                                if "BAT_ADC:" in line_str:
                                    try:
                                        raw_adc = float(re.search(r'BAT_ADC:\s*([\d\.]+)', line_str).group(1))
                                    except Exception:
                                        pass

                                telemetry_updated = binary_telemetry_updated

                                # If binary telemetry was not present, fall back to ASCII regex parsing
                                if not binary_telemetry_updated:
                                    p_match = re.search(r'(?:Pitch|P):\s*([\d\.-]+)', line_str)
                                    if p_match:
                                        try:
                                            latest_pitch = float(p_match.group(1))
                                            telemetry_updated = True
                                        except Exception:
                                            pass
                                    r_match = re.search(r'(?:Roll|R):\s*([\d\.-]+)', line_str)
                                    if r_match:
                                        try:
                                            latest_roll = float(r_match.group(1))
                                            telemetry_updated = True
                                        except Exception:
                                            pass
                                    y_match = re.search(r'(?:Yaw|Y):\s*([\d\.-]+)', line_str)
                                    if y_match:
                                        try:
                                            latest_yaw = float(y_match.group(1))
                                            telemetry_updated = True
                                        except Exception:
                                            pass

                                    lat_match = re.search(r'LAT:\s*([\d\.-]+)', line_str)
                                    if lat_match:
                                        try:
                                            latest_lat = float(lat_match.group(1))
                                            telemetry_updated = True
                                        except Exception:
                                            pass
                                    lon_match = re.search(r'LON:\s*([\d\.-]+)', line_str)
                                    if lon_match:
                                        try:
                                            latest_lon = float(lon_match.group(1))
                                            telemetry_updated = True
                                        except Exception:
                                            pass
                                    alt_match = re.search(r'ALT:\s*([\d\.-]+)', line_str)
                                    if alt_match:
                                        try:
                                            latest_alt = float(alt_match.group(1))
                                            telemetry_updated = True
                                        except Exception:
                                            pass
                                    temp_match = re.search(r'TEMP:\s*([\d\.-]+)', line_str)
                                    if temp_match:
                                        try:
                                            latest_temp = float(temp_match.group(1))
                                            telemetry_updated = True
                                        except Exception:
                                            pass
                                    sat_match = re.search(r'SAT:\s*(\d+)', line_str)
                                    if sat_match:
                                        try:
                                            latest_satellites = int(sat_match.group(1))
                                            telemetry_updated = True
                                        except Exception:
                                            pass
                                    fix_match = re.search(r'FIX:\s*(\d+)', line_str)
                                    if fix_match:
                                        try:
                                            latest_fix_type = int(fix_match.group(1))
                                            telemetry_updated = True
                                        except Exception:
                                            pass

                                    rc_match = re.search(r'RC:\s*([\d]+),([\d]+),([\d]+),([\d]+),([\d]+)', line_str)
                                    if rc_match:
                                        try:
                                            latest_rc = [int(rc_match.group(i)) for i in range(1, 6)]
                                            telemetry_updated = True
                                        except Exception:
                                            pass

                                    sig_match = re.search(r'SIG:\s*(\d+)', line_str)
                                    if sig_match:
                                        try:
                                            rc_signal_lost = (int(sig_match.group(1)) == 0)
                                        except Exception:
                                            pass

                                has_gps_lock = (latest_satellites >= 4 and (latest_fix_type > 0 or (abs(latest_lat) > 0.0001 and abs(latest_lon) > 0.0001)))

                                # Auto Home Position Setup: GPS Active + Calibration Mode
                                is_gps_active = (latest_satellites >= 4 and (latest_fix_type > 0 or abs(latest_lat) > 0.0001))
                                is_calib_mode = is_debug_mode_active()

                                if is_gps_active and is_calib_mode:
                                    new_lat_int = int(latest_lat * 1e7)
                                    new_lon_int = int(latest_lon * 1e7)
                                    new_alt_mm = int(latest_alt * 1000)
                                    if new_lat_int != 0 and new_lon_int != 0:
                                        home_lat_int = new_lat_int
                                        home_lon_int = new_lon_int
                                        home_alt_mm = new_alt_mm
                                        if not home_position_set:
                                            home_position_set = True
                                            print(f"[HOME POSITION SET] Mission Planner HOME set: Lat={latest_lat:.6f}, Lon={latest_lon:.6f}, Alt={latest_alt:.1f}m")


                                if raw_adc is not None or received_voltage is not None or telemetry_updated:
                                    now = time.time()

                                    if start_time is None:
                                        start_time = now
                                        last_excel_log_time = now - 10.0
                                    elapsed_sec = round(now - start_time, 2)
                                    
                                    if raw_adc is None:
                                        raw_adc = 0.0

                                    if received_voltage is not None and received_voltage > 0:
                                        latest_estimated_voltage = round(received_voltage, 2)
                                    
                                    records.append(record_number)
                                    timestamps.append(elapsed_sec)
                                    raw_adcs.append(raw_adc)

                                    motor_is_on = (current_throttle_pulse > 1000)
                                    if latest_estimated_voltage <= alert_voltage_threshold:
                                        low_voltage_cutoff_active = True
                                        send_alarm_command(motor_is_on)
                                    else:
                                        low_voltage_cutoff_active = False
                                        send_alarm_command(False)

                                    # Continuous live telemetry print to terminal (every ~1s)
                                    if (now - last_terminal_print_time) >= 1.0:
                                        last_terminal_print_time = now
                                        cal_p, cal_r = get_calibrated_angles(latest_pitch, latest_roll)
                                        gps_info = f"Lock [{latest_lat:.6f}, {latest_lon:.6f}]" if has_gps_lock else f"No Lock ({latest_satellites} sats)"
                                        rc = latest_rc
                                        sig_str = "[!! RC SIGNAL LOST !!]" if rc_signal_lost else "[RC OK]"
                                        manta_cut_str = f" | Cutoff: {last_manta_confirmed_cutoff:.2f}V" if last_manta_confirmed_cutoff is not None else ""
                                        manta_db_str = f" | DB: {last_manta_confirmed_deadband}us" if last_manta_confirmed_deadband is not None else ""
                                        print(f"[MANTA RX {elapsed_sec:.1f}s] Batt: {latest_estimated_voltage:.2f}V | Alt: {latest_alt:.2f}m | Pitch: {cal_p:.1f}° | Roll: {cal_r:.1f}° | GPS: {gps_info}{manta_db_str} {sig_str}")
                                        print(f"  RC RAW => CH1(Roll):{rc[0]:4d}us  CH2(Pitch):{rc[1]:4d}us  CH3(Throttle):{rc[2]:4d}us  CH4:{rc[3]:4d}us  CH5:{rc[4]:4d}us")

                                        # Extra alert on signal loss
                                        if rc_signal_lost and (now - last_rc_signal_warn_time) >= 5.0:
                                            last_rc_signal_warn_time = now
                                            print("  ╔══════════════════════════════════════════╗")
                                            print("  ║      RC TRANSMITTER SIGNAL LOST          ║")
                                            print("  ║  Throttle cut, surfaces centered!       ║")
                                            print("  ╚══════════════════════════════════════════╝")


                                    if (now - last_excel_log_time) >= 10.0:
                                        last_excel_log_time = now
                                        cal_p, cal_r = get_calibrated_angles(latest_pitch, latest_roll)
                                        ws.append([record_number, elapsed_sec, raw_adc, latest_estimated_voltage, round(cal_p, 2), round(cal_r, 2), round(latest_yaw, 2), latest_lat, latest_lon, latest_alt, latest_satellites, latest_fix_type])
                                        try:
                                            wb.save(EXCEL_FILE)
                                            manta_cut_str = f" | Cutoff: {last_manta_confirmed_cutoff:.2f}V" if last_manta_confirmed_cutoff is not None else ""
                                            print(f"  └─> [EXCEL SAVED #{record_number}] {elapsed_sec:.2f}s | Saved to {EXCEL_FILE}")
                                        except Exception as e:
                                            print(f"[Excel Save Error] {e}")
                                        record_number += 1


            if mav_conn:
                now_mav = time.time()
                try:
                    while True:
                        msg = mav_conn.recv_match(blocking=False)
                        if msg is None:
                            break

                        m_type = msg.get_type()
                        if m_type == 'PARAM_REQUEST_LIST':
                            send_battery_parameters(mav_conn)
                        elif m_type == 'PARAM_REQUEST_READ':
                            p_id = getattr(msg, 'param_id', None)
                            p_idx = getattr(msg, 'param_index', -1)
                            send_battery_parameters(mav_conn, req_param_id=p_id, req_param_index=p_idx)
                        elif m_type == 'COMMAND_LONG':
                            mav_conn.mav.command_ack_send(msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED)
                            send_version_and_banner(mav_conn)
                        elif m_type == 'AUTOPILOT_VERSION_REQUEST':
                            send_version_and_banner(mav_conn)
                        elif m_type in ['MISSION_REQUEST_LIST', 'MISSION_REQUEST']:
                            try:
                                mav_conn.mav.mission_count_send(mav_conn.target_system, mav_conn.target_component, 0)
                            except Exception:
                                pass
                except (ConnectionResetError, socket.error, Exception):
                    pass


                if (now_mav - last_mav_heartbeat) >= 0.05: # 20 Hz stream
                    last_mav_heartbeat = now_mav
                    batt_mv = int(max(0.0, latest_estimated_voltage) * 1000)
                    
                    if latest_estimated_voltage > 13.0:
                        batt_pct = int(max(0, min(100, (latest_estimated_voltage - 14.0) / (16.8 - 14.0) * 100)))
                    else:
                        batt_pct = int(max(0, min(100, (latest_estimated_voltage - 10.5) / (12.6 - 10.5) * 100)))

                    cal_p, cal_r = get_calibrated_angles(latest_pitch, latest_roll)
                    try:
                        mav_state = mavutil.mavlink.MAV_STATE_EMERGENCY if rc_signal_lost else mavutil.mavlink.MAV_STATE_ACTIVE
                        mav_conn.mav.heartbeat_send(
                            mavutil.mavlink.MAV_TYPE_FIXED_WING,
                            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                            mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED | mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            0,
                            mav_state
                        )
                        
                        rc_sensor_ok = 0 if rc_signal_lost else mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER
                        sensors_mask = (
                            mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                            mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                            mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                            mavutil.mavlink.MAV_SYS_STATUS_SENSOR_BATTERY |
                            mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS |
                            rc_sensor_ok
                        )
                        mav_conn.mav.sys_status_send(
                            sensors_mask, sensors_mask, sensors_mask, 500, batt_mv, -1, batt_pct, 0, 0, 0, 0, 0, 0
                        )

                        mav_conn.mav.battery_status_send(
                            0,
                            mavutil.mavlink.MAV_BATTERY_FUNCTION_ALL,
                            mavutil.mavlink.MAV_BATTERY_TYPE_LIPO,
                            32767,
                            [batt_mv, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535],
                            -1, -1, -1,
                            batt_pct
                        )

                        lat_int = int(latest_lat * 1e7)
                        lon_int = int(latest_lon * 1e7)
                        alt_mm = int(latest_alt * 1000)
                        fix_t = latest_fix_type if latest_fix_type > 0 else (3 if abs(latest_lat) > 0.001 else 0)

                        mav_conn.mav.gps_raw_int_send(
                            int(now_mav * 1e6) & 0xFFFFFFFFFFFFFFFF,
                            fix_t, lat_int, lon_int, alt_mm,
                            100, 100, 0, 0,
                            max(0, latest_satellites)
                        )

                        mav_conn.mav.global_position_int_send(
                            int(now_mav * 1000) & 0xFFFFFFFF,
                            lat_int, lon_int, alt_mm, alt_mm,
                            0, 0, 0,
                            int(latest_yaw % 360 * 100)
                        )

                        # Mission Planner HOME Position (Message #242)
                        if home_lat_int != 0 and home_lon_int != 0:
                            mav_conn.mav.home_position_send(
                                home_lat_int,
                                home_lon_int,
                                home_alt_mm,
                                0.0, 0.0, 0.0,
                                [1.0, 0.0, 0.0, 0.0],
                                0.0, 0.0, 0.0
                            )

                        mav_conn.mav.attitude_send(
                            int(now_mav * 1000) & 0xFFFFFFFF,
                            math.radians(cal_r),
                            -math.radians(cal_p),
                            math.radians(latest_yaw),
                            0.0, 0.0, 0.0
                        )


                        mav_conn.mav.vfr_hud_send(
                            0.0, 0.0, int(latest_yaw % 360), 50, 100.0, 0.0
                        )

                        # 8. Scaled Pressure (Message #29) for Baro Temperature (press_temp in Mission Planner)
                        press_hpa = 1013.25 * math.pow(max(0.0001, 1.0 - (latest_alt / 44330.0)), 5.255)
                        temp_val = int(round(latest_temp)) # Rounded to integer units for press_temp
                        mav_conn.mav.scaled_pressure_send(
                            int(now_mav * 1000) & 0xFFFFFFFF,
                            press_hpa,
                            0.0,
                            temp_val,
                            temp_val
                        )

                        # 9. RC Channels (Message #35 & Message #65) for Mission Planner Radio Calibration / RC Input HUD
                        mav_conn.mav.rc_channels_raw_send(
                            int(now_mav * 1000) & 0xFFFFFFFF,
                            0,
                            latest_rc[0], latest_rc[1], latest_rc[2], latest_rc[3],
                            latest_rc[4], 0, 0, 0,
                            255
                        )
                        mav_conn.mav.rc_channels_send(
                            int(now_mav * 1000) & 0xFFFFFFFF,
                            5,
                            latest_rc[0], latest_rc[1], latest_rc[2], latest_rc[3],
                            latest_rc[4], 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 255
                        )
                        # Calculate CH5 noise error in Python (diff to 1000us or 2000us)
                        ch5_val = latest_rc[4]
                        ch5_err = abs(ch5_val - (1000 if ch5_val < 1500 else 2000)) if ch5_val > 0 else 0

                        # Servo Output Raw (Message #36) -> maps ch5_err directly to ch5out in Mission Planner Status
                        mav_conn.mav.servo_output_raw_send(
                            int(now_mav * 1e6) & 0xFFFFFFFFFFFFFFFF,
                            0,
                            latest_rc[0], latest_rc[1], latest_rc[2], latest_rc[3],
                            ch5_err, 0, 0, 0
                        )






                    except Exception:
                        pass

            time.sleep(0.005)

        except KeyboardInterrupt:
            print("\n[Mission Planner Bridge] Terminated by user.")
            break
        except Exception:
            time.sleep(0.01)

    send_alarm_command(False, force=True)
    if ser and ser.is_open:
        try:
            ser.close()
        except Exception:
            pass
    if 'wb' in locals():
        wb.save(EXCEL_FILE)
    
    print("MANTA Mission Planner Bridge terminated successfully.")

if __name__ == "__main__":
    import threading
    import tkinter as tk
    from lora_logger import BatteryAnalyzerGUI

    print("[MANTA Ground Station] Starting Mission Planner MAVLink Bridge in background thread...")
    mav_thread = threading.Thread(target=main, daemon=True)
    mav_thread.start()

    # Wait briefly for the serial connection to be established in main()
    import time
    for _ in range(30):  # Wait up to 3 seconds
        if active_serial_conn is not None and active_serial_conn.is_open:
            break
        time.sleep(0.1)

    print("[MANTA Ground Station] Launching Graphical Ground Station GUI...")
    root = tk.Tk()
    # Pass the shared serial connection so the GUI doesn't try to open COM4 again
    app = BatteryAnalyzerGUI(root, shared_serial=active_serial_conn)
    gui_app = app
    root.mainloop()
