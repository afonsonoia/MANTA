import serial
import serial.tools.list_ports
import time
import openpyxl
from openpyxl import Workbook
import os
import math
import re
import contextlib
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.backends._backend_tk as _backend_tk

@contextlib.contextmanager
def _dummy_restore():
    yield

_backend_tk._restore_foreground_window_at_end = _dummy_restore

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, TextBox

# Config
DEFAULT_BAUD = 115200
EXCEL_FILE = 'registo_bateria_lora.xlsx'
MAX_PLOT_POINTS = 100
BATTERY_DIVIDER_RATIO = 4.84  # Voltage divider factor (4.84:1 ratio)

def calculate_battery_voltage(raw_adc):
    """Calculates battery voltage matching the ESP32 polynomial equation:
       voltage = -0.000000884 * (raw_adc^2) + 0.008835 * raw_adc - 5.6904
    """
    voltage = -0.000000884 * (raw_adc ** 2) + 0.008835 * raw_adc - 5.6904
    return max(0.0, voltage)

# Global state
current_throttle_pulse = 1000  # Default off/armed pulse (1000us)
last_sent_pulse = None
active_serial_conn = None
low_voltage_cutoff_active = False
alarm_active = None  # None force-sends initial state on connect
last_rssi = None
last_snr = None

# Configurable Low Voltage Alert Threshold (Default: 12.50V)
alert_voltage_threshold = 12.50

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
    """Sends a throttle PWM command (1000us - 2000us) over LoRa Serial ONLY if changed or forced."""
    global current_throttle_pulse, last_sent_pulse, active_serial_conn, low_voltage_cutoff_active
    
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

def update_live_plot(fig, ax, line, text_info, timestamps, raw_adcs, throttle_slider=None):
    """Updates the live matplotlib graph and evaluates low voltage alarm threshold & signal strength."""
    global low_voltage_cutoff_active, alert_voltage_threshold, last_rssi, last_snr
    N = len(timestamps)
    if N == 0:
        return
    
    if N <= MAX_PLOT_POINTS:
        plot_x = list(timestamps)
        plot_y = list(raw_adcs)
    else:
        step = math.ceil(N / MAX_PLOT_POINTS)
        plot_x = list(timestamps[::step])
        plot_y = list(raw_adcs[::step])
        if plot_x[-1] != timestamps[-1]:
            plot_x.append(timestamps[-1])
            plot_y.append(raw_adcs[-1])

    line.set_data(plot_x, plot_y)
    
    ax.relim()
    ax.autoscale_view()
    
    latest_adc = raw_adcs[-1]
    latest_t = timestamps[-1]
    pin_v = (latest_adc * 3.3) / 4095.0
    est_v = calculate_battery_voltage(latest_adc)

    # Check strictly against Alert Voltage Threshold
    if est_v <= alert_voltage_threshold and latest_adc > 0.0:
        low_voltage_cutoff_active = True
        send_alarm_command(True)  # Trigger 50% 2kHz intermittent beep ONLY when below threshold
        if throttle_slider and throttle_slider.val > 1000:
            throttle_slider.set_val(1000)
    else:
        low_voltage_cutoff_active = False
        send_alarm_command(False)  # Turn off alarm

    # Determine LoRa Signal Quality Status
    if last_rssi is not None:
        if last_rssi >= -75:
            sig_status = "🟢 STRONG (Excellent Signal)"
        elif last_rssi >= -90:
            sig_status = "🟡 GOOD (Stable Signal)"
        elif last_rssi >= -105:
            sig_status = "🟠 WEAK (Check Range)"
        else:
            sig_status = "🔴 CRITICAL (Low Signal)"
        snr_str = f" | SNR: {last_snr:.1f} dB" if last_snr is not None else ""
        signal_info = f"LoRa Signal: RSSI {last_rssi} dBm{snr_str}  [{sig_status}]"
    else:
        signal_info = "LoRa Signal: Waiting for first packet..."

    if low_voltage_cutoff_active:
        text_info.set_text(
            f"ALERT: BATTERY <= {alert_voltage_threshold:.2f}V! INTERMITTENT BEEP (50% 2kHz) ACTIVE!\n"
            f"Elapsed: {latest_t:.1f}s  |  Raw ADC: {latest_adc:.1f}  |  Pin: {pin_v:.2f}V  |  Est Batt: {est_v:.2f}V\n"
            f"{signal_info}"
        )
        text_info.set_bbox(dict(boxstyle='round,pad=0.5', facecolor='#660000', alpha=0.9, edgecolor='#FF0000'))
    else:
        throttle_str = f"OFF (1000us)" if current_throttle_pulse == 1000 else f"ON ({current_throttle_pulse}us)"
        text_info.set_text(
            f"Elapsed: {latest_t:.1f}s  |  Raw ADC: {latest_adc:.1f}  |  Pin: {pin_v:.2f}V  |  Est Batt: {est_v:.2f}V  |  Threshold: {alert_voltage_threshold:.2f}V  |  Throttle: {throttle_str}\n"
            f"{signal_info}"
        )
        text_info.set_bbox(dict(boxstyle='round,pad=0.5', facecolor='#1E1E1E', alpha=0.85, edgecolor='#00E5FF'))

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

    print("==================================================")
    print("  LoRa Wireless Battery Monitor & Ground Alarm   ")
    print("==================================================")
    print(f"Audio alarm configured: <= {alert_voltage_threshold:.2f}V (Intermittent 50% 2kHz)")
    print(f"Excel Log File       : {EXCEL_FILE}")
    print("==================================================\n")
    
    port_name = auto_find_com_port()
    if not port_name:
        print("[Error] No COM port found! Connect Ground Station ESP32.")
        return

    print(f"Connecting to Ground Station on port {port_name} @ {DEFAULT_BAUD} baud...")
    
    try:
        ser = serial.Serial(port_name, DEFAULT_BAUD, timeout=0.05)
        active_serial_conn = ser
        print(f"Successfully connected to port {port_name}!\n")
        
        # Ensure alarm is initially OFF on connect
        send_alarm_command(False, force=True)
    except Exception as e:
        print(f"[Error] Could not open {port_name}: {e}")
        return

    records = []
    timestamps = []
    raw_adcs = []
    start_time = None

    # Delete previous Excel log file if it exists, creating a fresh log every run
    if os.path.exists(EXCEL_FILE):
        try:
            os.remove(EXCEL_FILE)
            print(f"Previous log file '{EXCEL_FILE}' deleted successfully.")
        except Exception as e:
            print(f"[Warning] Could not delete previous log file '{EXCEL_FILE}': {e}")

    wb = Workbook()
    ws = wb.active
    ws.title = "LoRa Battery Log"
    ws.append(["Record Number", "Elapsed Time (s)", "Raw Sensor (ADC 0-4095)", "Raw Pin Voltage (V)", "Estimated Voltage (V)"])
    record_number = 1
    wb.save(EXCEL_FILE)
    print(f"Created new clean log file '{EXCEL_FILE}'.")

    plt.style.use('dark_background')
    plt.ion()
    fig, ax = plt.subplots(figsize=(11.5, 6.5))
    fig.canvas.manager.set_window_title("LoRa Wireless Battery Monitor & Alarm Controller")

    fig.subplots_adjust(left=0.08, right=0.80, top=0.90, bottom=0.18)

    line, = ax.plot([], [], color='#00E5FF', linewidth=2, marker='o', markersize=3.5, label='Raw Sensor Value (ADC)')
    ax.set_title("LoRa Live Battery Telemetry & Alarm Controller", fontsize=14, pad=12, fontweight='bold', color='#FFFFFF')
    ax.set_xlabel("Elapsed Time (s)", fontsize=11)
    ax.set_ylabel("Raw Sensor ADC Reading (0 - 4095)", fontsize=11)
    ax.grid(True, linestyle='--', alpha=0.35)
    ax.legend(loc='upper right')

    text_info = ax.text(0.02, 0.95, "Waiting for data via LoRa radio...", transform=ax.transAxes, fontsize=9.5,
                        verticalalignment='top', bbox=dict(boxstyle='round,pad=0.5', facecolor='#1E1E1E', alpha=0.85, edgecolor='#00E5FF'))

    # Add Vertical Slider for Throttle Control (1000us to 1500us, step 50)
    ax_slider = fig.add_axes([0.85, 0.20, 0.03, 0.65])
    throttle_slider = Slider(
        ax=ax_slider,
        label='Throttle\n(us)',
        valmin=1000,
        valmax=1500,
        valinit=1000,
        valstep=50,
        orientation='vertical',
        color='#00E5FF',
        valfmt='%d'
    )
    throttle_slider.label.set_color('#FFFFFF')
    throttle_slider.label.set_fontsize(9)
    throttle_slider.label.set_fontweight('bold')
    throttle_slider.valtext.set_color('#00E5FF')

    # Add Configurable Alert Voltage TextBox
    ax_txt = fig.add_axes([0.65, 0.03, 0.12, 0.05])
    txt_alert = TextBox(ax_txt, 'Alert Threshold (V): ', initial=str(alert_voltage_threshold))
    txt_alert.label.set_color('#FFFFFF')
    txt_alert.label.set_fontsize(9.5)
    txt_alert.label.set_fontweight('bold')
    txt_alert.text_disp.set_color('#00E5FF')

    def on_submit_alert(text):
        global alert_voltage_threshold
        try:
            val = float(text)
            if val > 0:
                alert_voltage_threshold = val
                print(f"[Config] Voltage Alert Threshold updated to: {alert_voltage_threshold:.2f} V")
                if timestamps:
                    update_live_plot(fig, ax, line, text_info, timestamps, raw_adcs, throttle_slider=throttle_slider)
        except ValueError:
            pass

    txt_alert.on_submit(on_submit_alert)

    # Add Emergency Stop Button
    ax_btn_stop = fig.add_axes([0.22, 0.03, 0.28, 0.05])
    btn_stop = Button(ax_btn_stop, 'EMERGENCY STOP (1000us)', color='#551111', hovercolor='#991111')
    btn_stop.label.set_color('#FFDDDD')
    btn_stop.label.set_fontsize(9.5)
    btn_stop.label.set_fontweight('bold')

    def on_slider_change(val):
        if low_voltage_cutoff_active:
            if throttle_slider.val > 1000:
                throttle_slider.set_val(1000)
                return
        pulse = int(val)
        send_throttle_command(pulse)
        if timestamps:
            update_live_plot(fig, ax, line, text_info, timestamps, raw_adcs, throttle_slider=throttle_slider)

    def on_click_stop(event):
        throttle_slider.set_val(1000)

    throttle_slider.on_changed(on_slider_change)
    btn_stop.on_clicked(on_click_stop)

    if timestamps:
        update_live_plot(fig, ax, line, text_info, timestamps, raw_adcs, throttle_slider=throttle_slider)

    send_throttle_command(current_throttle_pulse, force=True)
    buffer = ""

    while plt.fignum_exists(fig.number):
        try:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                if data:
                    buffer += data.decode('utf-8', errors='ignore')
                    while '\n' in buffer:
                        line_str, buffer = buffer.split('\n', 1)
                        line_str = line_str.strip()
                        if line_str:
                            # Always print all raw serial messages regardless of content
                            print(f"[RAW SERIAL] {line_str}")

                            # Extract LoRa RSSI and SNR if present in serial stream
                            if "RSSI:" in line_str:
                                try:
                                    rssi_match = re.search(r'RSSI:\s*(-?\d+)', line_str)
                                    if rssi_match:
                                        last_rssi = int(rssi_match.group(1))
                                except Exception:
                                    pass

                            if "SNR:" in line_str:
                                try:
                                    snr_match = re.search(r'SNR:\s*([\d\.-]+)', line_str)
                                    if snr_match:
                                        last_snr = float(snr_match.group(1))
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
                            elif line_str.replace('.', '', 1).isdigit():
                                raw_adc = float(line_str)

                            if raw_adc is not None or received_voltage is not None:
                                now = time.time()
                                if start_time is None:
                                    start_time = now
                                elapsed_sec = round(now - start_time, 2)
                                
                                if raw_adc is None:
                                    raw_adc = 0.0
                                pin_voltage = round((raw_adc * 3.3) / 4095.0, 3)

                                # Use pre-calculated voltage from MANTA if sent, avoiding duplicate conversion
                                if received_voltage is not None:
                                    estimated_voltage = round(received_voltage, 2)
                                else:
                                    estimated_voltage = round(calculate_battery_voltage(raw_adc), 2)
                                
                                ws.append([record_number, elapsed_sec, raw_adc, pin_voltage, estimated_voltage])
                                wb.save(EXCEL_FILE)
                                
                                records.append(record_number)
                                timestamps.append(elapsed_sec)
                                raw_adcs.append(raw_adc)
                                
                                print(f"  └─> [LOGGED TO EXCEL #{record_number}] {elapsed_sec:.2f}s | Raw ADC: {raw_adc:.1f} | Pin: {pin_voltage:.2f}V | Est Batt (From MANTA): {estimated_voltage:.2f}V")
                                record_number += 1
                                
                                try:
                                    update_live_plot(fig, ax, line, text_info, timestamps, raw_adcs, throttle_slider=throttle_slider)
                                except Exception:
                                    pass

            if plt.fignum_exists(fig.number):
                try:
                    fig.canvas.flush_events()
                    plt.pause(0.03)
                except Exception:
                    pass

        except KeyboardInterrupt:
            print("\nLogging cancelled by user.")
            break
        except Exception as e:
            print(f"[Error] {e}")
            plt.pause(0.1)

    send_alarm_command(False, force=True)
    if ser and ser.is_open:
        ser.close()
    if 'wb' in locals():
        wb.save(EXCEL_FILE)
    
    print("LoRa logger terminated successfully.")

if __name__ == "__main__":
    main()
