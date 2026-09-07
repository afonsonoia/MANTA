import sys
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import os
import json

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT_DIR = os.path.abspath(os.path.join(_SCRIPT_DIR, '..', '..'))
if os.path.exists(os.path.join(_ROOT_DIR, 'imu_calibration.json')):
    CALIB_FILE = os.path.join(_ROOT_DIR, 'imu_calibration.json')
elif os.path.exists('imu_calibration.json'):
    CALIB_FILE = os.path.abspath('imu_calibration.json')
else:
    CALIB_FILE = os.path.join(_ROOT_DIR, 'imu_calibration.json')


class SimpleGroundStationGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("GROUND-STATION - Buzzer Controller")
        self.root.geometry("540x720")
        self.root.resizable(True, True)

        # Serial State
        self.serial_conn = None
        self.is_connected = False
        self.read_thread = None
        self.current_mode = "OFF"

        # Theme Colors (Dark Theme)
        self.BG_COLOR = "#1e1e2e"
        self.CARD_BG = "#2a2a3c"
        self.TEXT_COLOR = "#cdd6f4"
        self.SUBTEXT_COLOR = "#a6adc8"
        self.ACCENT_GREEN = "#a6e3a1"
        self.ACCENT_RED = "#f38ba8"
        self.ACCENT_BLUE = "#89b4fa"
        self.ACCENT_YELLOW = "#f9e2af"
        self.ACCENT_CYAN = "#94e2d5"
        self.LOG_BG = "#11111b"

        self.root.configure(bg=self.BG_COLOR)

        self._configure_styles()
        self._build_ui()
        self.refresh_ports()

    def _configure_styles(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure(".", background=self.BG_COLOR, foreground=self.TEXT_COLOR, font=("Segoe UI", 10))
        style.configure("TCombobox", fieldbackground=self.CARD_BG, background=self.CARD_BG, foreground=self.TEXT_COLOR)
        style.map("TCombobox", fieldbackground=[("readonly", self.CARD_BG)], foreground=[("readonly", self.TEXT_COLOR)])

    def _build_ui(self):
        # --- HEADER ---
        header_frame = tk.Frame(self.root, bg=self.BG_COLOR, pady=10)
        header_frame.pack(fill=tk.X, padx=20)

        title_lbl = tk.Label(
            header_frame, text="GROUND-STATION",
            font=("Segoe UI", 18, "bold"), bg=self.BG_COLOR, fg=self.ACCENT_BLUE
        )
        title_lbl.pack(anchor="w")

        subtitle_lbl = tk.Label(
            header_frame, text="LoRa Simplex Downlink Receiver (433 MHz) & Local Buzzer Controller (Pin D22)",
            font=("Segoe UI", 9, "italic"), bg=self.BG_COLOR, fg=self.SUBTEXT_COLOR
        )
        subtitle_lbl.pack(anchor="w")

        # --- SERIAL CONNECTION CARD ---
        conn_card = tk.Frame(self.root, bg=self.CARD_BG, bd=0, relief="flat", padx=15, pady=10)
        conn_card.pack(fill=tk.X, padx=20, pady=4)

        conn_title = tk.Label(conn_card, text="Serial USB Connection", font=("Segoe UI", 10, "bold"), bg=self.CARD_BG, fg=self.TEXT_COLOR)
        conn_title.grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 6))

        tk.Label(conn_card, text="COM Port:", bg=self.CARD_BG, fg=self.TEXT_COLOR).grid(row=1, column=0, sticky="w")

        self.port_combo = ttk.Combobox(conn_card, state="readonly", width=20)
        self.port_combo.grid(row=1, column=1, padx=8, sticky="w")

        self.btn_refresh = tk.Button(
            conn_card, text="REFRESH", command=self.refresh_ports,
            bg="#313244", fg=self.TEXT_COLOR, activebackground="#45475a", bd=0, padx=8, pady=2, cursor="hand2"
        )
        self.btn_refresh.grid(row=1, column=2, sticky="w")

        self.btn_connect = tk.Button(
            conn_card, text="Connect", command=self.toggle_connection,
            font=("Segoe UI", 9, "bold"), bg=self.ACCENT_BLUE, fg="#11111b",
            activebackground="#74c7ec", bd=0, pady=5, cursor="hand2"
        )
        self.btn_connect.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(8, 0))

        self.lbl_status = tk.Label(conn_card, text="Disconnected", font=("Segoe UI", 9, "bold"), bg=self.CARD_BG, fg=self.ACCENT_RED)
        self.lbl_status.grid(row=3, column=0, columnspan=3, pady=(4, 0))

        # --- GROUND STATION LOCAL SETTINGS CARD (SIMPLEX DOWNLINK) ---
        settings_card = tk.Frame(self.root, bg=self.CARD_BG, bd=0, relief="flat", padx=15, pady=10)
        settings_card.pack(fill=tk.X, padx=20, pady=4)

        tk.Label(settings_card, text="Ground Station Local Alert Settings", font=("Segoe UI", 10, "bold"), bg=self.CARD_BG, fg=self.ACCENT_CYAN).grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 6))

        initial_db = 18
        initial_cutoff = 12.50
        if os.path.exists(CALIB_FILE):
            try:
                with open(CALIB_FILE, 'r', encoding='utf-8') as f:
                    c_data = json.load(f)
                    if 'deadband' in c_data:
                        initial_db = max(1, min(50, int(c_data['deadband'])))
                    if 'cutoff' in c_data:
                        initial_cutoff = float(c_data['cutoff'])
            except Exception:
                pass

        tk.Label(settings_card, text="Deadband Margin (us):", bg=self.CARD_BG, fg=self.TEXT_COLOR).grid(row=1, column=0, sticky="w", pady=4)
        self.spin_deadband = tk.Spinbox(settings_card, from_=1, to=50, width=6, bg=self.LOG_BG, fg=self.TEXT_COLOR, insertbackground=self.TEXT_COLOR)
        self.spin_deadband.delete(0, tk.END); self.spin_deadband.insert(0, str(initial_db))
        self.spin_deadband.grid(row=1, column=1, sticky="w", padx=4, pady=4)

        tk.Label(settings_card, text="Battery Alert Cutoff (V):", bg=self.CARD_BG, fg=self.TEXT_COLOR).grid(row=1, column=2, sticky="w", padx=(10, 4), pady=4)
        self.spin_cutoff = tk.Spinbox(settings_card, from_=10.0, to=16.8, increment=0.1, width=6, bg=self.LOG_BG, fg=self.TEXT_COLOR, insertbackground=self.TEXT_COLOR)
        self.spin_cutoff.delete(0, tk.END); self.spin_cutoff.insert(0, f"{initial_cutoff:.2f}")
        self.spin_cutoff.grid(row=1, column=3, sticky="w", padx=4, pady=4)

        self.btn_save_settings = tk.Button(
            settings_card, text="SAVE LOCAL SETTINGS", command=self.save_local_settings,
            font=("Segoe UI", 9, "bold"), bg=self.ACCENT_GREEN, fg="#11111b",
            activebackground="#a6e3a1", bd=0, pady=4, cursor="hand2"
        )
        self.btn_save_settings.grid(row=2, column=0, columnspan=4, sticky="ew", pady=(6, 2))

        info_lbl = tk.Label(
            settings_card,
            text="Simplex Telemetry (Aircraft TX -> GS RX). Aircraft filters & trims are set in MANTA firmware.",
            font=("Segoe UI", 7, "italic"), bg=self.CARD_BG, fg=self.SUBTEXT_COLOR
        )
        info_lbl.grid(row=3, column=0, columnspan=4, sticky="w", pady=(2, 0))

        # --- BUZZER CONTROL ACTIONS CARD ---
        ctrl_card = tk.Frame(self.root, bg=self.CARD_BG, bd=0, relief="flat", padx=15, pady=10)
        ctrl_card.pack(fill=tk.X, padx=20, pady=4)

        tk.Label(ctrl_card, text="Ground Station Buzzer Controls (Pin D22)", font=("Segoe UI", 10, "bold"), bg=self.CARD_BG, fg=self.TEXT_COLOR).pack(anchor="w", pady=(0, 6))

        actions_frame = tk.Frame(ctrl_card, bg=self.CARD_BG)
        actions_frame.pack(fill=tk.X)

        self.btn_short = tk.Button(
            actions_frame, text="Short Beep (0.7s)", command=self.set_short,
            font=("Segoe UI", 9, "bold"), bg="#45475a", fg=self.TEXT_COLOR, bd=0, pady=8, state="disabled", cursor="hand2"
        )
        self.btn_short.grid(row=0, column=0, padx=4, pady=2, sticky="nsew")

        self.btn_continuous = tk.Button(
            actions_frame, text="Continuous Beep", command=self.set_continuous,
            font=("Segoe UI", 9, "bold"), bg="#45475a", fg=self.TEXT_COLOR, bd=0, pady=8, state="disabled", cursor="hand2"
        )
        self.btn_continuous.grid(row=0, column=1, padx=4, pady=2, sticky="nsew")

        self.btn_intermittent = tk.Button(
            actions_frame, text="Intermittent Beep", command=self.set_intermittent,
            font=("Segoe UI", 9, "bold"), bg="#45475a", fg=self.TEXT_COLOR, bd=0, pady=8, state="disabled", cursor="hand2"
        )
        self.btn_intermittent.grid(row=1, column=0, padx=4, pady=2, sticky="nsew")

        self.btn_off = tk.Button(
            actions_frame, text="Turn OFF", command=self.set_off,
            font=("Segoe UI", 9, "bold"), bg="#45475a", fg=self.TEXT_COLOR, bd=0, pady=8, state="disabled", cursor="hand2"
        )
        self.btn_off.grid(row=1, column=1, padx=4, pady=2, sticky="nsew")

        actions_frame.columnconfigure(0, weight=1)
        actions_frame.columnconfigure(1, weight=1)

        # --- CONSOLE LOG ---
        log_frame = tk.Frame(self.root, bg=self.BG_COLOR, padx=20, pady=4)
        log_frame.pack(fill=tk.BOTH, expand=True)

        tk.Label(log_frame, text="Serial Console:", bg=self.BG_COLOR, fg=self.SUBTEXT_COLOR, font=("Segoe UI", 8)).pack(anchor="w")

        self.txt_log = tk.Text(
            log_frame, height=5, bg=self.LOG_BG, fg=self.TEXT_COLOR,
            font=("Consolas", 9), bd=0, padx=6, pady=6
        )
        self.txt_log.pack(fill=tk.BOTH, expand=True, pady=(2, 8))

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def log(self, msg):
        self.txt_log.insert(tk.END, f"{msg}\n")
        self.txt_log.see(tk.END)

    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        port_list = [f"{p.device} - {p.description}" for p in ports]
        self.port_combo['values'] = port_list

        if port_list:
            self.port_combo.current(0)
            self.log(f"[SYSTEM] {len(port_list)} COM port(s) detected.")
        else:
            self.port_combo.set('')
            self.log("[SYSTEM] No COM port detected. Please connect your ESP32.")

    def toggle_connection(self):
        if not self.is_connected:
            selection = self.port_combo.get()
            if not selection:
                messagebox.showwarning("Warning", "Please select a COM port before connecting.")
                return

            port_name = selection.split(" ")[0]

            try:
                self.serial_conn = serial.Serial(port_name, 115200, timeout=1)
                self.is_connected = True
                self.btn_connect.config(text="Disconnect", bg=self.ACCENT_RED)
                self.lbl_status.config(text=f"Connected ({port_name})", fg=self.ACCENT_GREEN)
                self._set_buttons_state("normal")
                self.log(f"[CONNECTION] Successfully connected to {port_name}.")

                self.read_thread = threading.Thread(target=self._read_serial_loop, daemon=True)
                self.read_thread.start()

                # Set default state OFF
                self.set_off()

            except Exception as e:
                messagebox.showerror("Connection Error", f"Could not open port {port_name}:\n{e}")
                self.log(f"[ERROR] Failed to connect: {e}")
        else:
            self._disconnect()

    def _disconnect(self):
        self.set_off()
        self.is_connected = False
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
            except Exception:
                pass

        self.btn_connect.config(text="Connect", bg=self.ACCENT_BLUE)
        self.lbl_status.config(text="Disconnected", fg=self.ACCENT_RED)
        self._set_buttons_state("disabled")
        self.log("[CONNECTION] Disconnected.")

    def _set_buttons_state(self, state):
        for btn in [self.btn_short, self.btn_continuous, self.btn_intermittent, self.btn_off]:
            btn.config(state=state)

    def save_local_settings(self):
        try:
            db_val = int(self.spin_deadband.get().strip())
            if db_val < 1: db_val = 1
            if db_val > 50: db_val = 50
            cutoff_val = float(self.spin_cutoff.get().strip())
            if cutoff_val < 10.0: cutoff_val = 10.0
            if cutoff_val > 16.8: cutoff_val = 16.8

            c_dict = {}
            if os.path.exists(CALIB_FILE):
                try:
                    with open(CALIB_FILE, 'r', encoding='utf-8') as f:
                        c_dict = json.load(f)
                except Exception:
                    c_dict = {}

            c_dict['deadband'] = db_val
            c_dict['cutoff'] = round(cutoff_val, 2)
            with open(CALIB_FILE, 'w', encoding='utf-8') as f:
                json.dump(c_dict, f, indent=2)

            self.log(f"[CONFIG] Local settings saved: Deadband={db_val} us, Cutoff={cutoff_val:.2f} V")
            messagebox.showinfo(
                "Local Settings Saved",
                f"Ground Station settings saved to imu_calibration.json:\n"
                f"• Deadband Margin: {db_val} us\n"
                f"• Battery Cutoff Alarm: {cutoff_val:.2f} V\n\n"
                f"(Simplex Telemetry: Aircraft filters and mixing are set in MANTA firmware config.h)"
            )
        except Exception as e:
            messagebox.showerror("Settings Error", f"Invalid settings value: {e}")

    def send_cmd(self, cmd_str):
        if self.is_connected and self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(f"{cmd_str}\n".encode('utf-8'))
                self.log(f"[TX] -> {cmd_str}")
            except Exception as e:
                self.log(f"[ERROR] Failed to send: {e}")

    def set_short(self):
        self.current_mode = "SHORT"
        self.send_cmd("BEEP:SHORT")
        self._update_active_button(self.btn_short, self.ACCENT_CYAN)

    def set_continuous(self):
        self.current_mode = "CONTINUOUS"
        self.send_cmd("BEEP:CONTINUOUS")
        self._update_active_button(self.btn_continuous, self.ACCENT_GREEN)

    def set_intermittent(self):
        self.current_mode = "INTERMITTENT"
        self.send_cmd("BEEP:INTERMITTENT")
        self._update_active_button(self.btn_intermittent, self.ACCENT_YELLOW)

    def set_off(self):
        self.current_mode = "OFF"
        self.send_cmd("BEEP:OFF")
        self._update_active_button(self.btn_off)

    def _update_active_button(self, active_btn, active_color="#585b70"):
        for btn in [self.btn_short, self.btn_continuous, self.btn_intermittent, self.btn_off]:
            if btn == active_btn:
                btn.config(bg=active_color, fg="#11111b" if active_color != "#585b70" else self.TEXT_COLOR)
            else:
                btn.config(bg="#45475a", fg=self.TEXT_COLOR)

    def _read_serial_loop(self):
        while self.is_connected and self.serial_conn and self.serial_conn.is_open:
            try:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        if "CALIB_DATA:" in line:
                            try:
                                import re
                                data_part = line[line.find("CALIB_DATA:") + 11:].strip()
                                db_m = re.search(r'DB=(\d+)', data_part)
                                if db_m:
                                    db_val = db_m.group(1)
                                    self.root.after(0, lambda v=db_val: (self.spin_deadband.delete(0, tk.END), self.spin_deadband.insert(0, v)))
                            except Exception:
                                pass
                        self.root.after(0, self.log, f"[RX] <- {line}")
            except Exception:
                break
            time.sleep(0.03)

    def on_close(self):
        self._disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = SimpleGroundStationGUI(root)
    root.mainloop()
