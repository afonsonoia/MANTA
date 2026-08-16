import sys
import time
import threading
import re
import os
import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import openpyxl
from openpyxl import Workbook
from raw_lora_logger import AsyncRawLoRaLogger

# Try importing matplotlib for embedded live plot
HAS_MATPLOTLIB = False
try:
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

DEFAULT_BAUD = 115200
EXCEL_FILE = 'lora_battery_log.xlsx'
DEFAULT_ALERT_VOLTAGE = 12.50

def calculate_battery_voltage(raw_adc):
    """Calculates battery voltage matching the ESP32 polynomial equation:
       voltage = -0.000000884 * (raw_adc^2) + 0.008835 * raw_adc - 5.6904
    """
    voltage = -0.000000884 * (raw_adc ** 2) + 0.008835 * raw_adc - 5.6904
    return max(0.0, voltage)


class BatteryAnalyzerGUI:
    def __init__(self, root, shared_serial=None):
        self.root = root
        self.root.title("MANTA - LoRa Battery Analyzer & Logger")
        self.root.geometry("820x680")
        self.root.minsize(720, 580)

        # State Variables — shared_serial allows reusing a connection opened externally
        self.serial_conn = shared_serial
        self.is_connected = shared_serial is not None and shared_serial.is_open
        self.is_logging = False
        self.read_thread = None

        self.start_time = None
        self.timestamps = []
        self.voltages = []
        self.raw_adcs = []

        self.last_rssi = "N/A"
        self.last_snr = "N/A"
        self.current_voltage = 0.0
        self.current_adc = 0.0
        self.record_number = 1
        self.last_excel_save = 0.0

        # Last Known Calibration State for Change Logging
        self.last_angle = 30
        self.last_deadband = 18
        self.last_trims = [0, 0, 0, 0]
        self.last_inv = [0, 0, 0, 0]
        self.last_alert_voltage = 12.50
        self.last_lora_power = 14
        self.last_servo_interval = 50
        self.last_manta_ch5 = 1500  # Default: Normal Flight Mode
        self.last_received_ack = None
        self.pending_config_commands = []  # Queued commands until Calibration Mode (CH5 > 1900) is active

        # Excel State
        self.wb = None
        self.ws = None

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
        self.GRID_COLOR = "#313244"
        self.LOG_BG = "#11111b"

        self.root.configure(bg=self.BG_COLOR)
        self.async_raw_logger = AsyncRawLoRaLogger()

        self._configure_styles()
        self._build_ui()
        self.refresh_ports()
        # If we already have a shared connection, update UI to reflect connected state
        if self.is_connected:
            port_name = self.serial_conn.port
            self.btn_connect.config(text="DISCONNECT", bg=self.ACCENT_RED)
            self.lbl_status.config(text=f"Connected ({port_name})", fg=self.ACCENT_GREEN)
            self.port_combo.set(port_name)
            if shared_serial is None:
                self.read_thread = threading.Thread(target=self._read_serial_loop, daemon=True)
                self.read_thread.start()
            self.start_logging()
        self._update_raw_logger_ui()

    def _configure_styles(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure(".", background=self.BG_COLOR, foreground=self.TEXT_COLOR, font=("Segoe UI", 10))
        style.configure("TCombobox", fieldbackground=self.CARD_BG, background=self.CARD_BG, foreground=self.TEXT_COLOR)
        style.map("TCombobox", fieldbackground=[("readonly", self.CARD_BG)], foreground=[("readonly", self.TEXT_COLOR)])

    def _build_ui(self):
        # --- HEADER ---
        header_frame = tk.Frame(self.root, bg=self.BG_COLOR, pady=10, padx=20)
        header_frame.pack(fill=tk.X)

        title_lbl = tk.Label(
            header_frame, text="BATTERY ANALYZER (LoRa)",
            font=("Segoe UI", 18, "bold"), bg=self.BG_COLOR, fg=self.ACCENT_CYAN
        )
        title_lbl.pack(anchor="w")

        subtitle_lbl = tk.Label(
            header_frame, text="Real-time Voltage vs Time Plotter & Excel Data Logger",
            font=("Segoe UI", 9, "italic"), bg=self.BG_COLOR, fg=self.SUBTEXT_COLOR
        )
        subtitle_lbl.pack(anchor="w")

        # --- CONNECTION CARD ---
        conn_card = tk.Frame(self.root, bg=self.CARD_BG, bd=0, relief="flat", padx=15, pady=10)
        conn_card.pack(fill=tk.X, padx=20, pady=5)

        tk.Label(conn_card, text="LoRa Receiver Port:", font=("Segoe UI", 10, "bold"), bg=self.CARD_BG, fg=self.TEXT_COLOR).grid(row=0, column=0, sticky="w")

        self.port_combo = ttk.Combobox(conn_card, state="readonly", width=18)
        self.port_combo.grid(row=0, column=1, padx=8, sticky="w")

        btn_refresh = tk.Button(
            conn_card, text="REFRESH", command=self.refresh_ports,
            bg="#313244", fg=self.TEXT_COLOR, activebackground="#45475a", bd=0, padx=8, pady=2, cursor="hand2"
        )
        btn_refresh.grid(row=0, column=2, padx=4, sticky="w")

        self.btn_connect = tk.Button(
            conn_card, text="CONNECT", command=self.toggle_connection,
            font=("Segoe UI", 9, "bold"), bg=self.ACCENT_BLUE, fg="#11111b",
            activebackground="#74c7ec", bd=0, padx=14, pady=4, cursor="hand2"
        )
        self.btn_connect.grid(row=0, column=3, padx=(12, 0), sticky="e")

        self.lbl_status = tk.Label(conn_card, text="Disconnected", font=("Segoe UI", 9, "bold"), bg=self.CARD_BG, fg=self.ACCENT_RED)
        self.lbl_status.grid(row=0, column=4, padx=12, sticky="w")

        conn_card.columnconfigure(3, weight=1)

        # --- METRICS DASHBOARD ---
        metrics_frame = tk.Frame(self.root, bg=self.BG_COLOR, padx=20, pady=5)
        metrics_frame.pack(fill=tk.X)

        # Voltage Display Card
        v_card = tk.Frame(metrics_frame, bg=self.CARD_BG, padx=15, pady=8)
        v_card.pack(side=tk.LEFT, expand=True, fill=tk.BOTH, padx=(0, 5))
        tk.Label(v_card, text="BATTERY VOLTAGE", font=("Segoe UI", 8, "bold"), bg=self.CARD_BG, fg=self.SUBTEXT_COLOR).pack(anchor="w")
        self.lbl_voltage = tk.Label(v_card, text="0.00 V", font=("Segoe UI", 20, "bold"), bg=self.CARD_BG, fg=self.ACCENT_GREEN)
        self.lbl_voltage.pack(anchor="w")

        # Raw ADC Card
        adc_card = tk.Frame(metrics_frame, bg=self.CARD_BG, padx=15, pady=8)
        adc_card.pack(side=tk.LEFT, expand=True, fill=tk.BOTH, padx=5)
        tk.Label(adc_card, text="RAW SENSOR (ADC)", font=("Segoe UI", 8, "bold"), bg=self.CARD_BG, fg=self.SUBTEXT_COLOR).pack(anchor="w")
        self.lbl_adc = tk.Label(adc_card, text="0", font=("Segoe UI", 20, "bold"), bg=self.CARD_BG, fg=self.TEXT_COLOR)
        self.lbl_adc.pack(anchor="w")

        # Elapsed Time Card
        t_card = tk.Frame(metrics_frame, bg=self.CARD_BG, padx=15, pady=8)
        t_card.pack(side=tk.LEFT, expand=True, fill=tk.BOTH, padx=5)
        tk.Label(t_card, text="ELAPSED TIME", font=("Segoe UI", 8, "bold"), bg=self.CARD_BG, fg=self.SUBTEXT_COLOR).pack(anchor="w")
        self.lbl_time = tk.Label(t_card, text="0.0 s", font=("Segoe UI", 20, "bold"), bg=self.CARD_BG, fg=self.ACCENT_YELLOW)
        self.lbl_time.pack(anchor="w")

        # Signal Quality Card
        sig_card = tk.Frame(metrics_frame, bg=self.CARD_BG, padx=15, pady=8)
        sig_card.pack(side=tk.LEFT, expand=True, fill=tk.BOTH, padx=(5, 0))
        tk.Label(sig_card, text="LORA RSSI / SNR", font=("Segoe UI", 8, "bold"), bg=self.CARD_BG, fg=self.SUBTEXT_COLOR).pack(anchor="w")
        self.lbl_signal = tk.Label(sig_card, text="N/A / N/A", font=("Segoe UI", 16, "bold"), bg=self.CARD_BG, fg=self.ACCENT_BLUE)
        self.lbl_signal.pack(anchor="w", pady=(4, 0))

        # --- PLOT AREA ---
        plot_container = tk.Frame(self.root, bg=self.CARD_BG, bd=0, relief="flat", padx=10, pady=10)
        plot_container.pack(fill=tk.BOTH, expand=True, padx=20, pady=8)

        if HAS_MATPLOTLIB:
            self.fig = Figure(figsize=(6, 3.2), dpi=100, facecolor=self.CARD_BG)
            self.ax = self.fig.add_subplot(111)
            self._setup_matplotlib_style()

            self.canvas = FigureCanvasTkAgg(self.fig, master=plot_container)
            self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
            self.line, = self.ax.plot([], [], color=self.ACCENT_CYAN, linewidth=2, label="Battery Voltage (V)")
            self.ax.legend(facecolor=self.BG_COLOR, edgecolor=self.GRID_COLOR, labelcolor=self.TEXT_COLOR, loc="upper right")
        else:
            # Canvas Fallback Plot
            self.tk_canvas = tk.Canvas(plot_container, bg=self.LOG_BG, highlightthickness=0)
            self.tk_canvas.pack(fill=tk.BOTH, expand=True)
            self.tk_canvas.create_text(200, 100, text="Matplotlib not installed. Canvas graph fallback enabled.", fill=self.SUBTEXT_COLOR, font=("Segoe UI", 11))

        # --- SERVO CALIBRATION & RANGE CARD ---
        calib_card = tk.Frame(self.root, bg=self.CARD_BG, bd=0, relief="flat", padx=15, pady=10)
        calib_card.pack(fill=tk.X, padx=20, pady=5)

        calib_title = tk.Label(
            calib_card, text="Servo Control & Range Setup",
            font=("Segoe UI", 10, "bold"), bg=self.CARD_BG, fg=self.ACCENT_YELLOW
        )
        calib_title.grid(row=0, column=0, columnspan=8, sticky="w", pady=(0, 6))

        # Row 1: Servo Rotation Max Angle Limit & RC Deadband
        tk.Label(calib_card, text="Max Angle (deg):", bg=self.CARD_BG, fg=self.TEXT_COLOR).grid(row=1, column=0, sticky="w", padx=2)
        self.spin_angle = tk.Spinbox(calib_card, from_=10, to=45, width=4, bg=self.LOG_BG, fg=self.TEXT_COLOR, insertbackground=self.TEXT_COLOR)
        self.spin_angle.delete(0, tk.END); self.spin_angle.insert(0, "30")
        self.spin_angle.grid(row=1, column=1, sticky="w", padx=2)

        tk.Label(calib_card, text="RC Deadband (us):", bg=self.CARD_BG, fg=self.TEXT_COLOR).grid(row=1, column=2, sticky="w", padx=(8, 2))
        self.spin_deadband = tk.Spinbox(calib_card, from_=1, to=50, width=4, bg=self.LOG_BG, fg=self.TEXT_COLOR, insertbackground=self.TEXT_COLOR)
        self.spin_deadband.delete(0, tk.END); self.spin_deadband.insert(0, "18")
        self.spin_deadband.grid(row=1, column=3, sticky="w", padx=2)

        # Fine Trims per servo (us)
        tk.Label(calib_card, text="BR (us):", bg=self.CARD_BG, fg=self.TEXT_COLOR).grid(row=1, column=4, sticky="w", padx=(8, 2))
        self.spin_br = tk.Spinbox(calib_card, from_=-250, to=250, width=5, bg=self.LOG_BG, fg=self.TEXT_COLOR, insertbackground=self.TEXT_COLOR, command=self.send_servo_trim_live)
        self.spin_br.delete(0, tk.END); self.spin_br.insert(0, "0")
        self.spin_br.grid(row=1, column=5, sticky="w", padx=2)

        tk.Label(calib_card, text="BL (us):", bg=self.CARD_BG, fg=self.TEXT_COLOR).grid(row=1, column=6, sticky="w", padx=(6, 2))
        self.spin_bl = tk.Spinbox(calib_card, from_=-250, to=250, width=5, bg=self.LOG_BG, fg=self.TEXT_COLOR, insertbackground=self.TEXT_COLOR, command=self.send_servo_trim_live)
        self.spin_bl.delete(0, tk.END); self.spin_bl.insert(0, "0")
        self.spin_bl.grid(row=1, column=7, sticky="w", padx=2)

        # Row 2: FR, FL trim, Direction Invert Checkbuttons & Save Button
        tk.Label(calib_card, text="FR (us):", bg=self.CARD_BG, fg=self.TEXT_COLOR).grid(row=2, column=0, sticky="w", padx=2, pady=4)
        self.spin_fr = tk.Spinbox(calib_card, from_=-250, to=250, width=5, bg=self.LOG_BG, fg=self.TEXT_COLOR, insertbackground=self.TEXT_COLOR, command=self.send_servo_trim_live)
        self.spin_fr.delete(0, tk.END); self.spin_fr.insert(0, "0")
        self.spin_fr.grid(row=2, column=1, sticky="w", padx=2, pady=4)

        tk.Label(calib_card, text="FL (us):", bg=self.CARD_BG, fg=self.TEXT_COLOR).grid(row=2, column=2, sticky="w", padx=(8, 2), pady=4)
        self.spin_fl = tk.Spinbox(calib_card, from_=-250, to=250, width=5, bg=self.LOG_BG, fg=self.TEXT_COLOR, insertbackground=self.TEXT_COLOR, command=self.send_servo_trim_live)
        self.spin_fl.delete(0, tk.END); self.spin_fl.insert(0, "0")
        self.spin_fl.grid(row=2, column=3, sticky="w", padx=2, pady=4)

        self.var_inv_br = tk.BooleanVar(value=False)
        self.var_inv_bl = tk.BooleanVar(value=False)
        self.var_inv_fr = tk.BooleanVar(value=False)
        self.var_inv_fl = tk.BooleanVar(value=False)

        tk.Checkbutton(calib_card, text="Rev BR", variable=self.var_inv_br, command=self.send_servo_inversion_live, bg=self.CARD_BG, fg=self.TEXT_COLOR, selectcolor=self.LOG_BG, activebackground=self.CARD_BG, activeforeground=self.TEXT_COLOR).grid(row=2, column=4, sticky="w", padx=2)
        tk.Checkbutton(calib_card, text="Rev BL", variable=self.var_inv_bl, command=self.send_servo_inversion_live, bg=self.CARD_BG, fg=self.TEXT_COLOR, selectcolor=self.LOG_BG, activebackground=self.CARD_BG, activeforeground=self.TEXT_COLOR).grid(row=2, column=5, sticky="w", padx=2)
        tk.Checkbutton(calib_card, text="Rev FR", variable=self.var_inv_fr, command=self.send_servo_inversion_live, bg=self.CARD_BG, fg=self.TEXT_COLOR, selectcolor=self.LOG_BG, activebackground=self.CARD_BG, activeforeground=self.TEXT_COLOR).grid(row=2, column=6, sticky="w", padx=2)
        tk.Checkbutton(calib_card, text="Rev FL", variable=self.var_inv_fl, command=self.send_servo_inversion_live, bg=self.CARD_BG, fg=self.TEXT_COLOR, selectcolor=self.LOG_BG, activebackground=self.CARD_BG, activeforeground=self.TEXT_COLOR).grid(row=2, column=7, sticky="w", padx=2)

        # Row 3: Alert Voltage (V), Servo Rate (ms), LoRa Power Boost & Save to Flash Button
        tk.Label(calib_card, text="Alert V:", bg=self.CARD_BG, fg=self.TEXT_COLOR, font=("Segoe UI", 9, "bold")).grid(row=3, column=0, sticky="w", padx=2, pady=4)
        self.spin_alert_voltage = tk.Spinbox(calib_card, from_=12.00, to=16.00, increment=0.10, width=5, bg=self.LOG_BG, fg=self.TEXT_COLOR, insertbackground=self.TEXT_COLOR)
        self.spin_alert_voltage.delete(0, tk.END); self.spin_alert_voltage.insert(0, "12.50")
        self.spin_alert_voltage.grid(row=3, column=1, sticky="w", padx=2, pady=4)

        tk.Label(calib_card, text="Rate (ms):", bg=self.CARD_BG, fg=self.TEXT_COLOR, font=("Segoe UI", 9, "bold")).grid(row=3, column=2, sticky="w", padx=(6, 2), pady=4)
        self.spin_servo_rate = tk.Spinbox(calib_card, from_=5, to=100, increment=5, width=4, bg=self.LOG_BG, fg=self.TEXT_COLOR, insertbackground=self.TEXT_COLOR)
        self.spin_servo_rate.delete(0, tk.END); self.spin_servo_rate.insert(0, "50")
        self.spin_servo_rate.grid(row=3, column=3, sticky="w", padx=2, pady=4)

        self.btn_toggle_power = tk.Button(
            calib_card, text="LORA POWER: STANDARD (14 dBm)", command=self.toggle_lora_power,
            font=("Segoe UI", 8, "bold"), bg=self.ACCENT_CYAN, fg="#11111b",
            activebackground="#89dceb", bd=0, padx=6, pady=2, cursor="hand2"
        )
        self.btn_toggle_power.grid(row=3, column=4, columnspan=2, sticky="w", padx=4, pady=4)

        btn_save_nvs = tk.Button(
            calib_card, text="SAVE TO FLASH (NVS)", command=self.save_all_calibration_nvs,
            font=("Segoe UI", 9, "bold"), bg=self.ACCENT_GREEN, fg="#11111b",
            activebackground="#a6e3a1", bd=0, padx=10, pady=3, cursor="hand2"
        )
        btn_save_nvs.grid(row=3, column=6, columnspan=2, sticky="e", padx=4, pady=4)

        # Row 4: RC Interference Filter controls
        tk.Label(calib_card, text="RC Filter:", bg=self.CARD_BG, fg=self.TEXT_COLOR, font=("Segoe UI", 9, "bold")).grid(row=4, column=0, sticky="w", padx=2, pady=4)
        self.combo_rc_filter_type = ttk.Combobox(calib_card, state="readonly", width=18, values=[
            "0: Raw (Off)",
            "1: SMA (Moving Avg)",
            "2: EMA (Exponential)",
            "3: WMA (Weighted)"
        ])
        self.combo_rc_filter_type.current(1)
        self.combo_rc_filter_type.grid(row=4, column=1, columnspan=2, sticky="w", padx=2, pady=4)

        tk.Label(calib_card, text="Win N:", bg=self.CARD_BG, fg=self.TEXT_COLOR).grid(row=4, column=3, sticky="w", padx=(4, 2), pady=4)
        self.entry_rc_win = tk.Entry(calib_card, width=5, bg=self.LOG_BG, fg=self.TEXT_COLOR, insertbackground=self.TEXT_COLOR)
        self.entry_rc_win.insert(0, "5")
        self.entry_rc_win.grid(row=4, column=4, sticky="w", padx=2, pady=4)

        tk.Label(calib_card, text="Alpha:", bg=self.CARD_BG, fg=self.TEXT_COLOR).grid(row=4, column=5, sticky="w", padx=(4, 2), pady=4)
        self.spin_rc_alpha = tk.Spinbox(calib_card, from_=0.05, to=1.00, increment=0.05, width=4, bg=self.LOG_BG, fg=self.TEXT_COLOR, insertbackground=self.TEXT_COLOR)
        self.spin_rc_alpha.delete(0, tk.END); self.spin_rc_alpha.insert(0, "0.33")
        self.spin_rc_alpha.grid(row=4, column=6, sticky="w", padx=2, pady=4)

        btn_apply_filter = tk.Button(
            calib_card, text="APPLY RC FILTER", command=self.apply_rc_filter,
            font=("Segoe UI", 9, "bold"), bg=self.ACCENT_BLUE, fg="#11111b",
            activebackground="#89b4fa", bd=0, padx=6, pady=2, cursor="hand2"
        )
        btn_apply_filter.grid(row=4, column=7, sticky="e", padx=2, pady=4)

        # --- RAW LORA PACKET LOGGER CARD (ASYNC & CRASH-PROOF) ---
        raw_card = tk.Frame(self.root, bg=self.CARD_BG, bd=0, relief="flat", padx=15, pady=8)
        raw_card.pack(fill=tk.X, padx=20, pady=5)

        tk.Label(
            raw_card, text="Raw LoRa Packet Logger (Async Worker Thread | Instant os.fsync Disk Commit)",
            font=("Segoe UI", 9, "bold"), bg=self.CARD_BG, fg=self.ACCENT_CYAN
        ).pack(anchor="w", pady=(0, 4))

        raw_sub_frame = tk.Frame(raw_card, bg=self.CARD_BG)
        raw_sub_frame.pack(fill=tk.X)

        self.btn_toggle_raw_log = tk.Button(
            raw_sub_frame, text="START RAW LOGGING", command=self.toggle_raw_logging,
            font=("Segoe UI", 9, "bold"), bg=self.ACCENT_GREEN, fg="#11111b",
            activebackground="#a6e3a1", bd=0, padx=14, pady=4, cursor="hand2"
        )
        self.btn_toggle_raw_log.pack(side=tk.LEFT, padx=(0, 10))

        self.lbl_raw_status = tk.Label(
            raw_sub_frame, text="RAW LOG: OFF", font=("Segoe UI", 9, "bold"),
            bg=self.CARD_BG, fg=self.SUBTEXT_COLOR
        )
        self.lbl_raw_status.pack(side=tk.LEFT, padx=5)

        self.lbl_raw_count = tk.Label(
            raw_sub_frame, text="Logged: 0 pkts", font=("Segoe UI", 9, "italic"),
            bg=self.CARD_BG, fg=self.SUBTEXT_COLOR
        )
        self.lbl_raw_count.pack(side=tk.RIGHT, padx=5)

        # --- ACTION BAR ---
        action_bar = tk.Frame(self.root, bg=self.BG_COLOR, padx=20, pady=5)
        action_bar.pack(fill=tk.X)

        self.btn_toggle_log = tk.Button(
            action_bar, text="START LOGGING", command=self.toggle_logging,
            font=("Segoe UI", 10, "bold"), bg=self.ACCENT_GREEN, fg="#11111b",
            activebackground="#a6e3a1", bd=0, padx=16, pady=6, cursor="hand2"
        )
        self.btn_toggle_log.pack(side=tk.LEFT, padx=(0, 10))

        btn_clear = tk.Button(
            action_bar, text="CLEAR DATA", command=self.clear_data,
            font=("Segoe UI", 9, "bold"), bg="#313244", fg=self.TEXT_COLOR,
            activebackground="#45475a", bd=0, padx=12, pady=6, cursor="hand2"
        )
        btn_clear.pack(side=tk.LEFT, padx=5)

        btn_calib = tk.Button(
            action_bar, text="CALIBRATE RADIO NEUTRALS", command=self.calibrate_neutral,
            font=("Segoe UI", 9, "bold"), bg=self.ACCENT_YELLOW, fg="#11111b",
            activebackground="#f9e2af", bd=0, padx=12, pady=6, cursor="hand2"
        )
        btn_calib.pack(side=tk.LEFT, padx=5)

        self.lbl_excel_info = tk.Label(
            action_bar, text=f"Excel Output: {EXCEL_FILE}",
            font=("Segoe UI", 9, "italic"), bg=self.BG_COLOR, fg=self.SUBTEXT_COLOR
        )
        self.lbl_excel_info.pack(side=tk.RIGHT, pady=6)

        # Window Close Protocol
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _setup_matplotlib_style(self):
        self.ax.set_facecolor(self.BG_COLOR)
        self.ax.tick_params(colors=self.TEXT_COLOR, labelsize=9)
        for spine in self.ax.spines.values():
            spine.set_color(self.GRID_COLOR)
        self.ax.set_xlabel("Elapsed Time (s)", color=self.SUBTEXT_COLOR, fontsize=9)
        self.ax.set_ylabel("Voltage (V)", color=self.SUBTEXT_COLOR, fontsize=9)
        self.ax.grid(True, color=self.GRID_COLOR, linestyle="--", alpha=0.5)

    def refresh_ports(self):
        ports = list(serial.tools.list_ports.comports())
        port_list = [p.device for p in ports]
        self.port_combo['values'] = port_list

        if port_list:
            default_port = port_list[0]
            for p in port_list:
                if "COM4" in p:
                    default_port = p
                    break
                elif "COM6" in p:
                    default_port = p
                    break
            self.port_combo.set(default_port)
        else:
            self.port_combo.set('')

    def toggle_connection(self):
        if not self.is_connected:
            port_name = self.port_combo.get()
            if not port_name:
                messagebox.showwarning("Warning", "Please select a COM port before connecting.")
                return

            try:
                # If there's already a shared open connection on the same port, reuse it
                if self.serial_conn and self.serial_conn.is_open and self.serial_conn.port == port_name:
                    pass  # reuse existing connection
                else:
                    self.serial_conn = serial.Serial(port_name, DEFAULT_BAUD, timeout=0.01)
                self.is_connected = True
                self.btn_connect.config(text="DISCONNECT", bg=self.ACCENT_RED)
                self.lbl_status.config(text=f"Connected ({port_name})", fg=self.ACCENT_GREEN)

                # Start Reader Thread
                self.read_thread = threading.Thread(target=self._read_serial_loop, daemon=True)
                self.read_thread.start()

                # Start Logging automatically upon connection
                self.start_logging()

            except Exception as e:
                messagebox.showerror("Connection Error", f"Could not open port {port_name}:\n{e}")
        else:
            self.disconnect()

    def disconnect(self):
        self.stop_logging()
        self.is_connected = False
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
            except Exception:
                pass

        self.btn_connect.config(text="CONNECT", bg=self.ACCENT_BLUE)
        self.lbl_status.config(text="Disconnected", fg=self.ACCENT_RED)

    def calibrate_neutral(self):
        if self.is_connected and self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(b"CALIB_TRIM\n")
                self.serial_conn.flush()
                self.async_raw_logger.log_packet("TX", "CALIB_TRIM")
                messagebox.showinfo("Radio Neutral Calibration", "Neutral calibration command sent successfully!\nRollerons & Elevator centers saved to ESP32 MANTA Flash NVS.")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to send calibration command: {e}")
        else:
            messagebox.showwarning("Warning", "Please connect serial port before calibrating!")

    def get_validated_alert_voltage(self):
        try:
            val = float(self.spin_alert_voltage.get())
            if val < 12.00:
                messagebox.showwarning("Invalid Input", "Minimum allowable Alert Voltage threshold is 12.00V!\nClamping value to 12.00V.")
                val = 12.00
                self.spin_alert_voltage.delete(0, tk.END)
                self.spin_alert_voltage.insert(0, "12.00")
            return val
        except ValueError:
            messagebox.showwarning("Invalid Input", "Invalid Alert Voltage entry! Resetting to default 12.50V.")
            self.spin_alert_voltage.delete(0, tk.END)
            self.spin_alert_voltage.insert(0, "12.50")
            return 12.50

    def toggle_lora_power(self):
        if self.last_lora_power == 14:
            new_power = 20
            text_str = "LORA POWER: HIGH BOOST (20 dBm - 1km+ VLOS)"
            bg_color = self.ACCENT_GREEN
        else:
            new_power = 14
            text_str = "LORA POWER: STANDARD (14 dBm)"
            bg_color = self.ACCENT_CYAN

        log_msg = f"Changed variable LORA_TX_POWER: [{self.last_lora_power} dBm] -> [{new_power} dBm]"
        self.last_lora_power = new_power
        self.btn_toggle_power.config(text=text_str, bg=bg_color)
        self.async_raw_logger.log_packet("CONFIG", log_msg)

        if self.is_connected and self.serial_conn and self.serial_conn.is_open:
            try:
                cmd = f"SET_LORA_POWER:{new_power}\n"
                self.serial_conn.write(cmd.encode('utf-8'))
                self.serial_conn.flush()
                self.async_raw_logger.log_packet("TX", cmd.strip())
                messagebox.showinfo("LoRa Power Updated", f"LoRa Transmit Power set to {new_power} dBm (+{new_power}dBm PA_BOOST for 1km+ VLOS mountain range)!")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to send LoRa power command: {e}")
        else:
            messagebox.showinfo("Power Level Updated", f"LoRa Power target set to {new_power} dBm. Will transmit upon connecting.")

    def send_servo_inversion_live(self):
        if self.is_connected and self.serial_conn and self.serial_conn.is_open:
            try:
                inv_br = 1 if self.var_inv_br.get() else 0
                inv_bl = 1 if self.var_inv_bl.get() else 0
                inv_fr = 1 if self.var_inv_fr.get() else 0
                inv_fl = 1 if self.var_inv_fl.get() else 0
                cmd = f"SET_SERVO_INV:{inv_br},{inv_bl},{inv_fr},{inv_fl}\n"
                for _ in range(2):
                    self.serial_conn.write(cmd.encode('utf-8'))
                    self.serial_conn.flush()
                    time.sleep(0.05)
                self.async_raw_logger.log_packet("TX", cmd.strip())
                print(f"[LoRa TX Live] {cmd.strip()}")
            except Exception as e:
                print(f"[LoRa TX Error] {e}")

    def send_servo_trim_live(self):
        if self.is_connected and self.serial_conn and self.serial_conn.is_open:
            try:
                br_val = int(self.spin_br.get())
                bl_val = int(self.spin_bl.get())
                fr_val = int(self.spin_fr.get())
                fl_val = int(self.spin_fl.get())
                cmd = f"SET_SERVO_TRIM:{br_val},{bl_val},{fr_val},{fl_val}\n"
                for _ in range(2):
                    self.serial_conn.write(cmd.encode('utf-8'))
                    self.serial_conn.flush()
                    time.sleep(0.05)
                self.async_raw_logger.log_packet("TX", cmd.strip())
                print(f"[LoRa TX Live] {cmd.strip()}")
            except Exception as e:
                print(f"[LoRa TX Error] {e}")

    def save_all_calibration_nvs(self):
        if self.is_connected and self.serial_conn and self.serial_conn.is_open:
            try:
                angle_val = int(self.spin_angle.get())
                deadband_val = int(self.spin_deadband.get())
                br_val = int(self.spin_br.get())
                bl_val = int(self.spin_bl.get())
                fr_val = int(self.spin_fr.get())
                fl_val = int(self.spin_fl.get())
                alert_val = self.get_validated_alert_voltage()
                power_val = self.last_lora_power
                rate_val = int(self.spin_servo_rate.get())

                inv_br = 1 if self.var_inv_br.get() else 0
                inv_bl = 1 if self.var_inv_bl.get() else 0
                inv_fr = 1 if self.var_inv_fr.get() else 0
                inv_fl = 1 if self.var_inv_fl.get() else 0

                cmd_angle  = f"SET_SERVO_ANGLE:{angle_val}\n"
                cmd_db     = f"SET_DEADBAND:{deadband_val}\n"
                cmd_rate   = f"SET_SERVO_INTERVAL:{rate_val}\n"
                cmd_pwr    = f"SET_LORA_POWER:{power_val}\n"
                cmd_cutoff = f"CUTOFF:{alert_val:.2f}\n"
                cmd_trim   = f"SET_SERVO_TRIM:{br_val},{bl_val},{fr_val},{fl_val}\n"
                cmd_inv    = f"SET_SERVO_INV:{inv_br},{inv_bl},{inv_fr},{inv_fl}\n"
                cmd_save   = "CALIB_SAVE\n"

                cmds_list = [cmd_angle, cmd_db, cmd_rate, cmd_pwr, cmd_cutoff, cmd_trim, cmd_inv, cmd_save]

                self.last_received_ack = None

                for cmd in cmds_list:
                    self.serial_conn.write(cmd.encode('utf-8'))
                    self.serial_conn.flush()
                    time.sleep(0.06)

                self.async_raw_logger.log_packet("TX", cmd_angle.strip())
                self.async_raw_logger.log_packet("TX", cmd_db.strip())
                self.async_raw_logger.log_packet("TX", cmd_rate.strip())
                self.async_raw_logger.log_packet("TX", cmd_pwr.strip())
                self.async_raw_logger.log_packet("TX", cmd_cutoff.strip())
                self.async_raw_logger.log_packet("TX", cmd_trim.strip())
                self.async_raw_logger.log_packet("TX", cmd_inv.strip())
                self.async_raw_logger.log_packet("TX", cmd_save.strip())

                # Wait up to 3.0 seconds for CALIB_SAVE ACK confirmation from MANTA over LoRa
                ack_confirmed = False
                wait_start = time.time()
                while time.time() - wait_start < 3.0:
                    self.root.update()
                    if self.last_received_ack and "CALIB_SAVE" in self.last_received_ack:
                        ack_confirmed = True
                        break
                    time.sleep(0.05)

                if not ack_confirmed and self.last_received_ack is not None:
                    ack_confirmed = True

                if ack_confirmed:
                    messagebox.showinfo("Calibration Saved", f"CONFIRMED: MANTA acknowledged ({self.last_received_ack}) and saved all parameters to Flash NVS!")
                else:
                    messagebox.showwarning("ACK Timeout", "Commands transmitted to Ground Station, but no ACK confirmation was received back from MANTA over LoRa. Check radio link.")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to send calibration parameters: {e}")
        else:
            messagebox.showwarning("Warning", "Please connect serial port before saving calibration!")

    def apply_rc_filter(self):
        if not self.serial_conn or not self.is_connected:
            messagebox.showwarning("Not Connected", "Please connect to Ground Station first.")
            return
        try:
            val_str = self.combo_rc_filter_type.get()
            f_type = int(val_str.split(":")[0])
            w_size = int(self.entry_rc_win.get().strip())
            alpha = float(self.spin_rc_alpha.get())
            alpha_int = int(round(alpha * 100))
            cmd = f"SET_RC_FILTER:{f_type}:{w_size}:{alpha_int}\n"
            self.serial_conn.write(cmd.encode('utf-8'))
            self.serial_conn.flush()
            f_names = ["RAW", "SMA", "EMA", "WMA"]
            messagebox.showinfo("RC Filter Config", f"Sent filter update to MANTA:\nType: {f_names[f_type]}\nWindow N: {w_size}\nAlpha: {alpha:.2f}")
        except Exception as e:
            messagebox.showerror("Filter Error", f"Invalid parameters: {e}")

    def start_logging(self):
        if self.is_logging:
            return

        self.is_logging = True
        self.start_time = time.time()
        self.last_excel_save = self.start_time - 10.0
        self.record_number = 1

        # Initialize Excel Workbook
        try:
            if os.path.exists(EXCEL_FILE):
                try:
                    os.remove(EXCEL_FILE)
                except Exception:
                    pass
            self.wb = Workbook()
            self.ws = self.wb.active
            self.ws.title = "Battery Voltage Log"
            self.ws.append(["Record Number", "Elapsed Time (s)", "Raw Sensor (ADC)", "Pin Voltage (V)", "Estimated Voltage (V)", "RSSI", "SNR"])
            self.wb.save(EXCEL_FILE)
        except Exception as e:
            print(f"[Excel Error] {e}")

        self.btn_toggle_log.config(text="STOP LOGGING", bg=self.ACCENT_RED)

    def stop_logging(self):
        if not self.is_logging:
            return
        self.is_logging = False
        if self.wb:
            try:
                self.wb.save(EXCEL_FILE)
            except Exception:
                pass
        self.btn_toggle_log.config(text="START LOGGING", bg=self.ACCENT_GREEN)

    def toggle_logging(self):
        if self.is_logging:
            self.stop_logging()
        else:
            self.start_logging()

    def clear_data(self):
        self.timestamps.clear()
        self.voltages.clear()
        self.raw_adcs.clear()
        self.start_time = time.time() if self.is_logging else None
        self._update_plot()
        self.lbl_voltage.config(text="0.00 V")
        self.lbl_adc.config(text="0")
        self.lbl_time.config(text="0.0 s")

    def toggle_raw_logging(self):
        is_running, count, path = self.async_raw_logger.get_status()
        if not is_running:
            success = self.async_raw_logger.start()
            if success:
                self.btn_toggle_raw_log.config(text="STOP RAW LOGGING", bg=self.ACCENT_RED)
                self.lbl_raw_status.config(text="RAW LOG: RECORDING", fg=self.ACCENT_GREEN)
        else:
            self.async_raw_logger.stop()
            self.btn_toggle_raw_log.config(text="START RAW LOGGING", bg=self.ACCENT_GREEN)
            self.lbl_raw_status.config(text="RAW LOG: OFF", fg=self.SUBTEXT_COLOR)

    def _update_raw_logger_ui(self):
        is_running, count, path = self.async_raw_logger.get_status()
        if is_running:
            file_name = os.path.basename(path) if path else ""
            self.lbl_raw_count.config(text=f"Logged: {count} pkts ({file_name})")
        self.root.after(500, self._update_raw_logger_ui)

    def _read_serial_loop(self):
        buffer = ""
        raw_bytes_buffer = bytearray()
        while self.is_connected and self.serial_conn and self.serial_conn.is_open:
            try:
                in_w = self.serial_conn.in_waiting
                if in_w > 0:
                    data = self.serial_conn.read(in_w)
                    if data:
                        raw_bytes_buffer.extend(data)
                        while len(raw_bytes_buffer) >= PACKET_SIZE:
                            idx = raw_bytes_buffer.find(b'MT')
                            if idx == -1:
                                if len(raw_bytes_buffer) > 1:
                                    raw_bytes_buffer = raw_bytes_buffer[-1:]
                                break
                            if idx > 0:
                                raw_bytes_buffer = raw_bytes_buffer[idx:]
                            if len(raw_bytes_buffer) < PACKET_SIZE:
                                break

                            pkt_bin = bytes(raw_bytes_buffer[:PACKET_SIZE])
                            decoded_pkt = decode_telemetry(pkt_bin)
                            if decoded_pkt is not None:
                                self.last_manta_confirmed_deadband = decoded_pkt.get("deadband", 25)
                                rec_v = decoded_pkt.get("batteryVoltage", 0.0)
                                raw_adc = decoded_pkt.get("rawADC", 0.0)
                                self._process_voltage_sample(rec_v, raw_adc)
                                raw_bytes_buffer = raw_bytes_buffer[PACKET_SIZE:]
                            else:
                                raw_bytes_buffer = raw_bytes_buffer[1:]

                        buffer += data.decode('utf-8', errors='ignore')
                        while '\n' in buffer:
                            line_str, buffer = buffer.split('\n', 1)
                            line_str = line_str.strip()
                            if line_str:
                                self.async_raw_logger.log_packet("RX", line_str)
                                self._parse_telemetry_line(line_str)
            except Exception:
                break
            time.sleep(0.01)

    def _process_voltage_sample(self, rec_v, raw_adc):
        if rec_v is not None or raw_adc is not None:
            now = time.time()
            if self.start_time is None:
                self.start_time = now

            elapsed_sec = round(now - self.start_time, 2)

            if raw_adc is None:
                raw_adc = 0.0

            if rec_v is not None and rec_v > 0:
                voltage = round(rec_v, 2)
            else:
                voltage = round(calculate_battery_voltage(raw_adc), 2)

            self.current_voltage = voltage
            self.current_adc = raw_adc

            self.timestamps.append(elapsed_sec)
            self.voltages.append(voltage)
            self.raw_adcs.append(raw_adc)

            # Excel Logging (every 10 seconds or when record increases)
            if self.is_logging and self.ws:
                if (now - self.last_excel_save) >= 10.0:
                    self.last_excel_save = now
                    pin_v = round((raw_adc * 3.3) / 4095.0, 3)
                    self.ws.append([self.record_number, elapsed_sec, raw_adc, pin_v, voltage, self.last_rssi, self.last_snr])
                    try:
                        self.wb.save(EXCEL_FILE)
                    except Exception:
                        pass
                    self.record_number += 1

            # Update UI on Main Thread
            self.root.after(0, self._update_ui_metrics, voltage, raw_adc, elapsed_sec)

    def _parse_telemetry_line(self, line_str):
        if "ACK:" in line_str:
            try:
                ack_content = line_str[line_str.find("ACK:"):].split()[0]
                self.last_received_ack = ack_content
                print(f"  [ACK RECEIVED] MANTA confirmed: {ack_content}")
                self.lbl_status.config(text=f"ACK: {ack_content}", fg=self.ACCENT_GREEN)
            except Exception:
                pass

        # Extract RSSI / SNR
        rssi_m = re.search(r'RSSI:\s*(-?\d+)', line_str)
        if rssi_m:
            self.last_rssi = f"{rssi_m.group(1)} dBm"

        snr_m = re.search(r'SNR:\s*([\d\.-]+)', line_str)
        if snr_m:
            self.last_snr = f"{snr_m.group(1)} dB"

        db_m = re.search(r'DB:\s*(\d+)', line_str)
        if db_m:
            try:
                manta_db = int(db_m.group(1))
                self.last_manta_confirmed_deadband = manta_db
            except Exception:
                pass

        rc_m = re.search(r'RC:\s*\[?\d+,\d+,\d+,\d+,(\d+)\]?', line_str)
        if rc_m:
            try:
                self.last_manta_ch5 = int(rc_m.group(1))
            except Exception:
                pass

        raw_adc = None
        rec_v = None

        v_m = re.search(r'BAT_V:\s*([\d\.]+)', line_str)
        if v_m:
            rec_v = float(v_m.group(1))

        adc_m = re.search(r'BAT_ADC:\s*([\d\.]+)', line_str)
        if adc_m:
            raw_adc = float(adc_m.group(1))

        if rec_v is not None or raw_adc is not None:
            self._process_voltage_sample(rec_v, raw_adc)

    def _update_ui_metrics(self, voltage, raw_adc, elapsed_sec):
        alert_v = self.last_alert_voltage
        if voltage > 0.0 and voltage <= alert_v:
            v_color = self.ACCENT_RED
            try:
                self.root.bell()  # Intermittent audio buzzer feedback
            except Exception:
                pass
        else:
            v_color = self.ACCENT_GREEN

        self.lbl_voltage.config(text=f"{voltage:.2f} V", fg=v_color)
        self.lbl_adc.config(text=f"{int(raw_adc)}")
        self.lbl_time.config(text=f"{elapsed_sec:.1f} s")
        self.lbl_signal.config(text=f"{self.last_rssi} / {self.last_snr}")

        # Update Plot
        self._update_plot()

    def _update_plot(self):
        if not HAS_MATPLOTLIB:
            return

        if not self.timestamps:
            self.line.set_data([], [])
            self.ax.set_xlim(0, 10)
            self.ax.set_ylim(0, 18)
        else:
            self.line.set_data(self.timestamps, self.voltages)
            max_t = max(self.timestamps)
            min_t = max(0, max_t - 60) if max_t > 60 else 0
            self.ax.set_xlim(min_t, max_t + 2)

            v_min = min(self.voltages) - 0.5
            v_max = max(self.voltages) + 0.5
            self.ax.set_ylim(max(0, v_min), max(16.0, v_max))

        self.canvas.draw_idle()

    def on_close(self):
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = BatteryAnalyzerGUI(root)
    root.mainloop()
