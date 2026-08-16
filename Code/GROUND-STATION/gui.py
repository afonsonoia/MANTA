import sys
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports


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
            header_frame, text="Preset Audio & RC Interference Filter Controller (Pin D22)",
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

        # --- RC RECEIVER NOISE FILTER SETUP CARD ---
        filter_card = tk.Frame(self.root, bg=self.CARD_BG, bd=0, relief="flat", padx=15, pady=10)
        filter_card.pack(fill=tk.X, padx=20, pady=4)

        tk.Label(filter_card, text="RC Receiver Interference Noise Filter", font=("Segoe UI", 10, "bold"), bg=self.CARD_BG, fg=self.ACCENT_CYAN).grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 6))

        tk.Label(filter_card, text="Filter Type:", bg=self.CARD_BG, fg=self.TEXT_COLOR).grid(row=1, column=0, sticky="w")
        self.combo_rc_filter = ttk.Combobox(filter_card, state="readonly", width=24, values=[
            "0: Disabled (Raw)",
            "1: Simple Moving Average (SMA)",
            "2: Exponential Moving Average (EMA)",
            "3: Weighted Moving Average (WMA)"
        ])
        self.combo_rc_filter.current(1)
        self.combo_rc_filter.grid(row=1, column=1, columnspan=3, sticky="w", padx=4, pady=2)

        tk.Label(filter_card, text="Window (N):", bg=self.CARD_BG, fg=self.TEXT_COLOR).grid(row=2, column=0, sticky="w", pady=4)
        self.entry_rc_win = tk.Entry(filter_card, width=6, bg=self.LOG_BG, fg=self.TEXT_COLOR, insertbackground=self.TEXT_COLOR)
        self.entry_rc_win.insert(0, "5")
        self.entry_rc_win.grid(row=2, column=1, sticky="w", padx=4, pady=4)

        tk.Label(filter_card, text="Alpha (EMA):", bg=self.CARD_BG, fg=self.TEXT_COLOR).grid(row=2, column=2, sticky="w", padx=(10, 2), pady=4)
        self.spin_rc_alpha = tk.Spinbox(filter_card, from_=0.05, to=1.00, increment=0.05, width=5, bg=self.LOG_BG, fg=self.TEXT_COLOR, insertbackground=self.TEXT_COLOR)
        self.spin_rc_alpha.delete(0, tk.END); self.spin_rc_alpha.insert(0, "0.33")
        self.spin_rc_alpha.grid(row=2, column=3, sticky="w", padx=4, pady=4)

        self.btn_apply_filter = tk.Button(
            filter_card, text="APPLY RC FILTER SETTINGS", command=self.apply_rc_filter,
            font=("Segoe UI", 9, "bold"), bg=self.ACCENT_GREEN, fg="#11111b",
            activebackground="#a6e3a1", bd=0, pady=4, cursor="hand2", state="disabled"
        )
        self.btn_apply_filter.grid(row=3, column=0, columnspan=4, sticky="ew", pady=(6, 0))

        # --- BUZZER CONTROL ACTIONS CARD ---
        ctrl_card = tk.Frame(self.root, bg=self.CARD_BG, bd=0, relief="flat", padx=15, pady=10)
        ctrl_card.pack(fill=tk.X, padx=20, pady=4)

        tk.Label(ctrl_card, text="Buzzer Controls", font=("Segoe UI", 10, "bold"), bg=self.CARD_BG, fg=self.TEXT_COLOR).pack(anchor="w", pady=(0, 6))

        actions_frame = tk.Frame(ctrl_card, bg=self.CARD_BG)
        actions_frame.pack(fill=tk.X)

        self.btn_continuous = tk.Button(
            actions_frame, text="Continuous Beep", command=self.set_continuous,
            font=("Segoe UI", 9, "bold"), bg="#45475a", fg=self.TEXT_COLOR, bd=0, pady=8, state="disabled", cursor="hand2"
        )
        self.btn_continuous.grid(row=0, column=0, padx=4, pady=2, sticky="nsew")

        self.btn_intermittent = tk.Button(
            actions_frame, text="Intermittent Beep", command=self.set_intermittent,
            font=("Segoe UI", 9, "bold"), bg="#45475a", fg=self.TEXT_COLOR, bd=0, pady=8, state="disabled", cursor="hand2"
        )
        self.btn_intermittent.grid(row=0, column=1, padx=4, pady=2, sticky="nsew")

        self.btn_off = tk.Button(
            actions_frame, text="Turn OFF", command=self.set_off,
            font=("Segoe UI", 9, "bold"), bg="#45475a", fg=self.TEXT_COLOR, bd=0, pady=8, state="disabled", cursor="hand2"
        )
        self.btn_off.grid(row=1, column=0, columnspan=2, padx=4, pady=2, sticky="nsew")

        self.btn_calib_trim = tk.Button(
            actions_frame, text="Calibrate Radio Neutrals", command=self.calibrate_trim,
            font=("Segoe UI", 9, "bold"), bg=self.ACCENT_YELLOW, fg="#11111b", bd=0, pady=8, state="disabled", cursor="hand2"
        )
        self.btn_calib_trim.grid(row=2, column=0, columnspan=2, padx=4, pady=2, sticky="nsew")

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
        for btn in [self.btn_continuous, self.btn_intermittent, self.btn_off, self.btn_calib_trim, self.btn_apply_filter]:
            btn.config(state=state)

    def calibrate_trim(self):
        self.send_cmd("CALIB_TRIM")
        messagebox.showinfo("Calibration", "CALIB_TRIM command sent to ESP32 MANTA!")

    def apply_rc_filter(self):
        try:
            val_str = self.combo_rc_filter.get()
            f_type = int(val_str.split(":")[0])
            w_size = int(self.entry_rc_win.get().strip())
            alpha = float(self.spin_rc_alpha.get())
            alpha_int = int(round(alpha * 100))
            cmd = f"SET_RC_FILTER:{f_type}:{w_size}:{alpha_int}"
            self.send_cmd(cmd)
            f_names = ["RAW", "SMA", "EMA", "WMA"]
            messagebox.showinfo("RC Filter Config", f"Sent filter update to MANTA:\nType: {f_names[f_type]}\nWindow N: {w_size}\nAlpha: {alpha:.2f}")
        except Exception as e:
            messagebox.showerror("Filter Config Error", f"Invalid parameters: {e}")

    def send_cmd(self, cmd_str):
        if self.is_connected and self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(f"{cmd_str}\n".encode('utf-8'))
                self.log(f"[TX] -> {cmd_str}")
            except Exception as e:
                self.log(f"[ERROR] Failed to send: {e}")

    def set_continuous(self):
        self.current_mode = "CONTINUOUS"
        self.send_cmd("CONTINUOUS")
        self._update_active_button(self.btn_continuous, self.ACCENT_GREEN)

    def set_intermittent(self):
        self.current_mode = "INTERMITTENT"
        self.send_cmd("INTERMITTENT")
        self._update_active_button(self.btn_intermittent, self.ACCENT_YELLOW)

    def set_off(self):
        self.current_mode = "OFF"
        self.send_cmd("OFF")
        self._update_active_button(self.btn_off)

    def _update_active_button(self, active_btn, active_color="#585b70"):
        for btn in [self.btn_continuous, self.btn_intermittent, self.btn_off]:
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
