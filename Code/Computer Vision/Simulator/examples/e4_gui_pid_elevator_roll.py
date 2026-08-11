import sys
import time
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QDoubleSpinBox, QPushButton, QGroupBox, QSlider)
from PyQt6.QtCore import QThread, pyqtSignal, Qt
from pymavlink import mavutil
from aux_code.general.basic_aux_code import arm_plane, get_all_telemetry, set_controls
from aux_code.control.pid import PID, AdaptivePID


# ==========================================
# CUSTOM WIDGET: SLIDER WITH INPUT BELOW
# ==========================================
class TargetControlWidget(QWidget):
    def __init__(self, label_name, min_val, max_val, default_val):
        super().__init__()
        layout = QVBoxLayout()
        layout.setContentsMargins(10, 10, 10, 10)

        self.label = QLabel(f"<b>{label_name}</b>")

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(int(min_val * 100), int(max_val * 100))
        self.slider.setValue(int(default_val * 100))

        self.spinbox = QDoubleSpinBox()
        self.spinbox.setRange(min_val, max_val)
        self.spinbox.setDecimals(2)
        self.spinbox.setValue(default_val)
        self.spinbox.setSingleStep(1.0)  # Step of 1.0 to be faster
        self.spinbox.setFixedWidth(100)

        self.slider.valueChanged.connect(self._slider_changed)
        self.spinbox.valueChanged.connect(self._spinbox_changed)

        layout.addWidget(self.label)
        layout.addWidget(self.slider)

        h_box = QHBoxLayout()
        h_box.addStretch()
        h_box.addWidget(self.spinbox)
        h_box.addStretch()
        layout.addLayout(h_box)

        self.setLayout(layout)

    def _slider_changed(self, value):
        self.spinbox.blockSignals(True)
        self.spinbox.setValue(value / 100.0)
        self.spinbox.blockSignals(False)

    def _spinbox_changed(self, value):
        self.slider.blockSignals(True)
        self.slider.setValue(int(value * 100))
        self.slider.blockSignals(False)

    def get_value(self):
        return self.spinbox.value()


# ==========================================
# 1. THE WORKER THREAD (The "Brain")
# ==========================================
class FlightControlThread(QThread):
    telemetry_signal = pyqtSignal(dict)
    log_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.running = True
        self.connection = None

        # Initial Targets
        self.target_pitch = 10.0
        self.target_roll = 0.0
        self.target_throttle_pwm = 1700  # 70% by default

        # PIDs
        self.elevator_pid = AdaptivePID(kp=4.5, ki=0.8, kd=0.1, setpoint=0, output_limits=(-500, 500), ki_limit=100)
        self.roll_pid = AdaptivePID(kp=0.2, ki=0.7, kd=1, setpoint=0, output_limits=(-500, 500), ki_limit=100)

    def update_targets(self, new_pitch, new_roll, new_throttle_pct):
        self.target_pitch = new_pitch
        self.target_roll = new_roll

        # Converts the percentage (0-100) to PWM (1000-2000)
        self.target_throttle_pwm = int(1000 + (new_throttle_pct * 10))

        self.log_signal.emit(f"TARGETS: Pitch {new_pitch}° | Roll {new_roll}° | Throttle {new_throttle_pct}% ({self.target_throttle_pwm} PWM)")

    def run(self):
        self.log_signal.emit("Waiting for SITL communication...")
        self.connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
        self.connection.wait_heartbeat()
        self.log_signal.emit("Connected to Simulator!")

        arm_plane(self.connection)
        self.log_signal.emit("Ready to fly!")
        time.sleep(2)

        while self.running:
            all_data = get_all_telemetry(self.connection)

            if all_data:
                roll, pitch, altitude = all_data["roll"], all_data["pitch"], all_data["alt"]

                # =======================================================
                # CONTROL LOGIC (Direct Attitude)
                # =======================================================
                self.elevator_pid.soft_set_target(self.target_pitch, iteration=0.2)
                self.roll_pid.soft_set_target(self.target_roll, iteration=2)

                # Calculate servo PWMs
                pitch_pwm = round(1500 + self.elevator_pid.update(pitch))
                roll_pwm = round(1500 + self.roll_pid.update(roll))

                # Send commands (including motor PWM)
                set_controls(self.connection, roll_pwm, pitch_pwm, self.target_throttle_pwm)

                # Update GUI
                self.telemetry_signal.emit({
                    "pitch": pitch, "roll": roll, "alt": altitude,
                    "target_pitch": self.elevator_pid.setpoint,
                    "target_roll": self.roll_pid.setpoint,
                    "throttle_pwm": self.target_throttle_pwm
                })

            time.sleep(0.05)  # 20Hz

    def stop(self):
        self.running = False
        self.wait()


# ==========================================
# 2. THE MAIN GUI (The "Face")
# ==========================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ZOHD Rebel - Attitude & Engine Controller")
        self.resize(500, 550)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        self.layout = QVBoxLayout(central_widget)

        # --- TELEMETRY ---
        self.lbl_status = QLabel("Status: Disconnected")
        self.lbl_status.setStyleSheet("color: blue; font-weight: bold;")
        self.lbl_attitude = QLabel("Pitch: 0.0° | Roll: 0.0° | Alt: 0.0m | Motor: 1000 PWM")
        self.layout.addWidget(self.lbl_status)
        self.layout.addWidget(self.lbl_attitude)

        # --- TARGET CONTROLS ---
        group_targets = QGroupBox("Flight Targets (Buffer)")
        v_targets = QVBoxLayout()

        self.throttle_target_widget = TargetControlWidget("Target Throttle (%)", 0.0, 100.0, 70.0)
        self.pitch_target_widget = TargetControlWidget("Target Pitch (°)", -45.0, 45.0, 10.0)
        self.roll_target_widget = TargetControlWidget("Target Roll (°)", -45.0, 45.0, 0.0)

        # Adds the widgets in order of flight importance
        v_targets.addWidget(self.throttle_target_widget)
        v_targets.addWidget(self.pitch_target_widget)
        v_targets.addWidget(self.roll_target_widget)
        group_targets.setLayout(v_targets)
        self.layout.addWidget(group_targets)

        # --- UPLOAD BUTTON ---
        self.btn_update = QPushButton("UPLOAD TARGETS TO VEHICLE")
        self.btn_update.setStyleSheet("background-color: darkorange; color: white; font-weight: bold; font-size: 14px; padding: 15px;")
        self.btn_update.clicked.connect(self.send_targets_to_plane)
        self.layout.addWidget(self.btn_update)

        # --- START BUTTON ---
        self.btn_start = QPushButton("CONNECT & ARM PLANE")
        self.btn_start.setStyleSheet("background-color: darkgreen; color: white; font-weight: bold; padding: 10px;")
        self.btn_start.clicked.connect(self.start_flight_loop)
        self.layout.addWidget(self.btn_start)

        self.flight_thread = FlightControlThread()
        self.flight_thread.telemetry_signal.connect(self.update_telemetry_ui)
        self.flight_thread.log_signal.connect(self.update_status_ui)

    def send_targets_to_plane(self):
        new_throttle = self.throttle_target_widget.get_value()
        new_pitch = self.pitch_target_widget.get_value()
        new_roll = self.roll_target_widget.get_value()

        self.flight_thread.update_targets(new_pitch, new_roll, new_throttle)

        self.btn_update.setStyleSheet("background-color: green; color: white; font-weight: bold; font-size: 14px; padding: 15px;")
        QApplication.processEvents()
        time.sleep(0.15)
        self.btn_update.setStyleSheet("background-color: darkorange; color: white; font-weight: bold; font-size: 14px; padding: 15px;")

    def start_flight_loop(self):
        self.btn_start.setEnabled(False)
        self.btn_start.setText("FLYING...")
        self.send_targets_to_plane()
        self.flight_thread.start()

    def update_telemetry_ui(self, data):
        self.lbl_attitude.setText(f"Pitch: {data['pitch']:.1f}° [T:{data['target_pitch']:.1f}] | "
                                  f"Roll: {data['roll']:.1f}° [T:{data['target_roll']:.1f}] | "
                                  f"Alt: {data['alt']:.1f}m | "
                                  f"Motor: {data['throttle_pwm']} PWM")

    def update_status_ui(self, message):
        self.lbl_status.setText(f"Status: {message}")

    def closeEvent(self, event):
        self.flight_thread.stop()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    sys.exit(app.exec())