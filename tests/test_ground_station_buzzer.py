import os
import sys
import subprocess
import pytest

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
GROUND_STATION_DIR = os.path.join(PROJECT_ROOT, "Code", "GROUND-STATION")
sys.path.insert(0, GROUND_STATION_DIR)

from telemetry_codec import encode_telemetry, decode_telemetry


class MockGroundStationBuzzer:
    """Simulates the Ground Station ESP32 C++ firmware buzzer and mode detection logic."""

    def __init__(self, mode_beep_duration_ms=700):
        self.mode_beep_duration_ms = mode_beep_duration_ms
        self.buzzer_state = False
        self.beep_until = 0
        self.intermittent_beep = False
        self.has_initial_mode = False
        self.last_flight_mode = 0
        self.last_roll_active = False
        self.beep_trigger_count = 0

    def trigger_buzzer(self, duration_ms=None, now_ms=0):
        if duration_ms is None:
            duration_ms = self.mode_beep_duration_ms
        self.buzzer_state = True
        self.beep_until = now_ms + duration_ms
        self.beep_trigger_count += 1

    def process_telemetry_packet(self, packet_bytes, now_ms):
        """Processes 61-byte binary packet matching main.cpp logic."""
        decoded = decode_telemetry(packet_bytes)
        if decoded is None:
            return

        rc_lost = decoded.get("rcSignalLost", False)
        if rc_lost:
            return

        mode = decoded.get("flight_mode", 1)
        roll_active = decoded.get("roll_active", False)

        if not self.has_initial_mode:
            self.last_flight_mode = mode
            self.last_roll_active = roll_active
            self.has_initial_mode = True
        elif mode != self.last_flight_mode or roll_active != self.last_roll_active:
            self.trigger_buzzer(self.mode_beep_duration_ms, now_ms=now_ms)
            self.last_flight_mode = mode
            self.last_roll_active = roll_active

    def process_serial_command(self, cmd_str, now_ms):
        """Processes serial commands matching main.cpp serial parser."""
        cmd = cmd_str.strip()
        if cmd in ("BEEP:SHORT", "SHORT", "BEEP:0.7S", "BEEP:1S", "BEEP:MODE"):
            self.trigger_buzzer(self.mode_beep_duration_ms, now_ms=now_ms)
        elif cmd in ("BEEP:CONTINUOUS", "CONTINUOUS"):
            self.intermittent_beep = False
            self.beep_until = 0
            self.buzzer_state = True
        elif cmd in ("BEEP:INTERMITTENT", "INTERMITTENT"):
            self.beep_until = 0
            self.intermittent_beep = True
        elif cmd in ("BEEP:OFF", "OFF"):
            self.intermittent_beep = False
            self.beep_until = 0
            self.buzzer_state = False

    def update_loop(self, now_ms):
        """Simulates firmware loop() tick."""
        if self.beep_until > 0:
            if now_ms >= self.beep_until:
                self.beep_until = 0
                if not self.intermittent_beep:
                    self.buzzer_state = False


def make_packet(flight_mode=1, roll_assist=False, rc_signal_lost=False):
    return encode_telemetry(
        pitch=0.0, roll=0.0,
        accel_x=0, accel_y=0, accel_z=16384,
        gyro_x=0, gyro_y=0, gyro_z=0,
        rc1=1500, rc2=1500, rc3=1000, rc5=1500,
        flight_mode=flight_mode,
        is_assist_mode=roll_assist,
        rc_signal_lost=rc_signal_lost
    )


class TestGroundStationBuzzer:

    def test_firmware_compilation(self):
        """Verifies Ground Station ESP32 firmware compiles cleanly with PlatformIO."""
        cmd = [sys.executable, "-m", "platformio", "run"]
        result = subprocess.run(cmd, cwd=GROUND_STATION_DIR, capture_output=True, text=True, timeout=300)
        assert result.returncode == 0, f"Ground Station build failed:\n{result.stderr}\n{result.stdout}"

    def test_initial_packet_does_not_trigger_beep(self):
        """Initial packet received on boot sets baseline mode without triggering a beep."""
        gs = MockGroundStationBuzzer()
        pkt = make_packet(flight_mode=1, roll_assist=False)
        gs.process_telemetry_packet(pkt, now_ms=100)

        assert gs.has_initial_mode is True
        assert gs.last_flight_mode == 1
        assert gs.last_roll_active is False
        assert gs.buzzer_state is False
        assert gs.beep_trigger_count == 0

    def test_mode_transition_triggers_one_second_beep(self):
        """Switching flight mode (e.g. Mode 1 -> Mode 2) triggers a 0.7-second beep."""
        gs = MockGroundStationBuzzer(mode_beep_duration_ms=700)
        # Boot in Mode 1
        pkt_m1 = make_packet(flight_mode=1, roll_assist=False)
        gs.process_telemetry_packet(pkt_m1, now_ms=100)
        assert gs.buzzer_state is False

        # Switch to Mode 2 at t=500ms
        pkt_m2 = make_packet(flight_mode=2, roll_assist=False)
        gs.process_telemetry_packet(pkt_m2, now_ms=500)
        assert gs.buzzer_state is True
        assert gs.beep_until == 1200
        assert gs.beep_trigger_count == 1

        # Intermediate ticks at 20 Hz (e.g. t=600ms, t=800ms) with same mode
        for t in range(550, 1150, 50):
            gs.process_telemetry_packet(pkt_m2, now_ms=t)
            gs.update_loop(now_ms=t)
            assert gs.buzzer_state is True
            assert gs.beep_trigger_count == 1  # No repeated triggers

        # At t=1200ms, beep duration expires and buzzer turns OFF
        gs.update_loop(now_ms=1200)
        assert gs.buzzer_state is False
        assert gs.beep_until == 0

    def test_roll_assist_toggle_triggers_beep(self):
        """Toggling SWB (Roll Assist ON/OFF) triggers feedback beep."""
        gs = MockGroundStationBuzzer(mode_beep_duration_ms=700)
        # Mode 1, Roll OFF
        pkt_off = make_packet(flight_mode=1, roll_assist=False)
        gs.process_telemetry_packet(pkt_off, now_ms=100)

        # Mode 1, Roll ON
        pkt_on = make_packet(flight_mode=1, roll_assist=True)
        gs.process_telemetry_packet(pkt_on, now_ms=200)

        assert gs.buzzer_state is True
        assert gs.last_roll_active is True
        assert gs.beep_trigger_count == 1

    def test_rc_loss_does_not_trigger_mode_beep(self):
        """When RC signal is lost (failsafe), mode beep is not falsely triggered."""
        gs = MockGroundStationBuzzer()
        pkt_normal = make_packet(flight_mode=1, roll_assist=False, rc_signal_lost=False)
        gs.process_telemetry_packet(pkt_normal, now_ms=100)

        # Signal lost flag set
        pkt_lost = make_packet(flight_mode=2, roll_assist=False, rc_signal_lost=True)
        gs.process_telemetry_packet(pkt_lost, now_ms=200)

        assert gs.buzzer_state is False
        assert gs.beep_trigger_count == 0
        assert gs.last_flight_mode == 1  # Frozen

    def test_serial_commands_control(self):
        """Serial commands BEEP:SHORT, BEEP:1S, BEEP:OFF correctly manage buzzer."""
        gs = MockGroundStationBuzzer(mode_beep_duration_ms=700)

        gs.process_serial_command("BEEP:SHORT", now_ms=100)
        assert gs.buzzer_state is True
        assert gs.beep_until == 800

        gs.process_serial_command("BEEP:OFF", now_ms=200)
        assert gs.buzzer_state is False
        assert gs.beep_until == 0

        gs.process_serial_command("BEEP:0.7S", now_ms=300)
        assert gs.buzzer_state is True
        assert gs.beep_until == 1000
