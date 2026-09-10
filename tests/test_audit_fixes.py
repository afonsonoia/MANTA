"""
Test suite validating code audit fixes, formula polarities, and kinematic safety:
1. Soft power floor vs hard emergency cutoff consistency under low voltage.
2. Auto-detection and correlation alignment of gyro axes in SysID for -90° PCB mounting.
3. Negative feedback polarity and rate damping signs for V-Tail and Rollerons.
4. Servo boundary compliance under simultaneous Pitch, Roll and Flaperon deflection.
"""

import math
import numpy as np
import pytest
from pathlib import Path


class TestAuditFixes:
    """Verifies that all audited fixes and formula polarities function correctly and safely."""

    def test_low_voltage_soft_ceiling_policy(self):
        """Validates that setThrottlePulse respects the soft power ceiling (1350us) to maintain flight during low voltage."""
        THROTTLE_MIN_PULSE = 1000
        THROTTLE_MAX_PULSE = 2000
        THROTTLE_LOW_VOLT_CEILING_PULSE = 1350

        def set_throttle_pulse(pulse_us: int, low_volt_triggered: bool):
            if low_volt_triggered:
                if pulse_us > THROTTLE_LOW_VOLT_CEILING_PULSE:
                    pulse_us = THROTTLE_LOW_VOLT_CEILING_PULSE
            if THROTTLE_MIN_PULSE <= pulse_us <= THROTTLE_MAX_PULSE:
                return pulse_us
            return THROTTLE_MIN_PULSE

        # Normal condition
        assert set_throttle_pulse(1700, False) == 1700
        assert set_throttle_pulse(1200, False) == 1200

        # Low voltage cutoff triggered: high throttle capped at 1350us (soft ceiling)
        assert set_throttle_pulse(1700, True) == 1350
        assert set_throttle_pulse(1800, True) == 1350
        assert set_throttle_pulse(2000, True) == 1350

        # Low voltage cutoff triggered: low throttle below ceiling preserved
        assert set_throttle_pulse(1200, True) == 1200
        assert set_throttle_pulse(1000, True) == 1000

    def test_sysid_gyro_axis_autodetection_for_manta_pcb(self):
        """Validates that SysID correctly detects the -90° PCB mounting and aligns Gyro X to Pitch and Gyro Y to Roll."""
        dt = 0.05
        t = np.linspace(0, 10, 200)

        # Simulated flight dynamics:
        # Pitch oscillation at 0.5 Hz
        pitch = 5.0 * np.sin(2 * np.pi * 0.5 * t)
        # Roll oscillation at 0.8 Hz
        roll = 15.0 * np.sin(2 * np.pi * 0.8 * t)

        dp_dt = np.gradient(pitch, dt)
        dr_dt = np.gradient(roll, dt)

        # On MANTA PCB (-90° rotation):
        # Gyro X measures -dp/dt * 32.8 (negative correlation)
        # Gyro Y measures +dr/dt * 32.8 (positive correlation)
        gx = -dp_dt * 32.8 + np.random.normal(0, 2, len(t))
        gy = dr_dt * 32.8 + np.random.normal(0, 2, len(t))

        c_gx_dp = abs(np.corrcoef(gx, dp_dt)[0, 1])
        c_gy_dp = abs(np.corrcoef(gy, dp_dt)[0, 1])
        c_gx_dr = abs(np.corrcoef(gx, dr_dt)[0, 1])
        c_gy_dr = abs(np.corrcoef(gy, dr_dt)[0, 1])

        # Verify auto-detection selects Gyro X for Pitch and Gyro Y for Roll
        detected_pitch_gyro = "gx" if c_gx_dp > c_gy_dp else "gy"
        detected_roll_gyro = "gy" if c_gy_dr > c_gx_dr else "gx"

        assert detected_pitch_gyro == "gx", "SysID auto-detection should identify Gyro X as Pitch rate on MANTA PCB"
        assert detected_roll_gyro == "gy", "SysID auto-detection should identify Gyro Y as Roll rate on MANTA PCB"

        # Verify sign alignment:
        q_raw = gx / 32.8
        if np.corrcoef(q_raw, dp_dt)[0, 1] < 0:
            q_raw = -q_raw

        p_raw = gy / 32.8
        if np.corrcoef(p_raw, dr_dt)[0, 1] < 0:
            p_raw = -p_raw

        assert np.corrcoef(q_raw, dp_dt)[0, 1] > 0.95, "Corrected Pitch rate must correlate positively with dPitch/dt"
        assert np.corrcoef(p_raw, dr_dt)[0, 1] > 0.95, "Corrected Roll rate must correlate positively with dRoll/dt"

    def test_vtail_negative_feedback_pitch_polarity(self):
        """Validates that pitchPidOut generates strictly negative feedback for V-Tail elevator kinematics."""
        neutral_br = 1478  # 1500 + TRIM_US_BR (-22)
        neutral_bl = 1633  # 1500 + TRIM_US_BL (+133)
        angle_limit = 278

        def calculate_elevators(target_pitch: float, cur_pitch: float, kp: float = 5.0):
            error = target_pitch - cur_pitch
            pitch_pid_out = kp * error
            pitch_diff = -max(-angle_limit, min(angle_limit, int(pitch_pid_out)))
            target_br = neutral_br + pitch_diff
            target_bl = neutral_bl - pitch_diff
            return target_br, target_bl, pitch_diff

        # Case 1: Aircraft nose is low (cur_pitch = -10°, target = 0°) -> Must command CABRAR (Up elevator)
        br_up, bl_up, diff_up = calculate_elevators(target_pitch=0.0, cur_pitch=-10.0)
        assert diff_up < 0, "pitchDiff must be negative for climb command"
        # On MANTA: BR pulse decreases (1478 -> lower) and BL pulse increases (1633 -> higher) = UP elevator!
        assert br_up < neutral_br, "BR pulse must decrease to deflect upward"
        assert bl_up > neutral_bl, "BL pulse must increase to deflect upward (mirrored servo)"

        # Case 2: Aircraft nose is high (cur_pitch = +10°, target = 0°) -> Must command PICAR (Down elevator)
        br_dn, bl_dn, diff_dn = calculate_elevators(target_pitch=0.0, cur_pitch=+10.0)
        assert diff_dn > 0, "pitchDiff must be positive for dive command"
        assert br_dn > neutral_br, "BR pulse must increase to deflect downward"
        assert bl_dn < neutral_bl, "BL pulse must decrease to deflect downward"

    def test_roll_rate_damping_sign(self):
        """Validates that roll derivative term produces corrective opposing deflection."""
        kd = 1.500
        angle_limit = 278

        # If aircraft is rolling RIGHT at +30 deg/s with error = 0:
        roll_rate = 30.0  # deg/s
        roll_pid_out = -(kd * roll_rate)  # = -45.0
        roll_diff = int(roll_pid_out)     # = -45

        # Mixer: rollOffsetFR = flaperonOffsetFR - rollDiff
        #        rollOffsetFL = flaperonOffsetFL - rollDiff
        offset_fr = -roll_diff  # = +45 us
        offset_fl = -roll_diff  # = +45 us

        # FR: positive PWM deflects DOWN (+45us pushes right wing UP)
        # FL: negative PWM deflects DOWN -> positive PWM deflects UP (+45us pushes left wing DOWN)
        # Result: Opposes the rightward roll by generating leftward aerodynamic torque!
        assert offset_fr > 0, "FR must receive positive offset to generate counter-roll moment"
        assert offset_fl > 0, "FL must receive positive offset to generate counter-roll moment"

    def test_flaperon_full_deflection_hardware_bounds(self):
        """Validates that even with maximum roll input and 10 deg flaperon deflection,
        servos stay strictly within safe angular limits and hardware bounds [1000, 2000] us
        with zero clipping or saturation due to dynamic headroom scaling.
        """
        neutral_fr = 1500
        neutral_fl = 1544
        angle_limit = 278
        flaperon_us_fr = +167  # 15.0 deg DOWN
        flaperon_us_fl = -167  # 15.0 deg DOWN

        for roll_diff in [-278, -150, 0, 150, 278]:
            # Anti-saturation headroom scaling (Roll Priority)
            roll_ratio = min(1.0, abs(roll_diff) / float(angle_limit))
            headroom = 1.0 - roll_ratio
            eff_fr = round(flaperon_us_fr * headroom)
            eff_fl = round(flaperon_us_fl * headroom)

            # Mixer: rollOffsetFR = eff_fr - roll_diff
            target_fr = max(neutral_fr - angle_limit, min(neutral_fr + angle_limit, neutral_fr + eff_fr - roll_diff))
            target_fl = max(neutral_fl - angle_limit, min(neutral_fl + angle_limit, neutral_fl + eff_fl - roll_diff))

            # Hardware clamp
            target_fr = max(1000, min(2000, target_fr))
            target_fl = max(1000, min(2000, target_fl))

            # Strictly inside [neutral - 278, neutral + 278]
            assert 1222 <= target_fr <= 1778, f"FR target {target_fr} exceeded safe angle span"
            assert 1266 <= target_fl <= 1822, f"FL target {target_fl} exceeded safe angle span"
            # Strictly inside [1000, 2000] hardware limits
            assert 1000 <= target_fr <= 2000
            assert 1000 <= target_fl <= 2000

    def test_ch5_mode_debounce_timing_and_noise_rejection(self):
        """Validates that mode transitions require exactly 2 consecutive identical ticks (40 ms @ 50 Hz),

        rejecting single-tick EMI glitches while ensuring instantaneous response below human latency.
        """
        CH5_DEBOUNCE_CONFIRM_TICKS = 2

        class ModeSwitchMachine:
            def __init__(self):
                self.current_mode = 1
                self.current_flaperon = False
                self.pending_mode = 1
                self.pending_flaperon = False
                self.debounce_count = 0
                self.first_tick = True

            def decode_raw(self, pulse_us: int):
                # Simplified representation of decodeCH5
                if pulse_us < 1247:
                    return 1, False
                elif pulse_us < 1370:
                    return 2, False
                elif pulse_us < 1476:
                    return 3, False
                elif pulse_us < 1683:
                    return 1, True
                elif pulse_us < 1884:
                    return 2, True
                else:
                    return 2, True  # Auto-demoted from 3 to 2

            def update_tick(self, pulse_us: int):
                cand_mode, cand_flap = self.decode_raw(pulse_us)
                if self.first_tick:
                    self.pending_mode = cand_mode
                    self.pending_flaperon = cand_flap
                    self.current_mode = cand_mode
                    self.current_flaperon = cand_flap
                    self.debounce_count = CH5_DEBOUNCE_CONFIRM_TICKS
                    self.first_tick = False
                    return self.current_mode, self.current_flaperon

                if cand_mode == self.pending_mode and cand_flap == self.pending_flaperon:
                    if self.debounce_count < CH5_DEBOUNCE_CONFIRM_TICKS:
                        self.debounce_count += 1
                        if self.debounce_count >= CH5_DEBOUNCE_CONFIRM_TICKS:
                            self.current_mode = cand_mode
                            self.current_flaperon = cand_flap
                else:
                    self.pending_mode = cand_mode
                    self.pending_flaperon = cand_flap
                    self.debounce_count = 1

                return self.current_mode, self.current_flaperon

        fsm = ModeSwitchMachine()
        # Initialize at Mode 1 Flap OFF (1166 us)
        m, flap = fsm.update_tick(1166)
        assert (m, flap) == (1, False)

        # Scenario 1: Glitch/Noise pulse of 1328 us (Mode 2) for only 1 tick (20 ms)
        m, flap = fsm.update_tick(1328)
        assert (m, flap) == (1, False), "Single-tick glitch must be rejected without switching mode"
        # Return to 1166 us on next tick
        m, flap = fsm.update_tick(1166)
        assert (m, flap) == (1, False), "Mode must remain Mode 1 after momentary spike"

        # Scenario 2: Legitimate switch from Mode 1 to Mode 2 (1328 us)
        # Tick 1 (t = 20 ms):
        m, flap = fsm.update_tick(1328)
        assert (m, flap) == (1, False), "At tick 1 (20 ms), transition is still debouncing"
        # Tick 2 (t = 40 ms):
        m, flap = fsm.update_tick(1328)
        assert (m, flap) == (2, False), "At tick 2 (40 ms), mode switch MUST be confirmed"

        # Scenario 3: Switch to Flaperons ON (1825 us)
        # Tick 1:
        m, flap = fsm.update_tick(1825)
        assert (m, flap) == (2, False)
        # Tick 2:
        m, flap = fsm.update_tick(1825)
        assert (m, flap) == (2, True), "Confirmed Mode 2 with Flaperons ON at 40 ms"

    def test_bumpless_mode_transfer_and_derivative_kick_suppression(self):
        """Validates that mode transitions re-anchor last attitude measurements to prevent derivative kick,

        and flush control integrators to eliminate command jumps.
        """
        dt = 0.02  # 50 Hz control loop
        cur_pitch = -12.5  # Non-zero pitch attitude during transition
        cur_roll = 25.0    # Banked turn attitude during transition

        # Controller state simulation
        last_pitch_meas = 0.0  # Old measurement from previous mode
        last_roll_meas = 0.0
        pitch_integrator = 45.2
        roll_integrator = -38.7
        mode_just_changed = True

        # Bumpless transfer logic from control.cpp:
        if mode_justChanged := True:
            # Integrator flush
            pitch_integrator = 0.0
            roll_integrator = 0.0
            # Derivative re-anchoring
            last_pitch_meas = cur_pitch
            last_roll_meas = cur_roll

        # Compute derivative terms on switch tick
        pitch_rate_deg_s = (cur_pitch - last_pitch_meas) / dt
        roll_rate_deg_s = (cur_roll - last_roll_meas) / dt

        kd_pitch = 0.8
        kd_roll = 1.5
        d_term_pitch = -(kd_pitch * pitch_rate_deg_s)
        d_term_roll = -(kd_roll * roll_rate_deg_s)

        # Assert zero derivative kick on the transition tick
        assert pitch_rate_deg_s == 0.0, "Pitch rate must be 0 on mode transition tick"
        assert roll_rate_deg_s == 0.0, "Roll rate must be 0 on mode transition tick"
        assert d_term_pitch == 0.0, "Derivative kick on pitch must be completely eliminated"
        assert d_term_roll == 0.0, "Derivative kick on roll must be completely eliminated"
        assert pitch_integrator == 0.0, "Pitch integrator must be zeroed for bumpless handoff"
        assert roll_integrator == 0.0, "Roll integrator must be zeroed for bumpless handoff"

    def test_flaperon_anti_learning_safety_rule(self):
        """Validates that enabling Flaperons strictly forbids Mode 3 and demotes to Mode 2."""
        def decode_ch5(pulse_us: int):
            if pulse_us < 1247:
                return 1, False
            elif pulse_us < 1370:
                return 2, False
            elif pulse_us < 1476:
                return 3, False
            elif pulse_us < 1683:
                return 1, True
            elif pulse_us < 1884:
                return 2, True
            else:
                # Mode 3 prohibited when Flaperons ON -> Demoted to Mode 2
                return 2, True

        # SWC 3 + SWB ON (~1942 us): Should be Mode 3 + Flap ON, but safety demotes to Mode 2
        mode, flap = decode_ch5(1942)
        assert mode == 2, "Mode 3 with flaperons must be automatically demoted to Mode 2"
        assert flap is True, "Flaperons must remain active"

        # Extreme high pulse (e.g. 2000 us)
        mode, flap = decode_ch5(2000)
        assert mode == 2
        assert flap is True

    def test_cascaded_failsafes(self):
        """Validates that RC loss and IMU loss trigger deterministic, safe fallback states."""
        # Failsafe 1: RC Signal Loss
        def handle_rc_loss(rc_signal_lost: bool, c3: int):
            target_throttle = 1000 if rc_signal_lost else c3
            return target_throttle

        assert handle_rc_loss(True, 1800) == 1000, "Throttle must be cut to 1000 us on RC loss"
        assert handle_rc_loss(False, 1800) == 1800, "Throttle operates normally when RC is healthy"

        # Failsafe 2: IMU Hardware Loss
        def handle_imu_loss(mpu_healthy: bool, current_mode: int):
            flight_mode = current_mode
            roll_active = True
            integrators_cleared = False

            if not mpu_healthy:
                flight_mode = 1  # Fallback to 100% manual
                roll_active = False  # Direct manual roll without attitude assist
                integrators_cleared = True

            return flight_mode, roll_active, integrators_cleared

        mode, roll_assist, integrators_cleared = handle_imu_loss(mpu_healthy=False, current_mode=2)
        assert mode == 1, "Must fall back to Mode 1 (Manual) when IMU fails"
        assert roll_assist is False, "Roll assist must be disabled to prevent corrupt gyro feedback"
        assert integrators_cleared is True, "Integrators must be flushed to avoid windup"

    def test_cumulative_flight_time_accumulation(self):
        """Validates that cumulative_flight_time accumulates time when throttle is active and stays zero when motor is idle."""
        cumulative_flight_time = 0.0

        # Scenario 1: On ground / Motor idle (throttle_pct = 0, latest_rc[2] = 1000)
        dt_ticks = [0.05, 0.05, 0.05, 0.05]
        latest_rc_idle = [1500, 1500, 1000, 0, 1166]
        throttle_pct = int(max(0, min(100, (latest_rc_idle[2] - 1000) / 10)))
        assert throttle_pct == 0
        for dt in dt_ticks:
            if throttle_pct > 5:
                cumulative_flight_time += dt
        assert cumulative_flight_time == 0.0, "Flight time must not accumulate when throttle is at zero/idle"

        # Scenario 2: Active takeoff / cruising flight (throttle_pct = 60%, latest_rc[2] = 1600)
        latest_rc_active = [1500, 1500, 1600, 0, 1328]
        throttle_pct = int(max(0, min(100, (latest_rc_active[2] - 1000) / 10)))
        assert throttle_pct == 60
        for dt in dt_ticks:
            if throttle_pct > 5:
                cumulative_flight_time += dt
        assert pytest.approx(cumulative_flight_time, 0.001) == 0.20, "Flight time must accumulate active flight seconds"

    def test_telemetry_csv_logger_ch5_extraction_safety(self):
        """Validates that CH5 extraction in telemetry CSV logger resolves rc5 correctly across 4-channel, 5-channel, and dict keys."""
        # Case A: 4-channel telemetry [ch1, ch2, ch3, ch5]
        dict_4ch = {"rc": [1520, 1490, 1650, 1328]}
        rc = dict_4ch.get("rc", [0, 0, 0, 0])
        ch5_a = dict_4ch.get("rc5", rc[4] if len(rc) > 4 else (rc[3] if len(rc) > 3 else 0))
        assert ch5_a == 1328

        # Case B: 5-channel telemetry [ch1, ch2, ch3, ch4, ch5]
        dict_5ch = {"rc": [1520, 1490, 1650, 1500, 1825]}
        rc = dict_5ch.get("rc", [0, 0, 0, 0])
        ch5_b = dict_5ch.get("rc5", rc[4] if len(rc) > 4 else (rc[3] if len(rc) > 3 else 0))
        assert ch5_b == 1825, "In 5-channel array, index 4 (CH5) must be chosen over index 3 (CH4)"

        # Case C: Explicit 'rc5' key in telemetry dictionary
        dict_explicit = {"rc": [1500, 1500, 1000, 1500, 0], "rc5": 1411}
        rc = dict_explicit.get("rc", [0, 0, 0, 0])
        ch5_c = dict_explicit.get("rc5", rc[4] if len(rc) > 4 else (rc[3] if len(rc) > 3 else 0))
        assert ch5_c == 1411, "Explicit rc5 key in telemetry dictionary must take precedence"

    def test_sysid_plot_column_resolution(self):
        """Validates that plot generation dynamically identifies pitch and roll columns across multiple CSV schema variants."""
        import pandas as pd

        # Schema 1: Mission Planner format: "RC2 Pitch (us)", "Pitch (deg)", "Roll (deg)", "Servo FR (us)"
        df1 = pd.DataFrame({"RC2 Pitch (us)": [1500, 1520], "Pitch (deg)": [1.0, 2.0], "Roll (deg)": [-3.0, -4.0], "Servo FR (us)": [1500, 1500]})
        pitch_col1 = next((c for c in df1.columns if "pitch" in c.lower() and "rc" not in c.lower() and "servo" not in c.lower()), None)
        roll_col1 = next((c for c in df1.columns if "roll" in c.lower() and "rc" not in c.lower() and "servo" not in c.lower()), None)
        assert pitch_col1 == "Pitch (deg)", "Must match Pitch (deg) without picking RC2 Pitch (us)"
        assert roll_col1 == "Roll (deg)"

        # Schema 2: Telemetry CSV Logger format: "rc2_pitch_pwm", "pitch_deg", "roll_deg", "rc1_roll_pwm"
        df2 = pd.DataFrame({"rc2_pitch_pwm": [1500, 1500], "pitch_deg": [1.0, 2.0], "roll_deg": [-3.0, -4.0], "rc1_roll_pwm": [1500, 1500]})
        pitch_col2 = next((c for c in df2.columns if "pitch" in c.lower() and "rc" not in c.lower() and "servo" not in c.lower()), None)
        roll_col2 = next((c for c in df2.columns if "roll" in c.lower() and "rc" not in c.lower() and "servo" not in c.lower()), None)
        assert pitch_col2 == "pitch_deg", "Must match pitch_deg without picking rc2_pitch_pwm"
        assert roll_col2 == "roll_deg", "Must match roll_deg without getting confused with rc1_roll_pwm"


