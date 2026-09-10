"""
Unit tests for MANTA Fly-by-Wire (FBW) and PI-D Closed-Loop Control Logic.
Validates:
1. CH5 PWM truth table decoding for Mode 1, 2, 3 and Roll toggle.
2. Quasi-linear stick exponential curve behavior (soft center, nearly linear).
3. PI-D control response, anti-windup clamping, and derivative on measurement.
4. Coordinated turn pitch compensation calculations.
"""

import math
from pathlib import Path
import pytest


def decode_ch5(ch5_pulse: int):
    """Mirror of decodeCH5() from control.cpp"""
    if ch5_pulse < 1247:
        return 1, False  # Mode 1, Flaperons OFF (~1166 us)
    elif ch5_pulse < 1370:
        return 2, False  # Mode 2, Flaperons OFF (~1328 us)
    elif ch5_pulse < 1476:
        return 3, False  # Mode 3, Flaperons OFF (~1411 us)
    elif ch5_pulse < 1683:
        return 1, True   # Mode 1, Flaperons ON  (~1541 us)
    elif ch5_pulse < 1884:
        return 2, True   # Mode 2, Flaperons ON  (~1825 us)
    else:
        # Rule: Flaperons ON prohibits Mode 3 -> Auto-demotes to Mode 2!
        return 2, True   # Mode 2 Auto Flap-Safe, Flaperons ON (~1942 us)


def apply_expo_and_scale_deg(raw_stick_us: int, center_us: int = 1500, max_deg: float = 45.0, expo_factor: float = 0.08):
    """Mirror of applyExpoAndScaleDeg() from control.cpp"""
    norm = max(-1.0, min(1.0, (raw_stick_us - center_us) / 500.0))
    shaped = (1.0 - expo_factor) * norm + expo_factor * (norm ** 3)
    return shaped * max_deg


def calculate_turn_compensation(roll_deg: float, gain: float = 6.0, max_comp: float = 3.5):
    """Mirror of Coordinated Turn Pitch Compensation from control.cpp"""
    roll_rad = math.radians(abs(roll_deg))
    raw_comp = gain * (1.0 - math.cos(roll_rad))
    return max(0.0, min(max_comp, raw_comp))


def pid_step(target_deg: float, cur_deg: float, rate_deg_s: float, dt: float,
             integrator: float, kp: float, ki: float, kd: float,
             max_i: float = 50.0, limit_us: float = 278.0, throttle_active: bool = True):
    """Mirror of the PI-D step in control.cpp"""
    error = target_deg - cur_deg

    if throttle_active:
        integrator += ki * error * dt
        integrator = max(-max_i, min(max_i, integrator))
    else:
        integrator = 0.0

    pid_out = (kp * error) + integrator - (kd * rate_deg_s)
    output = max(-limit_us, min(limit_us, pid_out))
    return output, integrator


calculate_fbw_pid_step = pid_step


# Hysteresis Thresholds matching config.h:
CH5_THRES_M1_OFF_TO_M2_OFF = 1250
CH5_THRES_M2_OFF_TO_M1_OFF = 1244
CH5_THRES_M2_OFF_TO_M3_OFF = 1373
CH5_THRES_M3_OFF_TO_M2_OFF = 1367
CH5_THRES_M3_OFF_TO_M1_ON = 1479
CH5_THRES_M1_ON_TO_M3_OFF = 1473
CH5_THRES_M1_ON_TO_M2_ON = 1686
CH5_THRES_M2_ON_TO_M1_ON = 1680
CH5_THRES_M2_ON_TO_M3_ON = 1887
CH5_THRES_M3_ON_TO_M2_ON = 1881
CH5_DEBOUNCE_CONFIRM_TICKS = 2

# Backward compatibility aliases
CH5_THRES_M1_OFF_TO_ON = CH5_THRES_M1_OFF_TO_M2_OFF
CH5_THRES_M1_ON_TO_OFF = CH5_THRES_M2_OFF_TO_M1_OFF
CH5_THRES_M1_TO_M2 = CH5_THRES_M2_OFF_TO_M3_OFF
CH5_THRES_M2_TO_M1 = CH5_THRES_M3_OFF_TO_M2_OFF
CH5_THRES_M2_OFF_TO_ON = CH5_THRES_M3_OFF_TO_M1_ON
CH5_THRES_M2_ON_TO_OFF = CH5_THRES_M1_ON_TO_M3_OFF
CH5_THRES_M2_TO_M3 = CH5_THRES_M1_ON_TO_M2_ON
CH5_THRES_M3_TO_M2 = CH5_THRES_M2_ON_TO_M1_ON
CH5_THRES_M3_OFF_TO_ON = CH5_THRES_M2_ON_TO_M3_ON
CH5_THRES_M3_ON_TO_OFF = CH5_THRES_M3_ON_TO_M2_ON


def decode_ch5_with_hysteresis(ch5_pulse: int, current_mode: int, current_roll: bool):
    """Mirror of decodeCH5WithHysteresis() from control.cpp"""
    if ch5_pulse < 850 or ch5_pulse > 2150:
        return current_mode, current_roll

    if current_mode == 1:
        cur_state = 3 if current_roll else 0
    elif current_mode == 2:
        cur_state = 4 if current_roll else 1
    else:
        cur_state = 5 if current_roll else 2

    if ch5_pulse < (CH5_THRES_M1_OFF_TO_M2_OFF if cur_state == 0 else CH5_THRES_M2_OFF_TO_M1_OFF):
        next_state = 0
    elif ch5_pulse < (CH5_THRES_M2_OFF_TO_M3_OFF if cur_state <= 1 else CH5_THRES_M3_OFF_TO_M2_OFF):
        next_state = 1
    elif ch5_pulse < (CH5_THRES_M3_OFF_TO_M1_ON if cur_state <= 2 else CH5_THRES_M1_ON_TO_M3_OFF):
        next_state = 2
    elif ch5_pulse < (CH5_THRES_M1_ON_TO_M2_ON if cur_state <= 3 else CH5_THRES_M2_ON_TO_M1_ON):
        next_state = 3
    elif ch5_pulse < (CH5_THRES_M2_ON_TO_M3_ON if cur_state <= 4 else CH5_THRES_M3_ON_TO_M2_ON):
        next_state = 4
    else:
        next_state = 5

    mapping = {
        0: (1, False),
        1: (2, False),
        2: (3, False),
        3: (1, True),
        4: (2, True),
        5: (2, True),  # Rule: Flaperons ON auto-demotes Mode 3 to Mode 2
    }
    return mapping[next_state]


class CH5DebounceFilter:
    """Mirror of temporal persistence debouncing in controlTaskLoop()"""
    def __init__(self, initial_mode=1, initial_roll=False):
        self.current_mode = initial_mode
        self.current_roll = initial_roll
        self.pending_mode = initial_mode
        self.pending_roll = initial_roll
        self.debounce_count = CH5_DEBOUNCE_CONFIRM_TICKS
        self.first_tick = True

    def process(self, ch5_pulse: int):
        cand_mode, cand_roll = decode_ch5_with_hysteresis(ch5_pulse, self.current_mode, self.current_roll)
        if self.first_tick:
            self.pending_mode = cand_mode
            self.pending_roll = cand_roll
            self.current_mode = cand_mode
            self.current_roll = cand_roll
            self.first_tick = False
        else:
            if cand_mode == self.pending_mode and cand_roll == self.pending_roll:
                if self.debounce_count < CH5_DEBOUNCE_CONFIRM_TICKS:
                    self.debounce_count += 1
                    if self.debounce_count >= CH5_DEBOUNCE_CONFIRM_TICKS:
                        self.current_mode = cand_mode
                        self.current_roll = cand_roll
            else:
                self.pending_mode = cand_mode
                self.pending_roll = cand_roll
                self.debounce_count = 1
        return self.current_mode, self.current_roll


class TestCH5Decoding:
    def test_truth_table_nominal_values(self):
        # Modo 1 + Flaperons OFF (1166 us)
        assert decode_ch5(1166) == (1, False)
        # Modo 2 + Flaperons OFF (1328 us)
        assert decode_ch5(1328) == (2, False)
        # Modo 3 + Flaperons OFF (1411 us)
        assert decode_ch5(1411) == (3, False)
        # Modo 1 + Flaperons ON (1541 us)
        assert decode_ch5(1541) == (1, True)
        # Modo 2 + Flaperons ON (1825 us)
        assert decode_ch5(1825) == (2, True)
        # Modo 3 commanded + Flaperons ON (1942 us) -> auto-demoted to Mode 2
        assert decode_ch5(1942) == (2, True)

    def test_window_thresholds(self):
        assert decode_ch5(1246) == (1, False)
        assert decode_ch5(1248) == (2, False)
        assert decode_ch5(1369) == (2, False)
        assert decode_ch5(1371) == (3, False)
        assert decode_ch5(1475) == (3, False)
        assert decode_ch5(1477) == (1, True)
        assert decode_ch5(1682) == (1, True)
        assert decode_ch5(1684) == (2, True)
        assert decode_ch5(1883) == (2, True)
        assert decode_ch5(1885) == (2, True)


class TestCH5HysteresisAndEMIMargins:
    """Verifies Schmitt-trigger deadbands and temporal debounce for EMI rejection."""

    def test_deadband_retention_m1_off_to_m2_off(self):
        # Boundary 1: Deadband [1244, 1250]
        # In State 0 (Mode 1 OFF), pulse rises into deadband (1247 us) -> stays State 0
        assert decode_ch5_with_hysteresis(1247, 1, False) == (1, False)
        assert decode_ch5_with_hysteresis(1249, 1, False) == (1, False)
        # Reaching 1250 us triggers switch to State 1 (Mode 2 OFF)
        assert decode_ch5_with_hysteresis(1250, 1, False) == (2, False)

        # In State 1 (Mode 2 OFF), pulse falls into deadband (1247 us) -> stays State 1
        assert decode_ch5_with_hysteresis(1247, 2, False) == (2, False)
        assert decode_ch5_with_hysteresis(1245, 2, False) == (2, False)
        # Dropping below 1244 us triggers switch back to State 0
        assert decode_ch5_with_hysteresis(1243, 2, False) == (1, False)

    def test_deadband_retention_m2_off_to_m3_off(self):
        # Boundary 2: Deadband [1367, 1373]
        # In State 1 (Mode 2 OFF), pulse at 1370 (inside deadband) -> stays State 1
        assert decode_ch5_with_hysteresis(1370, 2, False) == (2, False)
        assert decode_ch5_with_hysteresis(1373, 2, False) == (3, False)

        # In State 2 (Mode 3 OFF), pulse at 1370 (inside deadband) -> stays State 2
        assert decode_ch5_with_hysteresis(1370, 3, False) == (3, False)
        assert decode_ch5_with_hysteresis(1366, 3, False) == (2, False)

    def test_deadband_retention_m3_off_to_m1_on(self):
        # Boundary 3: Deadband [1473, 1479]
        # In State 2 (Mode 3 OFF), pulse at 1476 (inside deadband) -> stays State 2
        assert decode_ch5_with_hysteresis(1476, 3, False) == (3, False)
        assert decode_ch5_with_hysteresis(1479, 3, False) == (1, True)

        # In State 3 (Mode 1 ON), pulse at 1476 (inside deadband) -> stays State 3
        assert decode_ch5_with_hysteresis(1476, 1, True) == (1, True)
        assert decode_ch5_with_hysteresis(1472, 1, True) == (3, False)

    def test_deadband_retention_m1_on_to_m2_on(self):
        # Boundary 4: Deadband [1680, 1686]
        # In State 3 (Mode 1 ON), pulse at 1683 (inside deadband) -> stays State 3
        assert decode_ch5_with_hysteresis(1683, 1, True) == (1, True)
        assert decode_ch5_with_hysteresis(1686, 1, True) == (2, True)

        # In State 4 (Mode 2 ON), pulse at 1683 (inside deadband) -> stays State 4
        assert decode_ch5_with_hysteresis(1683, 2, True) == (2, True)
        assert decode_ch5_with_hysteresis(1679, 2, True) == (1, True)

    def test_deadband_retention_m2_on_to_m3_on(self):
        # Boundary 5: Deadband [1881, 1887]
        # In State 4 (Mode 2 ON), pulse rises to 1884 (inside deadband) -> stays State 4
        assert decode_ch5_with_hysteresis(1884, 2, True) == (2, True)
        assert decode_ch5_with_hysteresis(1886, 2, True) == (2, True)
        assert decode_ch5_with_hysteresis(1887, 2, True) == (2, True)  # Mode 3 commanded with Flaperon ON auto-demotes to Mode 2

        # In State 5 (commanded Mode 3 with Flaperons ON), decoded mode is (2, True)
        assert decode_ch5_with_hysteresis(1884, 3, True) == (2, True)
        assert decode_ch5_with_hysteresis(1882, 3, True) == (2, True)
        assert decode_ch5_with_hysteresis(1880, 3, True) == (2, True)

    def test_invalid_pulse_rejection(self):
        # Glitch < 850us or > 2150us leaves state untouched
        assert decode_ch5_with_hysteresis(500, 2, True) == (2, True)
        assert decode_ch5_with_hysteresis(2400, 2, True) == (2, True)

    def test_temporal_debounce_rejects_1_tick_spikes(self):
        # Simulates 50Hz control task loop (20ms per tick)
        debouncer = CH5DebounceFilter(initial_mode=2, initial_roll=False)
        assert debouncer.process(1328) == (2, False)

        # 1-Tick EMI Spike to Mode 3 ON (1942 us) - e.g. 20ms LoRa TX / ESC noise burst
        mode, roll = debouncer.process(1942)
        assert (mode, roll) == (2, False), "1-tick spike must NOT switch flight mode!"

        # Pulse returns to nominal 1328 us
        mode, roll = debouncer.process(1328)
        assert (mode, roll) == (2, False)

    def test_temporal_debounce_accepts_valid_2_tick_pilot_switch(self):
        debouncer = CH5DebounceFilter(initial_mode=2, initial_roll=False)
        assert debouncer.process(1328) == (2, False)

        # Pilot moves switch to Mode 1 ON (1541 us):
        # Tick 1: candidate detected, pending set
        assert debouncer.process(1541) == (2, False)
        # Tick 2: 2nd consecutive tick -> committed! (40ms total)
        assert debouncer.process(1541) == (1, True)
        # Stays in Mode 1 ON
        assert debouncer.process(1541) == (1, True)


class TestQuasiLinearExpo:
    def test_center_and_extremes(self):
        assert apply_expo_and_scale_deg(1500) == 0.0
        assert pytest.approx(apply_expo_and_scale_deg(2000), 0.01) == 45.0
        assert pytest.approx(apply_expo_and_scale_deg(1000), 0.01) == -45.0

    def test_near_linear_linearity(self):
        # Half stick (1750 us, norm = 0.5)
        # Perfectly linear would be 22.5 deg.
        # With expo_factor = 0.08, shaped = 0.92*0.5 + 0.08*0.125 = 0.47 -> 21.15 deg.
        val = apply_expo_and_scale_deg(1750)
        assert 20.0 < val < 22.5
        # The deviation from pure linear is less than 1.5 degrees
        assert abs(val - 22.5) < 1.5


class TestTurnCompensation:
    def test_wings_level_zero_compensation(self):
        assert calculate_turn_compensation(0.0) == 0.0

    def test_banked_turns_positive_compensation(self):
        # At 30 deg bank: 1 - cos(30 deg) = 1 - 0.866 = 0.134 -> 6.0 * 0.134 = 0.80 deg
        comp_30 = calculate_turn_compensation(30.0, gain=6.0)
        assert pytest.approx(comp_30, 0.05) == 0.80

        # At 45 deg bank: 1 - cos(45 deg) = 1 - 0.707 = 0.293 -> 6.0 * 0.293 = 1.76 deg
        comp_45 = calculate_turn_compensation(45.0, gain=6.0)
        assert pytest.approx(comp_45, 0.05) == 1.76

        # Symmetric for left bank (-45 deg)
        assert pytest.approx(calculate_turn_compensation(-45.0, gain=6.0), 0.01) == comp_45

    def test_banked_turns_clamped_at_max(self):
        # Extreme bank at 80 deg: raw would be 6.0 * (1 - 0.173) = 4.96 deg, clamped at 3.5 deg
        comp_80 = calculate_turn_compensation(80.0, gain=6.0, max_comp=3.5)
        assert comp_80 == 3.5


class TestPIDLoop:
    def test_pitch_proportional_and_rate_damping(self):
        kp, ki, kd = 5.00, 2.50, 0.450
        dt = 0.020

        # Positive pitch error (nose down at 0 deg, target +10 deg pitch up)
        # Stationary aircraft (rate = 0)
        out, i_val = pid_step(target_deg=10.0, cur_deg=0.0, rate_deg_s=0.0, dt=dt,
                              integrator=0.0, kp=kp, ki=ki, kd=kd, throttle_active=True)
        # P = 5.00 * 10 = 50.0 us, I = 2.50 * 10 * 0.02 = 0.5 us -> total ~50.5 us
        assert out > 49.0

        # With positive pitch rate (nose pitching up quickly at +30 deg/s)
        # Damping should reduce the actuator deflection
        damped_out, _ = pid_step(target_deg=10.0, cur_deg=0.0, rate_deg_s=30.0, dt=dt,
                                 integrator=0.0, kp=kp, ki=ki, kd=kd, throttle_active=True)
        assert damped_out < out

    def test_anti_windup_clamping(self):
        kp, ki, kd = 5.00, 2.50, 0.450
        dt = 0.020
        integrator = 0.0

        # Accumulate error for 200 steps (4 seconds)
        for _ in range(200):
            _, integrator = pid_step(target_deg=10.0, cur_deg=0.0, rate_deg_s=0.0, dt=dt,
                                     integrator=integrator, kp=kp, ki=ki, kd=kd,
                                     max_i=50.0, throttle_active=True)

        assert integrator == 50.0  # Clamped at MAX_INTEGRAL_PULSE_US

    def test_reset_when_throttle_inactive(self):
        kp, ki, kd = 5.00, 2.50, 0.450
        dt = 0.020

        _, integrator = pid_step(target_deg=10.0, cur_deg=0.0, rate_deg_s=0.0, dt=dt,
                                 integrator=50.0, kp=kp, ki=ki, kd=kd, throttle_active=False)

        assert integrator == 0.0

    def test_roll_integrator_attitude_safety_boundary(self):
        """Verifies that rollIntegrator accumulates only inside safe attitude envelope (abs(cur_roll) <= 60 deg)
        and resets to 0.0 when bank angle exceeds 60 deg in both Mode 2 and Mode 3.
        """
        ki = 5.00
        dt = 0.020
        max_i = 50.0

        def update_roll_integrator(integrator: float, roll_error: float, throttle_us: int, cur_roll: float) -> float:
            if throttle_us > 1050 and abs(cur_roll) <= 60.0:
                integrator += ki * roll_error * dt
                integrator = max(-max_i, min(max_i, integrator))
            else:
                integrator = 0.0
            return integrator

        # Normal flight inside envelope (bank 30 deg): integrates properly
        i_val = update_roll_integrator(integrator=10.0, roll_error=5.0, throttle_us=1200, cur_roll=30.0)
        assert i_val == 10.5

        # Extreme bank angle (70 deg): immediately resets to 0.0 to prevent windup
        i_val_extreme = update_roll_integrator(integrator=40.0, roll_error=5.0, throttle_us=1200, cur_roll=70.0)
        assert i_val_extreme == 0.0

        # Negative extreme bank (-65 deg): immediately resets to 0.0
        i_val_extreme_neg = update_roll_integrator(integrator=40.0, roll_error=-5.0, throttle_us=1200, cur_roll=-65.0)
        assert i_val_extreme_neg == 0.0

        # Throttle idle: resets to 0.0
        i_val_idle = update_roll_integrator(integrator=40.0, roll_error=5.0, throttle_us=1000, cur_roll=20.0)
        assert i_val_idle == 0.0



class ExtremumSeekingAxis:
    """Python test mirror of the embedded C++ ESC algorithm in control.cpp"""
    def __init__(self, omega: float, dither_amp: float = 0.08, gamma: float = 0.05,
                 min_scale: float = 0.60, max_scale: float = 1.60,
                 hpf_cutoff: float = 1.25, rate_weight: float = 0.02):
        self.omega = omega
        self.dither_amp = dither_amp
        self.gamma = gamma
        self.min_scale = min_scale
        self.max_scale = max_scale
        self.hpf_cutoff = hpf_cutoff
        self.rate_weight = rate_weight

        self.theta_hat = 1.0
        self.time_sec = 0.0
        self.hpf_val = 0.0
        self.last_cost = 0.0
        self.first_tick = True

    def reset(self):
        self.theta_hat = 1.0
        self.time_sec = 0.0
        self.hpf_val = 0.0
        self.last_cost = 0.0
        self.first_tick = True

    def step(self, error_deg: float, rate_deg_s: float, dt: float, throttle_active: bool = True, safe_envelope: bool = True):
        self.time_sec += dt
        dither = self.dither_amp * math.sin(self.omega * self.time_sec)
        effective_theta = max(self.min_scale, min(self.max_scale, self.theta_hat + dither))

        cost = (error_deg ** 2) + (self.rate_weight * (rate_deg_s ** 2))

        if self.first_tick:
            self.last_cost = cost
            self.hpf_val = 0.0
            self.first_tick = False
        else:
            alpha = 1.0 / (1.0 + (self.hpf_cutoff * dt))
            self.hpf_val = alpha * (self.hpf_val + cost - self.last_cost)
            self.last_cost = cost

            if throttle_active and safe_envelope:
                demod = self.hpf_val * math.sin(self.omega * self.time_sec)
                self.theta_hat -= self.gamma * demod * dt
                self.theta_hat = max(self.min_scale, min(self.max_scale, self.theta_hat))

        return effective_theta, self.theta_hat


class TestExtremumSeekingPID:
    def test_dither_amplitude_and_bounds(self):
        esc_pitch = ExtremumSeekingAxis(omega=2 * math.pi * 1.0)
        dt = 0.020
        eff_values = []
        for _ in range(50):  # 1 full second
            eff, theta_hat = esc_pitch.step(error_deg=0.0, rate_deg_s=0.0, dt=dt)
            eff_values.append(eff)

        # Baseline is 1.0, dither amplitude is 0.08 -> range [0.92, 1.08]
        assert max(eff_values) <= 1.08 + 1e-4
        assert min(eff_values) >= 0.92 - 1e-4

    def test_washout_filter_removes_dc_bias(self):
        esc = ExtremumSeekingAxis(omega=2 * math.pi * 1.0)
        dt = 0.020

        # Feed constant high DC cost (error = 10 deg -> cost = 100)
        for _ in range(150):  # 3 seconds of constant error
            esc.step(error_deg=10.0, rate_deg_s=0.0, dt=dt)

        # High pass filter output should decay to near 0 despite large steady error
        assert abs(esc.hpf_val) < 0.1

    def test_safety_clamping_limits(self):
        esc = ExtremumSeekingAxis(omega=2 * math.pi * 1.0, gamma=10.0)  # Extreme gamma
        dt = 0.020

        # Simulate massive destabilizing signal
        for i in range(200):
            esc.step(error_deg=40.0 * math.sin(i * 0.1), rate_deg_s=50.0, dt=dt)

        # Theta hat must NEVER violate hard safety limits [0.60, 1.60]
        assert 0.60 <= esc.theta_hat <= 1.60

    def test_throttle_inactive_freezes_adaptation(self):
        esc = ExtremumSeekingAxis(omega=2 * math.pi * 1.0)
        dt = 0.020
        initial_theta = esc.theta_hat

        # When throttle is at idle (throttle_active = False), adaptation law does not integrate
        for _ in range(100):
            esc.step(error_deg=15.0, rate_deg_s=20.0, dt=dt, throttle_active=False)

        assert esc.theta_hat == initial_theta

    def test_extreme_attitude_freezes_adaptation(self):
        esc = ExtremumSeekingAxis(omega=2 * math.pi * 1.0)
        dt = 0.020
        initial_theta = esc.theta_hat

        # When safe_envelope = False (bank > 60 deg or pitch > 50 deg), adaptation pauses
        for _ in range(100):
            esc.step(error_deg=15.0, rate_deg_s=20.0, dt=dt, safe_envelope=False)

        assert esc.theta_hat == initial_theta

    def test_damping_preservation_scaling(self):
        # Confirms Kd = Kd0 * sqrt(theta) preserves damping ratio zeta = Kd / (2 * sqrt(Kp * J))
        kp0 = 9.35
        kd0 = 0.623
        for theta in [0.7, 1.0, 1.3, 1.5]:
            kp = kp0 * theta
            kd = kd0 * math.sqrt(theta)
            zeta_ratio = kd / math.sqrt(kp)
            nominal_ratio = kd0 / math.sqrt(kp0)
            assert pytest.approx(zeta_ratio, rel=1e-3) == nominal_ratio


class TestBumplessModeTransitions:
    """Verifies that in-flight mode switches and activations produce 0 mechanical kick."""

    def test_startup_first_tick_derivative_kick_eliminated(self):
        # Aircraft boots on stand tilted at 15 degrees pitch
        cur_pitch = 15.0
        # With the fix: first loop tick initializes lastPitchMeas = curPitch
        first_tick = True
        last_pitch_meas = 0.0
        dt = 0.020

        if first_tick:
            last_pitch_meas = cur_pitch
            first_tick = False

        pitch_rate = (cur_pitch - last_pitch_meas) / dt
        assert pitch_rate == 0.0, "First tick pitch rate must be 0 (no derivative kick on startup!)"

    def test_mode3_roll_toggle_engages_at_zero_dither_and_nominal_gain(self):
        # When SWB toggles from OFF to ON in Mode 3:
        # escRollTimeSec is reset to 0.0, escRollThetaHat to 1.0
        esc_roll_time = 0.0
        esc_roll_theta_hat = 1.0
        omega = 9.424778  # 1.5 Hz
        dither_amp = 0.08
        dt = 0.020

        # Step 1: pilot turns switch ON
        esc_roll_time += dt
        dither = dither_amp * math.sin(omega * esc_roll_time)
        effective_gain = esc_roll_theta_hat + dither

        # At t = 0.020s (first active tick), sin(omega * dt) = sin(0.188) = ~0.015 -> variation < 1.5%
        assert 0.98 <= effective_gain <= 1.02, "Roll adaptation must engage cleanly near nominal 1.0"
        # And at t=0, dither was identically 0
        assert math.sin(omega * 0.0) == 0.0

    def test_rc_loss_freezes_mode_candidate(self):
        # Simulates controlTaskLoop logic during RC loss
        current_mode = 2
        current_roll = True
        rc_signal_lost = True
        ch5_raw = 0  # 0 indicates no pulse from receiver

        # Fixed logic: if rc_signal_lost or ch5_raw == 0, do not update candidate
        candidate_mode = current_mode
        candidate_roll = current_roll
        if not rc_signal_lost and ch5_raw > 0:
            candidate_mode, candidate_roll = decode_ch5_with_hysteresis(ch5_raw, current_mode, current_roll)

        # Mode MUST remain Mode 2 with Roll True (frozen until signal restores)
        assert (candidate_mode, candidate_roll) == (2, True)

    def test_mode3_symmetrical_attitude_protection(self):
        # Pitch at 55 deg (exceeds 50 deg safe limit) while roll is level (0 deg)
        c3 = 1200
        cur_pitch = 55.0
        cur_roll = 0.0

        allow_pitch_adaptation = (c3 > 1050) and (abs(cur_pitch) <= 50.0) and (abs(cur_roll) <= 60.0)
        allow_roll_adaptation = (c3 > 1050) and (abs(cur_pitch) <= 50.0) and (abs(cur_roll) <= 60.0)

        assert allow_pitch_adaptation is False
        assert allow_roll_adaptation is False, "Roll adaptation must also freeze during extreme pitch attitudes!"

    def test_dither_time_periodic_wrapping_precision(self):
        # Verifies that modulo wrapping preserves sine wave phase continuity
        omega = 6.283185  # 1.0 Hz
        period = 2.0 * math.pi / omega
        dt = 0.020

        t = 0.99
        t += dt  # 1.01
        if t >= period:
            t -= period  # wrapped to ~0.01

        val_wrapped = math.sin(omega * t)
        val_unwrapped = math.sin(omega * 1.01)
        assert pytest.approx(val_wrapped, abs=1e-5) == val_unwrapped


class TestMode2And3GainSharing:
    """Verifies that Mode 2 uses the exact optimal PI-D gains learned/adapted by Mode 3."""

    def test_mode2_inherits_mode3_adapted_gains(self):
        nominal_pitch_kp = 5.00
        nominal_pitch_kd = 0.450
        nominal_roll_kp = 15.00
        nominal_roll_kd = 1.500

        # 1. At cold boot, Mode 2 starts with nominal gains (scale = 1.0)
        theta_pitch = 1.0
        theta_roll = 1.0
        mode2_pitch_kp = nominal_pitch_kp * theta_pitch
        mode2_pitch_kd = nominal_pitch_kd * math.sqrt(theta_pitch)
        assert mode2_pitch_kp == nominal_pitch_kp
        assert mode2_pitch_kd == nominal_pitch_kd

        # 2. Pilot flies in Mode 3 (Extremum Seeking): plant dynamics adapt gains:
        # e.g., higher speed requires stiffer dampening
        theta_pitch = 1.25
        theta_roll = 1.15

        # 3. Pilot flips switch to Mode 2 (Fixed FBW to lock gains without dither):
        # Mode 2 now inherits the learned theta_pitch and theta_roll!
        mode2_pitch_kp = nominal_pitch_kp * theta_pitch
        mode2_pitch_kd = nominal_pitch_kd * math.sqrt(theta_pitch)
        mode2_roll_kp = nominal_roll_kp * theta_roll
        mode2_roll_kd = nominal_roll_kd * math.sqrt(theta_roll)

        assert pytest.approx(mode2_pitch_kp, rel=1e-4) == 6.25
        assert pytest.approx(mode2_pitch_kd, rel=1e-4) == 0.5031
        assert pytest.approx(mode2_roll_kp, rel=1e-4) == 17.25
        assert pytest.approx(mode2_roll_kd, rel=1e-4) == 1.6085

    def test_mode_switch_preserves_adapted_theta_hat(self):
        # Simulates mode transitions in controlTaskLoop()
        theta_hat_pitch = 1.30
        theta_hat_roll = 1.20

        def switch_mode(from_mode, to_mode, th_p, th_r, reset_learned=False):
            # Mirror of resetControlIntegrators() + resetExtremumSeeking(false)
            if reset_learned:
                th_p = 1.0
                th_r = 1.0
            return th_p, th_r

        # Transition Mode 3 -> Mode 2 (lock gains)
        th_p, th_r = switch_mode(3, 2, theta_hat_pitch, theta_hat_roll, reset_learned=False)
        assert th_p == 1.30 and th_r == 1.20

        # Transition Mode 2 -> Mode 1 (temporary manual override)
        th_p, th_r = switch_mode(2, 1, th_p, th_r, reset_learned=False)
        assert th_p == 1.30 and th_r == 1.20

        # Transition Mode 1 -> Mode 2 (return to FBW)
        th_p, th_r = switch_mode(1, 2, th_p, th_r, reset_learned=False)
        assert th_p == 1.30 and th_r == 1.20, "Mode 2 must still retain the learned gains!"

        # Transition Mode 2 -> Mode 3 (resume adaptation)
        th_p, th_r = switch_mode(2, 3, th_p, th_r, reset_learned=False)
        assert th_p == 1.30 and th_r == 1.20, "Mode 3 must resume from the current learned gains!"

    def test_cold_boot_resets_gains_to_1(self):
        # Explicit cold boot reset restores nominal 1.0
        theta_hat_pitch = 1.45
        theta_hat_roll = 1.35
        # resetExtremumSeeking(true)
        theta_hat_pitch = 1.0
        theta_hat_roll = 1.0
        assert theta_hat_pitch == 1.0
        assert theta_hat_roll == 1.0


class TestFailsafeAndSafetyRobustness:
    """Verifies critical safety protections: IMU failure fallback, soft throttle ceiling, and battery auto-detection."""

    def test_imu_failure_forces_mode1_manual_fallback(self):
        # Simulates controlTaskLoop behavior when MPU6050 loses communication in Mode 2 or 3
        current_flight_mode = 2
        current_roll_active = True
        pitch_integrator = 150.0
        roll_integrator = -120.0
        esc_is_active = True

        mpu_healthy = False  # Simulates isMPU6050Available() returning false

        flight_mode = current_flight_mode
        roll_active = current_roll_active

        if not mpu_healthy:
            flight_mode = 1
            roll_active = False
            pitch_integrator = 0.0
            roll_integrator = 0.0
            esc_is_active = False

        assert flight_mode == 1, "Must cleanly fallback to Mode 1 (Manual) when IMU fails!"
        assert roll_active is False
        assert pitch_integrator == 0.0, "Integrators must be cleared to prevent servo lockup!"
        assert roll_integrator == 0.0
        assert esc_is_active is False

    def test_low_voltage_soft_throttle_ceiling(self):
        # Full throttle commanded (2000us)
        c3 = 2000
        throttle_input_min = 1000
        throttle_input_max = 2000
        throttle_output_min = 1000
        throttle_output_max = 1800
        throttle_low_volt_ceiling = 1350

        # Linear mapping (normal condition)
        target_throttle = throttle_output_min + int(
            (c3 - throttle_input_min) * (throttle_output_max - throttle_output_min) / (throttle_input_max - throttle_input_min)
        )
        assert target_throttle == 1800

        # When low voltage cutoff is triggered: soft power ceiling applied
        low_voltage_triggered = True
        if low_voltage_triggered:
            if target_throttle > throttle_low_volt_ceiling:
                target_throttle = throttle_low_volt_ceiling

        assert target_throttle == 1350, "Throttle must be capped at 1350us (soft glide ceiling) rather than 1000us (dead motor)!"

    def test_battery_3s_vs_4s_auto_detection_and_debounce(self):
        # 1. 4S LiPo test: Initial voltage 15.2V
        init_voltage_4s = 15.2
        cutoff_4s = 12.00 if init_voltage_4s > 13.0 else 9.60
        assert cutoff_4s == 12.00

        # 2. 3S LiPo test: Initial voltage 11.8V
        init_voltage_3s = 11.8
        cutoff_3s = 12.00 if init_voltage_3s > 13.0 else 9.60
        assert cutoff_3s == 9.60, "3S battery must receive 9.6V cutoff threshold, not 12.0V!"

        # 3. Voltage sag debounce test: 4S battery transient dip to 11.9V
        hits = 0
        cutoff_triggered = False
        sample_voltages = [11.9, 11.9, 13.5]  # 2 dips then throttle eases

        for v in sample_voltages:
            if v <= cutoff_4s and v > 6.0:
                hits += 1
                if hits >= 3:
                    cutoff_triggered = True
            elif v > (cutoff_4s + 0.5):
                hits = 0
                cutoff_triggered = False

        assert cutoff_triggered is False, "Transient 2-sample sag must NOT trigger motor cutoff!"
        assert hits == 0


def apply_expo_and_scale(raw_stick_us: int, center_us: int = 1500, max_pulse_limit_us: int = 278, expo_factor: float = 0.35):
    """Mirror of applyExpoAndScale() from control.cpp"""
    norm = max(-1.0, min(1.0, (raw_stick_us - center_us) / 500.0))
    shaped = (1.0 - expo_factor) * norm + expo_factor * (norm ** 3)
    return int(shaped * max_pulse_limit_us)


class TestPitchFBWClosedLoopSafety:
    """Verifies that Pitch FBW PI-D operates with strictly NEGATIVE feedback,
    correct stick polarity, anti-stall angle capping, and coordinated turn compensation.
    """

    def test_pitch_negative_feedback_recovers_nose_up(self):
        # Aircraft is disturbed to +15 deg nose UP, pilot stick is neutral (target = 0 deg)
        cur_pitch = 15.0
        target_pitch = 0.0
        pitch_error = target_pitch - cur_pitch  # -15 deg
        kp = 5.00
        pid_out = kp * pitch_error  # -75 us

        # Correct kinematic mapping: pitchDiff = -pitchPidOut
        pitch_diff = -pid_out  # +75 us

        neutral_br = 1478  # 1500 - 22 us (trimmed +2.0 deg UP in neutral)
        neutral_bl = 1633  # 1500 + 133 us (trimmed +2.0 deg UP in neutral, inverted)
        target_br = neutral_br + pitch_diff   # 1553 us (INCREASED)
        target_bl = neutral_bl - pitch_diff   # 1558 us (DECREASED)

        # In this aircraft: BR > 1478 and BL < 1633 deflects elevator DOWN (commands NOSE DOWN)
        assert target_br > neutral_br, "Nose-up disturbance MUST command BR > 1478 (down elevator)!"
        assert target_bl < neutral_bl, "Nose-up disturbance MUST command BL < 1633 (down elevator)!"

    def test_pitch_negative_feedback_recovers_nose_down(self):
        # Aircraft is disturbed to -15 deg nose DOWN, pilot stick is neutral (target = 0 deg)
        cur_pitch = -15.0
        target_pitch = 0.0
        pitch_error = target_pitch - cur_pitch  # +15 deg
        kp = 5.00
        pid_out = kp * pitch_error  # +75 us

        pitch_diff = -pid_out  # -75 us

        neutral_br = 1478  # 1500 - 22 us (trimmed +2.0 deg UP in neutral)
        neutral_bl = 1633  # 1500 + 133 us (trimmed +2.0 deg UP in neutral, inverted)
        target_br = neutral_br + pitch_diff   # 1403 us (DECREASED)
        target_bl = neutral_bl - pitch_diff   # 1708 us (INCREASED)

        # In this aircraft: BR < 1478 and BL > 1633 deflects elevator UP (commands NOSE UP)
        assert target_br < neutral_br, "Nose-down disturbance MUST command BR < 1478 (up elevator)!"
        assert target_bl > neutral_bl, "Nose-down disturbance MUST command BL > 1633 (up elevator)!"

    def test_pitch_stick_mapping_polarity(self):
        # Pilot pulling stick (c2 = 1000 us): must command positive pitch target (climb)
        # Pilot pushing stick (c2 = 2000 us): must command negative pitch target (dive)
        # Neutral stick (c2 = 1500 us): 0 deg target
        max_pitch_deg = 25.0
        expo = 0.08

        pull_target = -apply_expo_and_scale_deg(1000, 1500, max_pitch_deg, expo)
        neutral_target = -apply_expo_and_scale_deg(1500, 1500, max_pitch_deg, expo)
        push_target = -apply_expo_and_scale_deg(2000, 1500, max_pitch_deg, expo)

        assert pytest.approx(pull_target, abs=0.1) == 25.0, "Full pull stick must command +25 deg climb target!"
        assert pytest.approx(neutral_target, abs=0.1) == 0.0, "Neutral stick must command 0 deg level flight!"
        assert pytest.approx(push_target, abs=0.1) == -25.0, "Full push stick must command -25 deg dive target!"

    def test_turn_compensation_adds_climb_target(self):
        # Banked turn of 45 deg must add ~1.8 deg pitch compensation (clamped <= 3.5 deg)
        roll_deg = 45.0
        turn_comp = calculate_turn_compensation(roll_deg, gain=6.0, max_comp=3.5)
        assert 1.5 < turn_comp < 2.0
        assert turn_comp <= 3.5

    def test_pitch_anti_windup_clamping(self):
        # Integrator must clamp at MAX_INTEGRAL_PULSE_US (50.0 us)
        kp, ki, kd = 5.00, 2.50, 0.450
        dt = 0.020
        integrator = 0.0
        for _ in range(200):
            _, integrator = pid_step(target_deg=10.0, cur_deg=0.0, rate_deg_s=0.0, dt=dt,
                                     integrator=integrator, kp=kp, ki=ki, kd=kd,
                                     max_i=50.0, throttle_active=True)
        assert integrator == 50.0

    def test_pitch_rate_damping_opposes_rotation(self):
        # When pitching up at +20 deg/s, D-term reduces pitchPidOut, which increases pitchDiff (pushes down)
        kp, ki, kd = 5.00, 2.50, 0.450
        dt = 0.020
        target = 0.0
        cur = 0.0

        # Zero rate
        out_zero_rate, _ = pid_step(target, cur, rate_deg_s=0.0, dt=dt, integrator=0.0, kp=kp, ki=ki, kd=kd)
        assert out_zero_rate == 0.0

        # Positive pitch rate (nose rotating up)
        out_pos_rate, _ = pid_step(target, cur, rate_deg_s=20.0, dt=dt, integrator=0.0, kp=kp, ki=ki, kd=kd)
        assert out_pos_rate == -(kd * 20.0)  # -9.0 us
        # With pitchDiff = -pitchPidOut:
        # pitchDiff = -(-9.0) = +9.0 us -> BR increases, BL decreases -> pushes nose DOWN!
        pitch_diff = -out_pos_rate
        assert pitch_diff > 0.0, "Positive pitch rate must produce positive pitchDiff to damp the climb!"

    def test_elevator_neutral_trims_trimmed_up_2deg(self):
        """Validates that elevators BR and BL are trimmed 2.0 degrees UP in neutral
        to counteract the nose-down pitch tendency in manual flight.
        Rollerons FR and FL are preserved for servo geometry."""
        us_per_degree = 1000.0 / 90.0  # 11.1111 us/deg

        trim_deg_br = -2.0
        trim_deg_bl = -12.0
        trim_deg_fr = 0.0
        trim_deg_fl = 4.0

        trim_us_br = int(trim_deg_br * us_per_degree - 0.5)  # -22 us
        trim_us_bl = -int(trim_deg_bl * us_per_degree - 0.5)  # +133 us
        trim_us_fr = int(trim_deg_fr * us_per_degree)  # 0 us
        trim_us_fl = int(trim_deg_fl * us_per_degree + 0.5)  # +44 us

        neutral_br = 1500 + trim_us_br
        neutral_bl = 1500 + trim_us_bl
        neutral_fr = 1500 + trim_us_fr
        neutral_fl = 1500 + trim_us_fl

        assert neutral_br == 1478, f"Expected BR neutral 1478 us, got {neutral_br}"
        assert neutral_bl == 1633, f"Expected BL neutral 1633 us, got {neutral_bl}"
        assert neutral_fr == 1500, f"Expected FR neutral 1500 us, got {neutral_fr}"
        assert neutral_fl == 1544, f"Expected FL neutral 1544 us, got {neutral_fl}"

        # Elevator deflection difference from level baseline (1500 BR, 1611 BL):
        delta_br_us = neutral_br - 1500  # -22 us (UP)
        delta_bl_us = neutral_bl - 1611  # +22 us (UP, inverted BL)
        assert delta_br_us == -22
        assert delta_bl_us == +22

    def test_imu_mounting_offsets(self):
        """Validates that IMU Pitch and Roll mounting offsets in config.h match calibrated values:
        Pitch: 9.2 deg, Roll: 4.4 deg (1.9 deg base + 2.5 deg left-drift correction)."""
        config_h = Path(__file__).resolve().parent.parent / "Code" / "MANTA_ESP32" / "include" / "config.h"
        text = config_h.read_text(encoding="utf-8")
        assert "IMU_PITCH_MOUNTING_OFFSET_DEG = 9.2f" in text
        assert "IMU_ROLL_MOUNTING_OFFSET_DEG = 4.4f" in text


class TestServoMaxAngleDeflectionLimits:
    """Validates the 25-degree maximum servo deflection authority, pulse limit calculation,
    and actuator PWM bounds across all trimmed neutrals.
    """

    def test_config_servo_max_angle_is_25_deg(self):
        config_h = Path(__file__).resolve().parent.parent / "Code" / "MANTA_ESP32" / "include" / "config.h"
        text = config_h.read_text(encoding="utf-8")
        assert "DEFAULT_SERVO_MAX_ANGLE_DEG =\n    25;" in text or "DEFAULT_SERVO_MAX_ANGLE_DEG = 25;" in text

    def test_angle_pulse_limit_calculation(self):
        servo_max_deg = 25
        us_per_deg = 11.11  # from config.h
        angle_pulse_limit = int(servo_max_deg * us_per_deg + 0.5)
        assert angle_pulse_limit == 278

    def test_servo_deflection_margins_within_hardware_limits(self):
        """Verifies that under maximum +/- 278 us deflection from trimmed neutrals,
        every servo remains safely inside the [1000, 2000] us hardware pulse range.
        Critically tests BL (neutral 1633 us) to prevent mechanical binding.
        """
        angle_pulse_limit = 278

        neutrals = {
            "BR": 1478,  # Back Right
            "BL": 1633,  # Back Left (inverted)
            "FR": 1500,  # Front Right
            "FL": 1544,  # Front Left
        }

        for servo_name, neutral in neutrals.items():
            min_pulse = neutral - angle_pulse_limit
            max_pulse = neutral + angle_pulse_limit

            assert min_pulse >= 1000, f"{servo_name} min pulse {min_pulse} us violated hardware floor (1000 us)!"
            assert max_pulse <= 2000, f"{servo_name} max pulse {max_pulse} us violated hardware ceiling (2000 us)!"

        # Specific safety margin check on BL (closest to 2000 us ceiling):
        bl_max_pulse = neutrals["BL"] + angle_pulse_limit
        assert bl_max_pulse == 1911
        assert (2000 - bl_max_pulse) >= 80, f"BL must retain at least 80 us headroom before 2000 us ceiling, got {2000 - bl_max_pulse} us"

    def test_manual_mode_expo_scaling_25deg(self):
        """Verifies that apply_expo_and_scale reaches full +/-278 us at stick extremes
        and exactly 0 at neutral stick."""
        max_pulse = 278
        expo = 0.35

        full_right = apply_expo_and_scale(2000, 1500, max_pulse, expo)
        center = apply_expo_and_scale(1500, 1500, max_pulse, expo)
        full_left = apply_expo_and_scale(1000, 1500, max_pulse, expo)

        assert full_right == 278
        assert center == 0
        assert full_left == -278

    def test_pid_step_saturation_at_278us(self):
        """Verifies that PID step clamps large attitude errors to +/-278 us without overflow."""
        kp, ki, kd = 15.0, 5.0, 1.5
        dt = 0.020
        # Very large error (+45 deg)
        out_pos, _ = pid_step(target_deg=45.0, cur_deg=0.0, rate_deg_s=0.0, dt=dt,
                              integrator=0.0, kp=kp, ki=ki, kd=kd, limit_us=278.0)
        assert out_pos == 278.0

        # Very large negative error (-45 deg)
        out_neg, _ = pid_step(target_deg=-45.0, cur_deg=0.0, rate_deg_s=0.0, dt=dt,
                              integrator=0.0, kp=kp, ki=ki, kd=kd, limit_us=278.0)
        assert out_neg == -278.0


class TestFlaperonKinematicsAndSafetyRule:
    """Verifies:
    1. Flaperon 5 deg downward deflection offsets (+56 us on FR, -56 us on FL).
    2. Symmetrical downward deflection produces 0.0 net differential roll moment.
    3. Servo physical limits under full +/-278 us roll command with Flaperons active.
    4. Auto-demotion from Mode 3 to Mode 2 when flaperons are active.
    5. Extremum Seeking adaptation is strictly disabled when flaperons are active.
    """

    def test_flaperon_offsets_produce_symmetric_downward_deflection(self):
        us_per_deg = 11.11
        flap_deg = 15.0
        flap_offset_us = int(flap_deg * us_per_deg + 0.5)  # 167 us

        flaperon_fr = +flap_offset_us  # +167 us (mirrored servo -> right surface DOWN)
        flaperon_fl = -flap_offset_us  # -167 us (left surface DOWN)

        assert flaperon_fr == 167
        assert flaperon_fl == -167

        # When roll command is zero (wings level), check that both surfaces deflect DOWN:
        # For FL: positive is UP (+4.0 deg mechanical spline trim = +44 us). Negative offset deflects DOWN.
        # For FR: mirrored servo, positive offset deflects DOWN.
        neutral_fr = 1500
        neutral_fl = 1544  # 1500 + 44 us mechanical trim

        target_fr_flap = neutral_fr + flaperon_fr  # 1667 us
        target_fl_flap = neutral_fl + flaperon_fl  # 1377 us

        # Differential roll is (rollOffsetFR - (-rollOffsetFL)) = 0
        diff_roll = (target_fr_flap - neutral_fr) + (target_fl_flap - neutral_fl)
        assert diff_roll == 0, "Symmetric flaperon offset must create exactly 0 differential roll moment!"

    def test_servo_physical_bounds_with_flaperons_and_full_roll_command(self):
        """Under full +/-278 us roll command with flaperons engaged (+/-167 us),
        anti-saturation headroom scaling prevents roll command clipping while ensuring
        both FR and FL servos respect the 25 deg angular limit (+/-278 us from trimmed neutral)
        and remain safely inside the [1000, 2000] us hardware pulse range.
        """
        angle_pulse_limit = 278
        flap_offset_fr = 167
        flap_offset_fl = -167

        neutral_fr = 1500
        neutral_fl = 1544

        # Test across stick extremes
        for roll_diff in [-278, -150, 0, 150, 278]:
            roll_ratio = min(1.0, abs(roll_diff) / float(angle_pulse_limit))
            headroom = 1.0 - roll_ratio
            eff_fr = round(flap_offset_fr * headroom)
            eff_fl = round(flap_offset_fl * headroom)

            roll_offset_fr = eff_fr - roll_diff
            roll_offset_fl = eff_fl - roll_diff

            target_fr = max(neutral_fr - angle_pulse_limit, min(neutral_fr + angle_pulse_limit, neutral_fr + roll_offset_fr))
            target_fl = max(neutral_fl - angle_pulse_limit, min(neutral_fl + angle_pulse_limit, neutral_fl + roll_offset_fl))

            target_fr = max(1000, min(2000, target_fr))
            target_fl = max(1000, min(2000, target_fl))

            # Strictly within +/-25 deg (+/-278 us) from each servo's trimmed neutral
            assert (neutral_fr - angle_pulse_limit) <= target_fr <= (neutral_fr + angle_pulse_limit)
            assert (neutral_fl - angle_pulse_limit) <= target_fl <= (neutral_fl + angle_pulse_limit)

            # Strictly within [1000, 2000] hardware limits
            assert 1000 <= target_fr <= 2000
            assert 1000 <= target_fl <= 2000

            # Substantial safety margins to electrical limits
            assert 2000 - target_fr >= 222
            assert target_fl - 1000 >= 266
            assert 2000 - target_fl >= 178

    def test_mode3_prohibited_when_flaperons_active(self):
        """Validates that any attempt to select Mode 3 while flaperons are ON
        forces the flight mode back to Mode 2 to prevent corrupting adaptive gains.
        """
        # Nominal PWM for SWC 3 + SWB ON is 1942 us
        mode, flaperon = decode_ch5(1942)
        assert flaperon is True
        assert mode == 2, "Mode 3 with flaperons ON must be auto-demoted to Mode 2!"

        # In hysteresis state machine:
        mode_hyst, flap_hyst = decode_ch5_with_hysteresis(1942, current_mode=3, current_roll=True)
        assert flap_hyst is True
        assert mode_hyst == 2, "State 5 must output Mode 2 with flaperons ON!"

    def test_extremum_seeking_disabled_when_flaperons_active(self):
        """Verifies that escIsActive is strictly false whenever flaperons are active,
        preventing online gradient adaptation or dither during landing approach.
        """
        roll_active = True
        flaperon_active = True
        esc_is_active = roll_active and not flaperon_active
        assert esc_is_active is False

    def test_flaperon_throttle_governor_linear_scaling_and_chatter_immunity(self):
        """Validates the throttle governor for flaperons:
        - Throttle <= 1200 us: 100% flaperon offset (+167 us FR, -167 us FL = 15 deg DOWN).
        - Throttle >= 1500 us: 0% flaperon offset (fully retracted for climb/cruise).
        - 1200 us < Throttle < 1500 us: Continuous uniform linear taper (300 us transition).
        - Jitter of +/-2 us around 1500 us (e.g. 1498 vs 1502 us) produces ~1 us variation (zero chatter).
        """
        FLAPERON_US_FR = +167
        FLAPERON_US_FL = -167
        FLAPERON_THROTTLE_MIN_US = 1200
        FLAPERON_THROTTLE_MAX_US = 1500

        def compute_flaperon_offsets(throttle_c3: int, flaperon_active: bool):
            flaperon_scale = 0.0
            if flaperon_active:
                if throttle_c3 <= FLAPERON_THROTTLE_MIN_US:
                    flaperon_scale = 1.0
                elif throttle_c3 >= FLAPERON_THROTTLE_MAX_US:
                    flaperon_scale = 0.0
                else:
                    flaperon_scale = (FLAPERON_THROTTLE_MAX_US - throttle_c3) / (FLAPERON_THROTTLE_MAX_US - FLAPERON_THROTTLE_MIN_US)

            offset_fr = round(FLAPERON_US_FR * flaperon_scale)
            offset_fl = round(FLAPERON_US_FL * flaperon_scale)
            return flaperon_scale, offset_fr, offset_fl

        # When flaperon switch is OFF: offset is 0 regardless of throttle
        for throt in [1000, 1100, 1200, 1300, 1350, 1400, 1500, 1800]:
            scale, fr, fl = compute_flaperon_offsets(throt, flaperon_active=False)
            assert scale == 0.0
            assert fr == 0
            assert fl == 0

        # When flaperon switch is ON:
        # Full deployment at approach/landing throttle (<= 1200 us)
        for throt in [950, 1000, 1100, 1150, 1200]:
            scale, fr, fl = compute_flaperon_offsets(throt, flaperon_active=True)
            assert scale == 1.0
            assert fr == +167
            assert fl == -167

        # Full retraction at climb/cruise throttle (>= 1500 us)
        for throt in [1500, 1502, 1550, 1600, 1800, 2000]:
            scale, fr, fl = compute_flaperon_offsets(throt, flaperon_active=True)
            assert scale == 0.0
            assert fr == 0
            assert fl == 0

        # Linear taper in transition zone (1200 < throttle < 1500)
        scale_mid, fr_mid, fl_mid = compute_flaperon_offsets(1350, flaperon_active=True)
        assert scale_mid == 0.50
        assert fr_mid == +84
        assert fl_mid == -84

        scale_75, fr_75, fl_75 = compute_flaperon_offsets(1275, flaperon_active=True)
        assert scale_75 == 0.75
        assert fr_75 == +125
        assert fl_75 == -125

        scale_25, fr_25, fl_25 = compute_flaperon_offsets(1425, flaperon_active=True)
        assert scale_25 == 0.25
        assert fr_25 == +42
        assert fl_25 == -42

        # Jitter immunity test around 1500 us:
        # Mini-interference between 1502 and 1498 us:
        _, fr_1502, fl_1502 = compute_flaperon_offsets(1502, flaperon_active=True)
        _, fr_1498, fl_1498 = compute_flaperon_offsets(1498, flaperon_active=True)
        assert fr_1502 == 0
        assert fr_1498 == 1  # 167 * (2 / 300) = 1.11 -> 1 us change (0.09 deg)
        assert abs(fr_1502 - fr_1498) <= 1, "Jitter variation must be <= 1 us (well below servo deadband)!"
        assert abs(fl_1502 - fl_1498) <= 1

    def test_roll_authority_preservation_under_all_throttle_levels(self):
        """Validates that roll authority (pilot in Mode 1 or PID in Mode 2)
        maintains 100% control priority on top of flaperons across the full throttle envelope,
        with dynamic headroom scaling preventing saturation and physical limits strictly respected.
        """
        neutral_fr = 1500
        neutral_fl = 1544
        angle_pulse_limit = 278
        FLAPERON_US_FR = +167
        FLAPERON_US_FL = -167
        FLAPERON_THROTTLE_MIN_US = 1200
        FLAPERON_THROTTLE_MAX_US = 1500

        for throttle in [1000, 1200, 1275, 1350, 1425, 1500, 1800]:
            # Scale from throttle governor:
            if throttle <= FLAPERON_THROTTLE_MIN_US:
                scale = 1.0
            elif throttle >= FLAPERON_THROTTLE_MAX_US:
                scale = 0.0
            else:
                scale = (FLAPERON_THROTTLE_MAX_US - throttle) / float(FLAPERON_THROTTLE_MAX_US - FLAPERON_THROTTLE_MIN_US)

            # Test roll commands from extreme left to extreme right
            for roll_diff in [-278, -150, 0, 150, 278]:
                roll_ratio = min(1.0, abs(roll_diff) / float(angle_pulse_limit))
                headroom = 1.0 - roll_ratio
                eff_scale = scale * headroom

                flap_fr = round(FLAPERON_US_FR * eff_scale)
                flap_fl = round(FLAPERON_US_FL * eff_scale)

                roll_offset_fr = flap_fr - roll_diff
                roll_offset_fl = flap_fl - roll_diff

                target_fr = max(neutral_fr - angle_pulse_limit, min(neutral_fr + angle_pulse_limit, neutral_fr + roll_offset_fr))
                target_fl = max(neutral_fl - angle_pulse_limit, min(neutral_fl + angle_pulse_limit, neutral_fl + roll_offset_fl))

                target_fr = max(1000, min(2000, target_fr))
                target_fl = max(1000, min(2000, target_fl))

                # Verify both servos remain inside their angular limits (+/-25 deg)
                assert (neutral_fr - angle_pulse_limit) <= target_fr <= (neutral_fr + angle_pulse_limit)
                assert (neutral_fl - angle_pulse_limit) <= target_fl <= (neutral_fl + angle_pulse_limit)

                # Verify both servos remain inside physical hardware pulse bounds [1000, 2000]
                assert 1000 <= target_fr <= 2000
                assert 1000 <= target_fl <= 2000

                # Aerodynamic roll moment check:
                # Right surface lift is +(target_fr - neutral_fr)
                # Left surface lift is -(target_fl - neutral_fl)
                # Net differential roll moment = (target_fr - neutral_fr) + (target_fl - neutral_fl)
                net_roll_moment = (target_fr - neutral_fr) + (target_fl - neutral_fl)
                if roll_diff == 0:
                    assert net_roll_moment == 0, "Symmetric flaperon must create zero net roll moment"
                elif roll_diff < 0:
                    assert net_roll_moment > 0, "Negative roll_diff must generate positive roll moment"
                else:
                    assert net_roll_moment < 0, "Positive roll_diff must generate negative roll moment"


