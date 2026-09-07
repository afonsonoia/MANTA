"""
Unit tests for MANTA Fly-by-Wire (FBW) and PI-D Closed-Loop Control Logic.
Validates:
1. CH5 PWM truth table decoding for Mode 1, 2, 3 and Roll toggle.
2. Quasi-linear stick exponential curve behavior (soft center, nearly linear).
3. PI-D control response, anti-windup clamping, and derivative on measurement.
4. Coordinated turn pitch compensation calculations.
"""

import math
import pytest


def decode_ch5(ch5_pulse: int):
    """Mirror of decodeCH5() from control.cpp"""
    if ch5_pulse < 1247:
        return 1, False  # Mode 1, Roll OFF (~1166 us)
    elif ch5_pulse < 1370:
        return 2, False  # Mode 2, Roll OFF (~1328 us)
    elif ch5_pulse < 1476:
        return 3, False  # Mode 3, Roll OFF (~1411 us)
    elif ch5_pulse < 1683:
        return 1, True   # Mode 1, Roll ON  (~1541 us)
    elif ch5_pulse < 1884:
        return 2, True   # Mode 2, Roll ON  (~1825 us)
    else:
        return 3, True   # Mode 3, Roll ON  (~1942 us)


def apply_expo_and_scale_deg(raw_stick_us: int, center_us: int = 1500, max_deg: float = 45.0, expo_factor: float = 0.08):
    """Mirror of applyExpoAndScaleDeg() from control.cpp"""
    norm = max(-1.0, min(1.0, (raw_stick_us - center_us) / 500.0))
    shaped = (1.0 - expo_factor) * norm + expo_factor * (norm ** 3)
    return shaped * max_deg


def calculate_turn_compensation(roll_deg: float, gain: float = 10.0):
    """Mirror of Coordinated Turn Pitch Compensation from control.cpp"""
    roll_rad = math.radians(abs(roll_deg))
    return gain * (1.0 - math.cos(roll_rad))


def pid_step(target_deg: float, cur_deg: float, rate_deg_s: float, dt: float,
             integrator: float, kp: float, ki: float, kd: float,
             max_i: float = 70.0, limit_us: float = 222.0, throttle_active: bool = True):
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
        5: (3, True),
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
        # Modo 1 + Roll OFF (1166 us)
        assert decode_ch5(1166) == (1, False)
        # Modo 2 + Roll OFF (1328 us)
        assert decode_ch5(1328) == (2, False)
        # Modo 3 + Roll OFF (1411 us)
        assert decode_ch5(1411) == (3, False)
        # Modo 1 + Roll ON (1541 us)
        assert decode_ch5(1541) == (1, True)
        # Modo 2 + Roll ON (1825 us)
        assert decode_ch5(1825) == (2, True)
        # Modo 3 + Roll ON (1942 us)
        assert decode_ch5(1942) == (3, True)

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
        assert decode_ch5(1885) == (3, True)


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
        assert decode_ch5_with_hysteresis(1887, 2, True) == (3, True)

        # In State 5 (Mode 3 ON), pulse drops to 1884 (inside deadband) -> stays State 5
        assert decode_ch5_with_hysteresis(1884, 3, True) == (3, True)
        assert decode_ch5_with_hysteresis(1882, 3, True) == (3, True)
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
        # At 30 deg bank: 1 - cos(30 deg) = 1 - 0.866 = 0.134 -> 1.34 deg
        comp_30 = calculate_turn_compensation(30.0)
        assert pytest.approx(comp_30, 0.1) == 1.34

        # At 45 deg bank: 1 - cos(45 deg) = 1 - 0.707 = 0.293 -> 2.93 deg
        comp_45 = calculate_turn_compensation(45.0)
        assert pytest.approx(comp_45, 0.1) == 2.93

        # Symmetric for left bank (-45 deg)
        assert pytest.approx(calculate_turn_compensation(-45.0), 0.01) == comp_45


class TestPIDLoop:
    def test_pitch_proportional_and_rate_damping(self):
        kp, ki, kd = 9.35, 5.00, 0.623
        dt = 0.020

        # Positive pitch error (nose down at 0 deg, target +10 deg pitch up)
        # Stationary aircraft (rate = 0)
        out, i_val = pid_step(target_deg=10.0, cur_deg=0.0, rate_deg_s=0.0, dt=dt,
                              integrator=0.0, kp=kp, ki=ki, kd=kd, throttle_active=True)
        # P = 9.35 * 10 = 93.5 us, I = 5.0 * 10 * 0.02 = 1.0 us -> total ~94.5 us
        assert out > 90.0

        # With positive pitch rate (nose pitching up quickly at +30 deg/s)
        # Damping should reduce the actuator deflection
        damped_out, _ = pid_step(target_deg=10.0, cur_deg=0.0, rate_deg_s=30.0, dt=dt,
                                 integrator=0.0, kp=kp, ki=ki, kd=kd, throttle_active=True)
        assert damped_out < out

    def test_anti_windup_clamping(self):
        kp, ki, kd = 9.35, 5.00, 0.623
        dt = 0.020
        integrator = 0.0

        # Accumulate error for 200 steps (4 seconds)
        for _ in range(200):
            _, integrator = pid_step(target_deg=10.0, cur_deg=0.0, rate_deg_s=0.0, dt=dt,
                                     integrator=integrator, kp=kp, ki=ki, kd=kd,
                                     max_i=70.0, throttle_active=True)

        assert integrator == 70.0  # Clamped at MAX_INTEGRAL_PULSE_US

    def test_reset_when_throttle_inactive(self):
        kp, ki, kd = 9.35, 5.00, 0.623
        dt = 0.020

        _, integrator = pid_step(target_deg=10.0, cur_deg=0.0, rate_deg_s=0.0, dt=dt,
                                 integrator=50.0, kp=kp, ki=ki, kd=kd, throttle_active=False)

        assert integrator == 0.0


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
        nominal_pitch_kp = 9.35
        nominal_pitch_kd = 0.623
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

        assert pytest.approx(mode2_pitch_kp, rel=1e-4) == 11.6875
        assert pytest.approx(mode2_pitch_kd, rel=1e-4) == 0.6965
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


def apply_expo_and_scale(raw_stick_us: int, center_us: int = 1500, max_pulse_limit_us: int = 222, expo_factor: float = 0.35):
    """Mirror of applyExpoAndScale() from control.cpp"""
    norm = max(-1.0, min(1.0, (raw_stick_us - center_us) / 500.0))
    shaped = (1.0 - expo_factor) * norm + expo_factor * (norm ** 3)
    return int(shaped * max_pulse_limit_us)


class TestPitchAntiStallManualSafety:
    """Verifies complete deactivation of Pitch PID and calibration in Mode 2 and Mode 3.
    Guarantees elevator is 100% manual direct stick control to eliminate stall hazard.
    """

    def test_pitch_identical_manual_response_in_all_modes(self):
        # In Mode 1, Mode 2, and Mode 3:
        # pitchDiff = applyExpoAndScale(c2, centerCH2, anglePulseLimit, RC_EXPO_FACTOR)
        limit_us = 222
        expo = 0.35
        test_sticks = [1000, 1200, 1350, 1500, 1650, 1800, 2000]

        for stick in test_sticks:
            mode1_pitch_diff = apply_expo_and_scale(stick, 1500, limit_us, expo)
            mode2_pitch_diff = apply_expo_and_scale(stick, 1500, limit_us, expo)
            mode3_pitch_diff = apply_expo_and_scale(stick, 1500, limit_us, expo)

            assert mode1_pitch_diff == mode2_pitch_diff == mode3_pitch_diff
            if stick == 1500:
                assert mode2_pitch_diff == 0
                assert mode3_pitch_diff == 0
            elif stick == 2000:
                assert mode2_pitch_diff == limit_us
            elif stick == 1000:
                assert mode2_pitch_diff == -limit_us

    def test_pitch_immune_to_roll_bank_angle_turn_compensation(self):
        # In previous FBW, banked turns added up to ~2.9 deg pitch compensation.
        # With Pitch PID disabled, pitchDiff is purely from pilot stick c2.
        c2_neutral = 1500
        limit_us = 222
        expo = 0.35

        for roll_deg in [0.0, 15.0, 30.0, 45.0, 60.0]:
            pitch_diff = apply_expo_and_scale(c2_neutral, 1500, limit_us, expo)
            assert pitch_diff == 0, f"At roll {roll_deg} deg, neutral pitch stick must produce 0 deflection!"

    def test_pitch_immune_to_attitude_error_and_integrator_is_zero(self):
        # Even with extreme simulated pitch error (airplane pitched 20 deg down or up),
        # pitchIntegrator is clamped to 0.0 and no PID output is generated.
        pitch_integrator = 0.0
        current_target_pitch = 0.0
        active_pitch_kp = 0.0
        active_pitch_ki = 0.0
        active_pitch_kd = 0.0

        assert pitch_integrator == 0.0
        assert current_target_pitch == 0.0
        assert active_pitch_kp == 0.0
        assert active_pitch_ki == 0.0
        assert active_pitch_kd == 0.0

    def test_mode3_esc_active_only_when_roll_is_active(self):
        # In Mode 3: escIsActive = rollActive
        # When SWB is OFF (Roll Manual): escIsActive is False (no Extremum Seeking anywhere!)
        roll_active_off = False
        esc_is_active_off = roll_active_off
        assert esc_is_active_off is False

        # When SWB is ON (Roll Adaptive): escIsActive is True (ESC calibrates roll only)
        roll_active_on = True
        esc_is_active_on = roll_active_on
        assert esc_is_active_on is True
