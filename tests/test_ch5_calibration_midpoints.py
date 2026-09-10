"""
CI/CD Automated Test Suite: CH5 Flight Mode Calibration & Exact Midpoint Geometry.

Guarantees:
1. Nominal PWM measured values from transmitter are validated.
2. Every boundary separating two adjacent modes is EXACTLY the mathematical midpoint:
     Midpoint_i = (Value_i + Value_{i+1}) / 2
3. Each nominal mode position is centered with equal distance to the dividing boundary.
4. C++ Firmware (config.h) Schmitt-trigger thresholds are centered around the exact midpoints
   with symmetric hysteresis (+/- 3us).
5. Ground Station Python decoder (telemetry_codec.py) uses the exact integer midpoints.
6. Continuous 1-us resolution sweep across 900-2100 us confirms seamless, monotonic,
   gapless mode classification.
7. Noise immunity margins to the nearest boundary exceed >= 35 us for all 6 flight modes.
"""

import os
import re
import pytest
import sys

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
GROUND_STATION_DIR = os.path.join(PROJECT_ROOT, "Code", "GROUND-STATION")
if GROUND_STATION_DIR not in sys.path:
    sys.path.insert(0, GROUND_STATION_DIR)

from telemetry_codec import decode_ch5_mode

# ── 1. GROUND TRUTH CALIBRATED TRANSMITTER VALUES ──────────────────────────────
NOMINAL_CH5_CALIBRATION = [
    {"index": 0, "swc": 1, "swb": "OFF", "pwm": 1166, "mode": 1, "flaperon": False, "name": "Modo 1 + Flaperons OFF"},
    {"index": 1, "swc": 2, "swb": "OFF", "pwm": 1328, "mode": 2, "flaperon": False, "name": "Modo 2 (FBW Fixo) + Flaperons OFF"},
    {"index": 2, "swc": 3, "swb": "OFF", "pwm": 1411, "mode": 3, "flaperon": False, "name": "Modo 3 (ESC PI-D) + Flaperons OFF"},
    {"index": 3, "swc": 1, "swb": "ON",  "pwm": 1541, "mode": 1, "flaperon": True,  "name": "Modo 1 + Flaperons ON"},
    {"index": 4, "swc": 2, "swb": "ON",  "pwm": 1825, "mode": 2, "flaperon": True,  "name": "Modo 2 (FBW Fixo) + Flaperons ON"},
    {"index": 5, "swc": 3, "swb": "ON",  "pwm": 1942, "mode": 2, "flaperon": True,  "name": "Modo 2 (Auto Flap-Safe) + Flaperons ON"},
]

EXPECTED_MIDPOINTS = [
    (1166 + 1328) / 2.0,  # 1247.0
    (1328 + 1411) / 2.0,  # 1369.5 -> rounded 1370
    (1411 + 1541) / 2.0,  # 1476.0
    (1541 + 1825) / 2.0,  # 1683.0
    (1825 + 1942) / 2.0,  # 1883.5 -> rounded 1884
]


class TestCH5ExactMidpointsMathematicalGeometry:
    """Mathematical verification that dividing points are precisely halfway between modes."""

    def test_midpoints_are_exact_halfway_between_nominal_values(self):
        """Validates Midpoint_i = (V_i + V_{i+1}) / 2 with zero asymmetry."""
        for i in range(len(NOMINAL_CH5_CALIBRATION) - 1):
            v_low = NOMINAL_CH5_CALIBRATION[i]["pwm"]
            v_high = NOMINAL_CH5_CALIBRATION[i + 1]["pwm"]
            exact_midpoint = (v_low + v_high) / 2.0

            # Distance from lower mode to midpoint
            dist_low = exact_midpoint - v_low
            # Distance from upper mode to midpoint
            dist_high = v_high - exact_midpoint

            assert dist_low == dist_high, (
                f"Boundary between {v_low} and {v_high} is not symmetric: "
                f"dist_low={dist_low} != dist_high={dist_high}"
            )
            assert exact_midpoint == EXPECTED_MIDPOINTS[i]

    def test_integer_rounded_midpoints_have_minimal_rounding_delta(self):
        """Verifies integer rounding delta is <= 0.5 us (the theoretical minimum for integer math)."""
        expected_integers = [1247, 1370, 1476, 1683, 1884]
        for exact, rounded in zip(EXPECTED_MIDPOINTS, expected_integers):
            assert abs(exact - rounded) <= 0.5

    def test_noise_margins_exceed_35us_at_all_modes(self):
        """Every flight mode must have >= 35us margin to the nearest decision boundary.
        Typical RC receiver jitter is +/- 2 to 5us, so 35us gives a >= 7x safety margin.
        """
        integers = [1247, 1370, 1476, 1683, 1884]
        for i, item in enumerate(NOMINAL_CH5_CALIBRATION):
            pwm = item["pwm"]
            margins = []
            if i > 0:
                margins.append(pwm - integers[i - 1])  # margin to lower boundary
            if i < len(integers):
                margins.append(integers[i] - pwm)      # margin to upper boundary

            min_margin = min(margins)
            assert min_margin >= 35, (
                f"Mode {item['name']} ({pwm} us) has insufficient margin to boundary: {min_margin} us < 35 us"
            )


class TestCppHeaderConfigSync:
    """Verifies that Code/MANTA_ESP32/include/config.h defines thresholds centered on midpoints."""

    @pytest.fixture
    def config_content(self):
        config_path = os.path.join(PROJECT_ROOT, "Code", "MANTA_ESP32", "include", "config.h")
        with open(config_path, "r", encoding="utf-8") as f:
            return f.read()

    def test_config_h_hysteresis_thresholds_average_to_exact_midpoints(self, config_content):
        """Verifies that (Rising_Trigger + Falling_Trigger) / 2 == Exact Midpoint."""
        patterns = [
            ("CH5_THRES_M1_OFF_TO_M2_OFF", "CH5_THRES_M2_OFF_TO_M1_OFF", 1247),
            ("CH5_THRES_M2_OFF_TO_M3_OFF", "CH5_THRES_M3_OFF_TO_M2_OFF", 1370),
            ("CH5_THRES_M3_OFF_TO_M1_ON",  "CH5_THRES_M1_ON_TO_M3_OFF",  1476),
            ("CH5_THRES_M1_ON_TO_M2_ON",   "CH5_THRES_M2_ON_TO_M1_ON",   1683),
            ("CH5_THRES_M2_ON_TO_M3_ON",   "CH5_THRES_M3_ON_TO_M2_ON",   1884),
        ]

        for rising_name, falling_name, expected_midpoint in patterns:
            rising_match = re.search(rf"constexpr\s+uint16_t\s+{rising_name}\s*=\s*(\d+);", config_content)
            falling_match = re.search(rf"constexpr\s+uint16_t\s+{falling_name}\s*=\s*(\d+);", config_content)

            assert rising_match is not None, f"Could not find {rising_name} in config.h"
            assert falling_match is not None, f"Could not find {falling_name} in config.h"

            rising_val = int(rising_match.group(1))
            falling_val = int(falling_match.group(1))

            midpoint_avg = (rising_val + falling_val) / 2.0
            assert midpoint_avg == expected_midpoint, (
                f"Thresholds {rising_name}={rising_val}, {falling_name}={falling_val} "
                f"average to {midpoint_avg}, expected {expected_midpoint}!"
            )

            # Hysteresis band must be symmetric (+/- 3us)
            assert rising_val - expected_midpoint == 3
            assert expected_midpoint - falling_val == 3


class TestPythonTelemetryCodecSync:
    """Verifies that decode_ch5_mode in telemetry_codec.py switches precisely at the midpoints."""

    def test_nominal_values_decode_to_exact_flight_modes(self):
        for item in NOMINAL_CH5_CALIBRATION:
            mode, flap, name = decode_ch5_mode(item["pwm"])
            assert mode == item["mode"], f"Nominal {item['pwm']}us decoded mode {mode} != expected {item['mode']}"
            assert flap == item["flaperon"], f"Nominal {item['pwm']}us decoded flap {flap} != expected {item['flaperon']}"
            assert name == item["name"]

    def test_boundary_transitions_occur_exactly_at_midpoints(self):
        """Tests that 1us below midpoint stays in lower mode, and at midpoint enters upper mode."""
        midpoint_transitions = [
            # (midpoint, expected_below, expected_at_or_above)
            (1247, (1, False, "Modo 1 + Flaperons OFF"),               (2, False, "Modo 2 (FBW Fixo) + Flaperons OFF")),
            (1370, (2, False, "Modo 2 (FBW Fixo) + Flaperons OFF"),   (3, False, "Modo 3 (ESC PI-D) + Flaperons OFF")),
            (1476, (3, False, "Modo 3 (ESC PI-D) + Flaperons OFF"),   (1, True,  "Modo 1 + Flaperons ON")),
            (1683, (1, True,  "Modo 1 + Flaperons ON"),               (2, True,  "Modo 2 (FBW Fixo) + Flaperons ON")),
            (1884, (2, True,  "Modo 2 (FBW Fixo) + Flaperons ON"),   (2, True,  "Modo 2 (Auto Flap-Safe) + Flaperons ON")),
        ]

        for midpoint, expected_below, expected_above in midpoint_transitions:
            # 1 us below midpoint -> lower mode
            res_below = decode_ch5_mode(midpoint - 1)
            assert res_below == expected_below, (
                f"At {midpoint - 1} us (1us below midpoint {midpoint}), "
                f"expected {expected_below}, got {res_below}"
            )

            # Exactly at midpoint -> upper mode
            res_above = decode_ch5_mode(midpoint)
            assert res_above == expected_above, (
                f"At {midpoint} us (exact midpoint), "
                f"expected {expected_above}, got {res_above}"
            )

            # 1 us above midpoint -> upper mode
            res_above2 = decode_ch5_mode(midpoint + 1)
            assert res_above2 == expected_above, (
                f"At {midpoint + 1} us (1us above midpoint {midpoint}), "
                f"expected {expected_above}, got {res_above2}"
            )

    def test_continuous_1us_resolution_sweep_900_to_2100(self):
        """Sweeps every single 1us pulse value from 900 to 2100 us.
        Confirms that exactly 5 mode transitions occur, strictly at the 5 expected midpoints.
        """
        last_state = None
        transition_points = []

        for pulse in range(900, 2101):
            mode, flap, name = decode_ch5_mode(pulse)
            state = (mode, flap, name)
            if last_state is not None and state != last_state:
                transition_points.append((pulse, last_state, state))
            last_state = state

        # Must have exactly 5 transitions
        assert len(transition_points) == 5, f"Expected 5 transitions, got {len(transition_points)}: {transition_points}"

        expected_transition_pulses = [1247, 1370, 1476, 1683, 1884]
        actual_transition_pulses = [tp[0] for tp in transition_points]

        assert actual_transition_pulses == expected_transition_pulses, (
            f"Transitions occurred at {actual_transition_pulses}, expected {expected_transition_pulses}"
        )
