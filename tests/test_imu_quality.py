import os
import csv
import math
import pytest

DATASET_CSV = os.path.abspath(os.path.join(os.path.dirname(__file__), "data", "manta_imu_calibration_log.csv"))


def load_calibration_dataset(filepath):
    """Loads and validates the cleaned 20 Hz IMU telemetry dataset."""
    assert os.path.exists(filepath), f"IMU calibration dataset not found at: {filepath}"

    records = []
    with open(filepath, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            records.append({
                "record_number": int(row["record_number"]),
                "timestamp_s": float(row["timestamp_s"]),
                "pitch_deg": float(row["pitch_deg"]),
                "roll_deg": float(row["roll_deg"]),
                "altitude_m": float(row["altitude_m"]),
                "rssi_dbm": float(row["rssi_dbm"]),
                "snr_db": float(row["snr_db"]),
            })
    return records


class TestIMUCalibrationQuality:
    """CI/CD test suite validating IMU formulas, level calibration offsets, dynamic tracking and drift resistance."""

    @pytest.fixture(scope="class")
    def dataset(self):
        records = load_calibration_dataset(DATASET_CSV)
        assert len(records) >= 500, f"Expected at least 500 packets in dataset, got {len(records)}"
        return records

    def test_dataset_completeness_and_monotonicity(self, dataset):
        """Verifies packet count, sample rate (>15 Hz) and monotonic timestamp progression."""
        duration = dataset[-1]["timestamp_s"] - dataset[0]["timestamp_s"]
        assert duration >= 30.0, f"Expected flight test duration >= 30.0s, got {duration:.2f}s"

        sample_rate = len(dataset) / duration
        print(f"\n[CI/CD IMU] Dataset Packets: {len(dataset)} | Duration: {duration:.2f}s | Effective Rate: {sample_rate:.1f} Hz")
        assert sample_rate >= 15.0, f"Effective telemetry rate ({sample_rate:.1f} Hz) is below 15 Hz requirement"

        # Monotonic time check
        for i in range(1, len(dataset)):
            dt = dataset[i]["timestamp_s"] - dataset[i - 1]["timestamp_s"]
            assert dt >= 0.0, f"Non-monotonic timestamp detected at row #{dataset[i]['record_number']}: dt={dt}s"

    def test_stationary_initial_level_calibration(self, dataset):
        """Verifies that MANTA resting level calibration is centered within +-1.0 deg with low noise."""
        initial_rest = [r for r in dataset if r["timestamp_s"] <= 3.0]
        assert len(initial_rest) >= 40, "Expected at least 40 resting samples in initial 3s window"

        pitches = [r["pitch_deg"] for r in initial_rest]
        rolls = [r["roll_deg"] for r in initial_rest]

        mean_pitch = sum(pitches) / len(pitches)
        mean_roll = sum(rolls) / len(rolls)
        std_pitch = math.sqrt(sum((p - mean_pitch)**2 for p in pitches) / len(pitches))
        std_roll = math.sqrt(sum((r - mean_roll)**2 for r in rolls) / len(rolls))

        print(f"\n[Initial Rest] Mean Pitch: {mean_pitch:+.2f}° (StdDev: {std_pitch:.3f}°)")
        print(f"[Initial Rest] Mean Roll : {mean_roll:+.2f}° (StdDev: {std_roll:.3f}°)")

        assert abs(mean_pitch) <= 1.0, f"Initial pitch mean ({mean_pitch:+.2f}°) exceeded +-1.0° level threshold"
        assert abs(mean_roll) <= 1.0, f"Initial roll mean ({mean_roll:+.2f}°) exceeded +-1.0° level threshold"
        assert std_pitch < 0.20, f"Initial pitch noise ({std_pitch:.3f}°) exceeded 0.20° threshold"
        assert std_roll < 0.20, f"Initial roll noise ({std_roll:.3f}°) exceeded 0.20° threshold"

    def test_dynamic_maneuver_envelope_coverage(self, dataset):
        """Verifies that the dataset exercises 3D pitch and roll angles without singularities, NaNs or lockups."""
        pitches = [r["pitch_deg"] for r in dataset]
        rolls = [r["roll_deg"] for r in dataset]

        min_pitch, max_pitch = min(pitches), max(pitches)
        min_roll, max_roll = min(rolls), max(rolls)

        print(f"\n[Dynamic Envelope] Pitch Span: [{min_pitch:+.1f}°, {max_pitch:+.1f}°] | Total Span: {max_pitch - min_pitch:.1f}°")
        print(f"[Dynamic Envelope] Roll  Span: [{min_roll:+.1f}°, {max_roll:+.1f}°] | Total Span: {max_roll - min_roll:.1f}°")

        # Check that aggressive attitudes were reached
        assert min_pitch <= -60.0, f"Expected deep pitch down maneuver <= -60.0°, got min {min_pitch:.1f}°"
        assert max_pitch >= 10.0, f"Expected pitch up maneuver >= +10.0°, got max {max_pitch:.1f}°"
        assert max_roll >= 90.0, f"Expected steep roll maneuver >= +90.0°, got max {max_roll:.1f}°"
        assert min_roll <= -30.0, f"Expected negative roll maneuver <= -30.0°, got min {min_roll:.1f}°"

        # Check for NaN / infinite values
        for r in dataset:
            assert not math.isnan(r["pitch_deg"]), f"NaN pitch at record #{r['record_number']}"
            assert not math.isnan(r["roll_deg"]), f"NaN roll at record #{r['record_number']}"
            assert -90.0 <= r["pitch_deg"] <= 90.0, f"Pitch out of physical range [-90, +90]: {r['pitch_deg']}°"
            assert -180.0 <= r["roll_deg"] <= 180.0, f"Roll out of physical range [-180, +180]: {r['roll_deg']}°"

    def test_angular_continuity_and_phase_smoothness(self, dataset):
        """Verifies that angular deltas between consecutive frames are smooth without discontinuous phase glitches."""
        max_delta_pitch = 0.0
        max_delta_roll = 0.0

        for i in range(1, len(dataset)):
            dt = max(0.01, dataset[i]["timestamp_s"] - dataset[i - 1]["timestamp_s"])
            dp = abs(dataset[i]["pitch_deg"] - dataset[i - 1]["pitch_deg"])
            dr = abs(dataset[i]["roll_deg"] - dataset[i - 1]["roll_deg"])

            # Handle 180/-180 wrap-around if any
            if dr > 180.0:
                dr = 360.0 - dr

            if dp > max_delta_pitch:
                max_delta_pitch = dp
            if dr > max_delta_roll:
                max_delta_roll = dr

            # In 50ms at 600 deg/s, max expected physical delta is ~30 deg
            assert dp <= 35.0, f"Discontinuous pitch jump ({dp:.1f}°) at row #{dataset[i]['record_number']}"
            assert dr <= 40.0, f"Discontinuous roll jump ({dr:.1f}°) at row #{dataset[i]['record_number']}"

        print(f"\n[Continuity Check] Max Frame-to-Frame Pitch Delta: {max_delta_pitch:.2f}°")
        print(f"[Continuity Check] Max Frame-to-Frame Roll Delta : {max_delta_roll:.2f}°")

    def test_return_to_rest_level_and_zero_drift(self, dataset):
        """Verifies that after severe 3D dynamic maneuvers, the IMU returns cleanly to level resting with <0.5 deg drift."""
        initial_rest = [r for r in dataset if r["timestamp_s"] <= 3.0]
        final_rest = [r for r in dataset if r["timestamp_s"] >= 34.0]

        assert len(final_rest) >= 40, "Expected at least 40 resting samples in final window"

        init_pitch = sum(r["pitch_deg"] for r in initial_rest) / len(initial_rest)
        init_roll = sum(r["roll_deg"] for r in initial_rest) / len(initial_rest)

        final_pitch = sum(r["pitch_deg"] for r in final_rest) / len(final_rest)
        final_roll = sum(r["roll_deg"] for r in final_rest) / len(final_rest)

        drift_pitch = abs(final_pitch - init_pitch)
        drift_roll = abs(final_roll - init_roll)

        print(f"\n[End-to-End Drift] Pitch Initial: {init_pitch:+.2f}°, Final: {final_pitch:+.2f}°, Drift: {drift_pitch:.3f}° (Limit: <= 0.5°)")
        print(f"[End-to-End Drift] Roll  Initial: {init_roll:+.2f}°, Final: {final_roll:+.2f}°, Drift: {drift_roll:.3f}° (Limit: <= 0.5°)")

        assert abs(final_pitch) <= 1.0, f"Final pitch ({final_pitch:+.2f}°) failed level threshold (+-1.0°)"
        assert abs(final_roll) <= 1.0, f"Final roll ({final_roll:+.2f}°) failed level threshold (+-1.0°)"
        assert drift_pitch <= 0.50, f"Pitch end-to-end drift ({drift_pitch:.3f}°) exceeded 0.50° limit"
        assert drift_roll <= 0.50, f"Roll end-to-end drift ({drift_roll:.3f}°) exceeded 0.50° limit"

    def test_telemetry_link_and_sensor_stability(self, dataset):
        """Verifies LoRa radio reception strength and barometer altitude stability."""
        altitudes = [r["altitude_m"] for r in dataset]
        rssis = [r["rssi_dbm"] for r in dataset]
        snrs = [r["snr_db"] for r in dataset]

        min_rssi, max_rssi = min(rssis), max(rssis)
        min_snr, max_snr = min(snrs), max(snrs)
        alt_span = max(altitudes) - min(altitudes)

        print(f"\n[Telemetry Health] LoRa RSSI: [{min_rssi:.0f}, {max_rssi:.0f}] dBm | SNR: [{min_snr:.1f}, {max_snr:.1f}] dB | Alt Span: {alt_span:.2f}m")

        assert min_rssi >= -85.0, f"Poor signal strength: RSSI min {min_rssi:.0f} dBm"
        assert min_snr >= 6.0, f"Low signal-to-noise ratio: SNR min {min_snr:.1f} dB"
        assert alt_span <= 2.0, f"Unstable altitude reading span ({alt_span:.2f}m) in indoor test"
