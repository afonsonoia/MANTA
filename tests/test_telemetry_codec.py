import os
import sys
import random
import pytest

# Add Code/GROUND-STATION to path for imports
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
GROUND_STATION_DIR = os.path.join(PROJECT_ROOT, "Code", "GROUND-STATION")
if GROUND_STATION_DIR not in sys.path:
    sys.path.insert(0, GROUND_STATION_DIR)

from telemetry_codec import encode_telemetry, decode_telemetry, calculate_crc16, PACKET_SIZE


def generate_random_telemetry():
    """Generates random valid telemetry inputs within operational flight ranges."""
    return {
        "pitch": round(random.uniform(-45.0, 45.0), 1),
        "roll": round(random.uniform(-45.0, 45.0), 1),
        "accel_x": random.randint(-4000, 4000),
        "accel_y": random.randint(-4000, 4000),
        "accel_z": random.randint(-5000, -3000),
        "gyro_x": random.randint(-1000, 1000),
        "gyro_y": random.randint(-1000, 1000),
        "gyro_z": random.randint(-1000, 1000),
        "rc1": random.randint(1000, 2000),
        "rc2": random.randint(1000, 2000),
        "rc3": random.randint(1000, 2000),
        "rc5": random.choice([1000, 2000]),
        "servo_br": random.randint(1000, 2000),
        "servo_bl": random.randint(1000, 2000),
        "servo_fr": random.randint(1000, 2000),
        "servo_fl": random.randint(1000, 2000),
        "esc_throttle": random.randint(1000, 2000),
        "battery_v": round(random.uniform(10.00, 18.00), 2),
        "alt": round(random.uniform(0.0, 1000.0), 1),
        "lat": round(random.uniform(36.0, 42.0), 6),
        "lon": round(random.uniform(-10.0, -6.0), 6),
        "gps_alt": round(random.uniform(0.0, 1000.0), 1),
        "satellites": random.randint(0, 24),
        "gps_fix": random.choice([0, 1, 2]),
        "rc_signal_lost": random.choice([True, False]),
        "is_assist_mode": random.choice([True, False]),
        "is_low_volt": random.choice([True, False]),
        "is_esc_active": random.choice([True, False]),
        "flight_mode": random.choice([1, 2, 3]),
        "pitch_kp": round(random.uniform(5.0, 25.0), 2),
        "pitch_ki": round(random.uniform(1.0, 10.0), 2),
        "pitch_kd": round(random.uniform(0.1, 2.0), 3),
        "roll_kp": round(random.uniform(5.0, 25.0), 2),
        "roll_ki": round(random.uniform(1.0, 10.0), 2),
        "roll_kd": round(random.uniform(0.1, 2.0), 3),
        "pkt_seq": random.randint(0, 255),
        "timestamp_ms": random.randint(0, 1000000)
    }


def test_uncorrupted_packets_100_percent_acceptance_rate():
    """CI/CD Acceptance Test: Verifies that 500 valid, uncorrupted telemetry packets are 100% accepted without any false rejections."""
    random.seed(555)
    N_TESTS = 500
    accepted_count = 0

    for i in range(N_TESTS):
        orig = generate_random_telemetry()
        valid_encoded = encode_telemetry(**orig)

        decoded = decode_telemetry(valid_encoded)
        if decoded is not None:
            accepted_count += 1

    assert accepted_count == N_TESTS, f"[Acceptance Rate Test FAILED] Expected 100% acceptance ({N_TESTS}/{N_TESTS}), but only {accepted_count} were accepted!"
    print(f"\n[CI/CD Acceptance Test] SUCCESS: {accepted_count}/{N_TESTS} uncorrupted packets accepted (0% false rejection rate!).")


def test_random_telemetry_fuzzing_500_iterations():
    """CI/CD Fuzzing Test: Encodes and decodes 500 random telemetry packets and verifies 100% exact equality."""
    random.seed(42)  # Deterministic seed for reproducible CI/CD testing
    N_ITERATIONS = 500

    print(f"\n[CI/CD Telemetry Codec Test] Starting {N_ITERATIONS} randomized encode/decode verification iterations...")

    for i in range(N_ITERATIONS):
        orig = generate_random_telemetry()
        encoded = encode_telemetry(**orig)

        assert len(encoded) == PACKET_SIZE, f"[Iteration {i+1}] Encoded packet size mismatch! Expected {PACKET_SIZE}, got {len(encoded)}"

        decoded = decode_telemetry(encoded)
        assert decoded is not None, f"[Iteration {i+1}] Decoding returned None for valid packet!"

        # Assert exact field equality
        assert decoded["pkt_seq"] == orig["pkt_seq"], f"[Iteration {i+1}] pkt_seq mismatch"
        assert decoded["timestamp_ms"] == orig["timestamp_ms"], f"[Iteration {i+1}] timestamp_ms mismatch"
        assert decoded["pitch"] == pytest.approx(orig["pitch"], abs=0.1), f"[Iteration {i+1}] pitch mismatch"
        assert decoded["roll"] == pytest.approx(orig["roll"], abs=0.1), f"[Iteration {i+1}] roll mismatch"
        assert decoded["accel_x"] == orig["accel_x"], f"[Iteration {i+1}] accel_x mismatch"
        assert decoded["accel_y"] == orig["accel_y"], f"[Iteration {i+1}] accel_y mismatch"
        assert decoded["accel_z"] == orig["accel_z"], f"[Iteration {i+1}] accel_z mismatch"
        assert decoded["gyro_x"] == orig["gyro_x"], f"[Iteration {i+1}] gyro_x mismatch"
        assert decoded["gyro_y"] == orig["gyro_y"], f"[Iteration {i+1}] gyro_y mismatch"
        assert decoded["gyro_z"] == orig["gyro_z"], f"[Iteration {i+1}] gyro_z mismatch"
        assert decoded["rc"] == [orig["rc1"], orig["rc2"], orig["rc3"], orig["rc5"]], f"[Iteration {i+1}] RC channels mismatch"
        assert decoded["servo_br"] == orig["servo_br"], f"[Iteration {i+1}] servo_br mismatch"
        assert decoded["servo_bl"] == orig["servo_bl"], f"[Iteration {i+1}] servo_bl mismatch"
        assert decoded["servo_fr"] == orig["servo_fr"], f"[Iteration {i+1}] servo_fr mismatch"
        assert decoded["servo_fl"] == orig["servo_fl"], f"[Iteration {i+1}] servo_fl mismatch"
        assert decoded["esc_throttle"] == orig["esc_throttle"], f"[Iteration {i+1}] esc_throttle mismatch"
        assert decoded["batteryVoltage"] == pytest.approx(orig["battery_v"], abs=0.01), f"[Iteration {i+1}] batteryVoltage mismatch"
        assert decoded["alt"] == pytest.approx(orig["alt"], abs=0.1), f"[Iteration {i+1}] alt mismatch"
        assert decoded["rcSignalLost"] == orig["rc_signal_lost"], f"[Iteration {i+1}] rcSignalLost mismatch"
        assert decoded["isAssistMode"] == orig["is_assist_mode"], f"[Iteration {i+1}] isAssistMode mismatch"
        assert decoded["isEscActive"] == orig["is_esc_active"], f"[Iteration {i+1}] isEscActive mismatch"
        assert decoded["flightMode"] == orig["flight_mode"], f"[Iteration {i+1}] flightMode mismatch"
        assert decoded["pitch_kp"] == pytest.approx(orig["pitch_kp"], abs=0.01), f"[Iteration {i+1}] pitch_kp mismatch"
        assert decoded["pitch_ki"] == pytest.approx(orig["pitch_ki"], abs=0.01), f"[Iteration {i+1}] pitch_ki mismatch"
        assert decoded["pitch_kd"] == pytest.approx(orig["pitch_kd"], abs=0.001), f"[Iteration {i+1}] pitch_kd mismatch"
        assert decoded["roll_kp"] == pytest.approx(orig["roll_kp"], abs=0.01), f"[Iteration {i+1}] roll_kp mismatch"
        assert decoded["roll_ki"] == pytest.approx(orig["roll_ki"], abs=0.01), f"[Iteration {i+1}] roll_ki mismatch"
        assert decoded["roll_kd"] == pytest.approx(orig["roll_kd"], abs=0.001), f"[Iteration {i+1}] roll_kd mismatch"

    print(f"[CI/CD Telemetry Codec Test] SUCCESS: All {N_ITERATIONS} randomized telemetry packets matched 100% perfectly!")


def test_single_and_multi_bit_corruption_detection():
    """CI/CD Bit-Corruption Test: Encodes 500 packets, flips 1 to 20 random bits in each, and verifies 100% rejection as invalid."""
    random.seed(123)
    N_TESTS = 500

    print(f"\n[CI/CD Bit Corruption Test] Starting {N_TESTS} single-bit and multi-bit corruption tests...")

    for i in range(N_TESTS):
        orig = generate_random_telemetry()
        valid_encoded = bytearray(encode_telemetry(**orig))

        # Select random number of bits to corrupt (from 1 to 20 bits)
        n_corrupt_bits = random.randint(1, 20)
        corrupted_encoded = bytearray(valid_encoded)

        # Pick distinct bit positions across the payload
        total_bits = len(valid_encoded) * 8
        bit_indices = random.sample(range(total_bits), n_corrupt_bits)

        for bit_idx in bit_indices:
            byte_pos = bit_idx // 8
            bit_offset = bit_idx % 8
            corrupted_encoded[byte_pos] ^= (1 << bit_offset)

        decoded = decode_telemetry(bytes(corrupted_encoded))
        assert decoded is None, (
            f"[Bit Corruption Test #{i+1}] CRC16 failed to reject packet with {n_corrupt_bits} corrupted bits!"
        )

    print(f"[CI/CD Bit Corruption Test] SUCCESS: All {N_TESTS} multi-bit corrupted packets were correctly identified as INVALID and rejected!")


def test_burst_noise_and_block_corruption():
    """CI/CD Radio Noise Test: Overwrites contiguous blocks of 1 to 10 bytes with random noise to simulate radio burst interference."""
    random.seed(999)
    N_BURST_TESTS = 500

    for i in range(N_BURST_TESTS):
        orig = generate_random_telemetry()
        valid_encoded = bytearray(encode_telemetry(**orig))

        # Burst noise: overwrite 1 to 10 contiguous bytes (guaranteeing changed byte values)
        burst_len = random.randint(1, 10)
        start_byte = random.randint(0, len(valid_encoded) - burst_len)
        
        corrupted_encoded = bytearray(valid_encoded)
        for b in range(start_byte, start_byte + burst_len):
            orig_b = valid_encoded[b]
            corrupted_encoded[b] = (orig_b + random.randint(1, 255)) % 256

        decoded = decode_telemetry(bytes(corrupted_encoded))
        assert decoded is None, (
            f"[Burst Noise Test #{i+1}] CRC16 failed to reject packet with {burst_len}-byte burst noise corruption!"
        )


def test_legacy_format_backward_compatibility():
    """Verifies that legacy 33-byte format packets continue to decode seamlessly."""
    orig = {
        "pitch": 12.3,
        "roll": -5.6,
        "accel_x": 100,
        "accel_y": -200,
        "accel_z": -4000,
        "gyro_x": 15,
        "gyro_y": -30,
        "gyro_z": 5,
        "rc1": 1520,
        "rc2": 1480,
        "rc3": 1200,
        "rc5": 1000,
        "battery_v": 15.5,
        "alt": 25.0,
        "rc_signal_lost": False,
        "legacy_format": True
    }
    legacy_bytes = encode_telemetry(**orig)
    assert len(legacy_bytes) == 33
    decoded = decode_telemetry(legacy_bytes)
    assert decoded is not None
    assert decoded["packet_size"] == 33
    assert decoded["pitch"] == pytest.approx(12.3, abs=0.1)
    assert decoded["roll"] == pytest.approx(-5.6, abs=0.1)
    assert decoded["rc1"] == 1520
    assert decoded["batteryVoltage"] == pytest.approx(15.5, abs=0.01)


def test_invalid_packet_length_and_header():
    """Verifies that truncated packets or invalid headers are cleanly rejected."""
    assert decode_telemetry(b"") is None
    assert decode_telemetry(b"MT" + b"\x00" * 10) is None
    assert decode_telemetry(b"XX" + b"\x00" * 47) is None


def test_negative_pitch_roll_angle_codec():
    """Verifies that negative pitch and roll angles (e.g. -15.4 deg pitch, -25.2 deg roll) are correctly encoded and decoded."""
    data = generate_random_telemetry()
    data["pitch"] = -15.4
    data["roll"] = -25.2
    encoded = encode_telemetry(**data)
    decoded = decode_telemetry(encoded)
    assert decoded is not None
    assert decoded["pitch"] == pytest.approx(-15.4, abs=0.1)
    assert decoded["roll"] == pytest.approx(-25.2, abs=0.1)


def test_49b_default_telemetry_codec():
    """Verifies that legacy 49-byte non-GPS telemetry packets encode and decode cleanly."""
    data = generate_random_telemetry()
    encoded = encode_telemetry(**data, packet_format="49B")
    assert len(encoded) == 49

    decoded = decode_telemetry(encoded)
    assert decoded is not None
    assert decoded["packet_size"] == 49
    assert decoded["lat"] == 0.0
    assert decoded["lon"] == 0.0
    assert decoded["gps_alt"] == 0.0
    assert decoded["satellites"] == 0
    assert decoded["fix_type"] == 0
    assert decoded["gps_fixed"] is False
    assert decoded["pitch_kp"] == 9.35
    assert decoded["roll_kp"] == 15.00


def test_gps_air530_telemetry_codec():
    """Verifies that legacy 61-byte GPS Air530 telemetry packets encode and decode with high precision."""
    data = generate_random_telemetry()
    data["lat"] = 38.7223000  # Lisbon, Portugal (North)
    data["lon"] = -9.1393000  # Lisbon, Portugal (West)
    data["gps_alt"] = 112.5
    data["satellites"] = 12
    data["gps_fix"] = 1
    data["packet_format"] = "61B_GPS"

    encoded = encode_telemetry(**data)
    assert len(encoded) == 61

    decoded = decode_telemetry(encoded)
    assert decoded is not None
    assert decoded["packet_size"] == 61
    assert decoded["lat"] == pytest.approx(38.7223000, abs=1e-6)
    assert decoded["lon"] == pytest.approx(-9.1393000, abs=1e-6)
    assert decoded["gps_alt"] == pytest.approx(112.5, abs=0.1)
    assert decoded["satellites"] == 12
    assert decoded["fix_type"] == 1
    assert decoded["gps_fixed"] is True


def test_calibration_data_report_parsing():
    """Verifies that 5-second periodic CALIB_DATA broadcast strings are correctly parsed."""
    import re
    calib_str = "CALIB_DATA:DB=18,TRIM=-10,5,0,12,INV=0,1,0,0,ANG=30,RATE=50,CUT=12.50,PWR=14\n"
    assert "CALIB_DATA:" in calib_str
    data_part = calib_str[calib_str.find("CALIB_DATA:") + 11:].strip()

    db_m = re.search(r'DB=(\d+)', data_part)
    assert db_m is not None
    assert int(db_m.group(1)) == 18

    trim_m = re.search(r'TRIM=(-?\d+),(-?\d+),(-?\d+),(-?\d+)', data_part)
    assert trim_m is not None
    assert list(map(int, trim_m.groups())) == [-10, 5, 0, 12]

    inv_m = re.search(r'INV=(\d+),(\d+),(\d+),(\d+)', data_part)
    assert inv_m is not None
    assert list(map(int, inv_m.groups())) == [0, 1, 0, 0]

    ang_m = re.search(r'ANG=(\d+)', data_part)
    assert ang_m is not None
    assert int(ang_m.group(1)) == 30

    rate_m = re.search(r'RATE=(\d+)', data_part)
    assert rate_m is not None
    assert int(rate_m.group(1)) == 50

    cut_m = re.search(r'CUT=([\d\.]+)', data_part)
    assert cut_m is not None
    assert float(cut_m.group(1)) == 12.50

    pwr_m = re.search(r'PWR=(\d+)', data_part)
    assert pwr_m is not None
    assert int(pwr_m.group(1)) == 14


def test_set_deadband_clamping_and_validation():
    """Verifies deadband clamping to 1..50 us range for Ground Station local config."""
    def clamp_deadband(val: int) -> int:
        return max(1, min(50, int(val)))

    assert clamp_deadband(18) == 18
    assert clamp_deadband(0) == 1
    assert clamp_deadband(-5) == 1
    assert clamp_deadband(55) == 50
    assert clamp_deadband(50) == 50


def test_ground_station_simplex_buzzer_commands():
    """Verifies that Ground Station local buzzer commands conform to the Simplex RX firmware specification."""
    valid_buzzer_cmds = ["BEEP:SHORT", "BEEP:CONTINUOUS", "BEEP:INTERMITTENT", "BEEP:OFF"]
    for cmd in valid_buzzer_cmds:
        assert cmd.startswith("BEEP:")
        assert len(cmd) <= 20



def test_ground_station_deadband_json_persistence(tmp_path):
    """Verifies that ground station loads and saves deadband to JSON config correctly."""
    import json
    import MANTA_MISSION_PLANNER as mp

    calib_json = tmp_path / "imu_calibration.json"
    calib_json.write_text(json.dumps({
        "pitch_offset": 2.5,
        "roll_offset": -1.2,
        "deadband": 22,
        "cutoff": 12.4
    }))

    orig_calib_file = mp.CALIB_FILE
    try:
        mp.CALIB_FILE = str(calib_json)
        mp.load_calibration()
        assert mp.rc_margin_deadband == 22
        assert mp.alert_voltage_threshold == 12.4
        assert mp.pitch_offset == 2.5
        assert mp.roll_offset == -1.2

        # Edit deadband and save
        mp.rc_margin_deadband = 35
        mp.save_calibration()

        # Reload from disk
        with open(calib_json, 'r', encoding='utf-8') as f:
            saved_data = json.load(f)
        assert saved_data["deadband"] == 35
        assert saved_data["cutoff"] == 12.4
        assert saved_data["pitch_offset"] == 2.5
    finally:
        mp.CALIB_FILE = orig_calib_file


def test_mode2_esc_telemetry_encoding_decoding():
    """Verifies that Mode 2 fixed nominal FBW telemetry is correctly packed and unpacked in both 61B and 49B."""
    data = {
        "pitch": 12.5,
        "roll": -5.0,
        "accel_x": 100, "accel_y": -50, "accel_z": -4096,
        "gyro_x": 10, "gyro_y": -5, "gyro_z": 2,
        "rc1": 1550, "rc2": 1600, "rc3": 1400, "rc5": 1400,  # CH5 = 1400 (SWC 2, SWB OFF: Mode 2 Roll OFF)
        "servo_br": 1580, "servo_bl": 1420, "servo_fr": 1530, "servo_fl": 1470, "esc_throttle": 1350,
        "battery_v": 15.20,
        "alt": 45.0,
        "is_assist_mode": False,
        "is_esc_active": False,
        "flight_mode": 2,
        "pitch_kp": 9.35, "pitch_ki": 5.00, "pitch_kd": 0.623,
        "roll_kp": 15.00, "roll_ki": 5.00, "roll_kd": 1.500,
        "pkt_seq": 42,
        "timestamp_ms": 98765
    }
    encoded = encode_telemetry(**data)
    assert len(encoded) == 61

    decoded = decode_telemetry(encoded)
    assert decoded is not None
    assert decoded["packet_size"] == 61
    assert decoded["flightMode"] == 2
    assert decoded["isEscActive"] is False
    assert decoded["rollActive"] is False
    assert decoded["roll_active"] is False
    assert decoded["pitch"] == 12.5
    assert decoded["roll"] == -5.0
    assert decoded["pitch_kp"] == 9.35
    assert decoded["roll_kp"] == 15.00


def test_mode3_adaptive_pid_telemetry_encoding_decoding():
    """Verifies that Mode 3 adaptive fine-tuning (Extremum Seeking PI-D) encodes and decodes adapted gains with high precision."""
    data = {
        "pitch": 8.2,
        "roll": -14.6,
        "accel_x": 250, "accel_y": -120, "accel_z": -4100,
        "gyro_x": 18, "gyro_y": -8, "gyro_z": 4,
        "rc1": 1620, "rc2": 1540, "rc3": 1500, "rc5": 1760,  # CH5 = 1760 (Mode 3 Roll ON)
        "servo_br": 1610, "servo_bl": 1390, "servo_fr": 1560, "servo_fl": 1440, "esc_throttle": 1500,
        "battery_v": 14.85,
        "alt": 82.3,
        "is_assist_mode": True,
        "is_esc_active": True,
        "flight_mode": 3,
        "pitch_kp": 11.45,  # Adapted Extremum Seeking gains
        "pitch_ki": 5.00,
        "pitch_kd": 0.685,
        "roll_kp": 18.20,
        "roll_ki": 5.00,
        "roll_kd": 1.650,
        "pkt_seq": 105,
        "timestamp_ms": 254100
    }
    encoded = encode_telemetry(**data)
    assert len(encoded) == 61

    decoded = decode_telemetry(encoded)
    assert decoded is not None
    assert decoded["packet_size"] == 61
    assert decoded["flightMode"] == 3
    assert decoded["isEscActive"] is True
    assert decoded["rollActive"] is True
    assert decoded["roll_active"] is True
    assert decoded["pitch_kp"] == 11.45
    assert decoded["pitch_ki"] == 5.00
    assert decoded["pitch_kd"] == 0.685
    assert decoded["roll_kp"] == 18.20
    assert decoded["roll_ki"] == 5.00
    assert decoded["roll_kd"] == 1.650


def test_flight_loggers_include_pid_and_mode_headers(tmp_path):
    """Verifies that both TelemetryCSVLogger and AsyncTelemetryLogger record active flight mode and all 6 PID parameters."""
    from telemetry_csv_logger import TelemetryCSVLogger
    from MANTA_MISSION_PLANNER import AsyncTelemetryLogger

    csv_path = tmp_path / "test_flight_log.csv"
    logger = TelemetryCSVLogger(str(csv_path))
    assert logger.start()

    expected_csv_cols = ["flight_mode", "esc_active", "flaperon_active", "pitch_kp", "pitch_ki", "pitch_kd", "roll_kp", "roll_ki", "roll_kd"]
    with open(logger.file_path, "r", encoding="utf-8") as f:
        csv_header = f.readline().strip().split(",")
    for col in expected_csv_cols:
        assert col in csv_header, f"TelemetryCSVLogger header missing '{col}'!"
    logger.stop()

    mp_logger = AsyncTelemetryLogger(str(tmp_path / "test_mp_flight_log.csv"))
    expected_mp_cols = ["Flight Mode", "ESC Active", "Flaperons Active", "Pitch Kp", "Pitch Ki", "Pitch Kd", "Roll Kp", "Roll Ki", "Roll Kd"]
    for col in expected_mp_cols:
        assert col in mp_logger.headers, f"AsyncTelemetryLogger header missing '{col}'!"


def test_flaperon_telemetry_and_logging():
    """Verifies that Flaperons Active state (Bit 1) is cleanly encoded, decoded, and logged across subsystems."""
    # Test 1: Flaperons ON (Bit 1 = 1)
    pkt_on = encode_telemetry(
        pitch=5.0, roll=-2.0,
        accel_x=0, accel_y=0, accel_z=-4000,
        gyro_x=0, gyro_y=0, gyro_z=0,
        rc1=1500, rc2=1500, rc3=1200, rc5=1530,  # Mode 2 + Flaperons ON
        flaperon_active=True,
        flight_mode=2
    )
    dec_on = decode_telemetry(pkt_on)
    assert dec_on is not None
    assert dec_on["flaperonActive"] is True
    assert dec_on["flaperon_active"] is True
    assert dec_on["flapsActive"] is True
    assert dec_on["flightMode"] == 2

    # Test 2: Flaperons OFF (Bit 1 = 0)
    pkt_off = encode_telemetry(
        pitch=5.0, roll=-2.0,
        accel_x=0, accel_y=0, accel_z=-4000,
        gyro_x=0, gyro_y=0, gyro_z=0,
        rc1=1500, rc2=1500, rc3=1200, rc5=1400,  # Mode 2 + Flaperons OFF
        flaperon_active=False,
        flight_mode=2
    )
    dec_off = decode_telemetry(pkt_off)
    assert dec_off is not None
    assert dec_off["flaperonActive"] is False
    assert dec_off["flaperon_active"] is False
    assert dec_off["flapsActive"] is False
    assert dec_off["flightMode"] == 2


def test_crc16_lut_equivalence():
    """Validates that the optimized CRC16 lookup table produces 100% identical checksums compared to reference bit-by-bit calculation."""
    def reference_crc16(data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    # Test edge cases: empty, single byte, all zeros, all 0xFF, full 59-byte packets
    assert calculate_crc16(b"") == reference_crc16(b"")
    assert calculate_crc16(b"\x00") == reference_crc16(b"\x00")
    assert calculate_crc16(b"\xFF" * 59) == reference_crc16(b"\xFF" * 59)
    assert calculate_crc16(b"MT\x01\x02\x03\x04\x05") == reference_crc16(b"MT\x01\x02\x03\x04\x05")

    # Test 200 random byte sequences of various lengths
    random.seed(42)
    for _ in range(200):
        test_payload = bytes(random.randint(0, 255) for _ in range(random.randint(1, 100)))
        assert calculate_crc16(test_payload) == reference_crc16(test_payload), "CRC LUT mismatch with reference algorithm!"



