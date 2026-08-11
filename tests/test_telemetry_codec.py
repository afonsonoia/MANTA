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
        "batteryVoltage": round(random.uniform(10.00, 18.00), 2),
        "rawADC": round(random.uniform(2000.0, 4000.0), 1),
        "pitch": round(random.uniform(-45.0, 45.0), 1),
        "roll": round(random.uniform(-45.0, 45.0), 1),
        "yaw": round(random.uniform(0.0, 359.9), 1),
        "effectiveCutoff": round(random.uniform(12.00, 15.00), 2),
        "deadband": random.randint(1, 50),
        "lat": round(random.uniform(38.0000000, 39.0000000), 7),
        "lon": round(random.uniform(-9.5000000, -8.5000000), 7),
        "alt": round(random.uniform(0.0, 1000.0), 1),
        "temp": round(random.uniform(-10.0, 50.0), 1),
        "sats": random.randint(0, 20),
        "fix": random.randint(0, 3),
        "rch1": random.randint(1000, 2000),
        "rch2": random.randint(1000, 2000),
        "rch3": random.randint(1000, 2000),
        "rch4": random.randint(1000, 2000),
        "rch5": random.randint(1000, 2000),
        "rcSignalLost": random.choice([True, False])
    }


def test_uncorrupted_packets_100_percent_acceptance_rate():
    """CI/CD Acceptance Test: Verifies that 500 valid, uncorrupted telemetry packets are 100% accepted without any false rejections."""
    random.seed(555)
    N_TESTS = 500
    accepted_count = 0

    for i in range(N_TESTS):
        orig = generate_random_telemetry()
        valid_encoded = encode_telemetry(
            batteryVoltage=orig["batteryVoltage"], rawADC=orig["rawADC"],
            pitch=orig["pitch"], roll=orig["roll"], yaw=orig["yaw"],
            effectiveCutoff=orig["effectiveCutoff"], deadband=orig["deadband"],
            lat=orig["lat"], lon=orig["lon"], alt=orig["alt"], temp=orig["temp"],
            sats=orig["sats"], fix=orig["fix"],
            rch1=orig["rch1"], rch2=orig["rch2"], rch3=orig["rch3"], rch4=orig["rch4"], rch5=orig["rch5"],
            rcSignalLost=orig["rcSignalLost"]
        )

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
        
        encoded = encode_telemetry(
            batteryVoltage=orig["batteryVoltage"],
            rawADC=orig["rawADC"],
            pitch=orig["pitch"],
            roll=orig["roll"],
            yaw=orig["yaw"],
            effectiveCutoff=orig["effectiveCutoff"],
            deadband=orig["deadband"],
            lat=orig["lat"],
            lon=orig["lon"],
            alt=orig["alt"],
            temp=orig["temp"],
            sats=orig["sats"],
            fix=orig["fix"],
            rch1=orig["rch1"],
            rch2=orig["rch2"],
            rch3=orig["rch3"],
            rch4=orig["rch4"],
            rch5=orig["rch5"],
            rcSignalLost=orig["rcSignalLost"]
        )

        assert len(encoded) == PACKET_SIZE, f"[Iteration {i+1}] Encoded packet size mismatch! Expected {PACKET_SIZE}, got {len(encoded)}"

        decoded = decode_telemetry(encoded)
        assert decoded is not None, f"[Iteration {i+1}] Decoding returned None for valid packet!"

        # Assert exact field equality
        assert decoded["batteryVoltage"] == pytest.approx(orig["batteryVoltage"], abs=0.01), f"[Iteration {i+1}] batteryVoltage mismatch"
        assert decoded["rawADC"] == pytest.approx(orig["rawADC"], abs=0.1), f"[Iteration {i+1}] rawADC mismatch"
        assert decoded["pitch"] == pytest.approx(orig["pitch"], abs=0.1), f"[Iteration {i+1}] pitch mismatch"
        assert decoded["roll"] == pytest.approx(orig["roll"], abs=0.1), f"[Iteration {i+1}] roll mismatch"
        assert decoded["yaw"] == pytest.approx(orig["yaw"], abs=0.1), f"[Iteration {i+1}] yaw mismatch"
        assert decoded["effectiveCutoff"] == pytest.approx(orig["effectiveCutoff"], abs=0.01), f"[Iteration {i+1}] effectiveCutoff mismatch"
        assert decoded["deadband"] == orig["deadband"], f"[Iteration {i+1}] deadband mismatch"
        assert decoded["lat"] == pytest.approx(orig["lat"], abs=1e-7), f"[Iteration {i+1}] lat mismatch"
        assert decoded["lon"] == pytest.approx(orig["lon"], abs=1e-7), f"[Iteration {i+1}] lon mismatch"
        assert decoded["alt"] == pytest.approx(orig["alt"], abs=0.1), f"[Iteration {i+1}] alt mismatch"
        assert decoded["temp"] == pytest.approx(orig["temp"], abs=0.1), f"[Iteration {i+1}] temp mismatch"
        assert decoded["sats"] == orig["sats"], f"[Iteration {i+1}] sats mismatch"
        assert decoded["fix"] == orig["fix"], f"[Iteration {i+1}] fix mismatch"
        assert decoded["rc"] == [orig["rch1"], orig["rch2"], orig["rch3"], orig["rch4"], orig["rch5"]], f"[Iteration {i+1}] RC channels mismatch"
        assert decoded["rcSignalLost"] == orig["rcSignalLost"], f"[Iteration {i+1}] rcSignalLost mismatch"

    print(f"[CI/CD Telemetry Codec Test] SUCCESS: All {N_ITERATIONS} randomized telemetry packets matched 100% perfectly!")


def test_single_and_multi_bit_corruption_detection():
    """CI/CD Bit-Corruption Test: Encodes 500 packets, flips 1 to 20 random bits in each, and verifies 100% rejection as invalid."""
    random.seed(123)
    N_TESTS = 500

    print(f"\n[CI/CD Bit Corruption Test] Starting {N_TESTS} single-bit and multi-bit corruption tests...")

    for i in range(N_TESTS):
        orig = generate_random_telemetry()
        valid_encoded = bytearray(encode_telemetry(
            batteryVoltage=orig["batteryVoltage"], rawADC=orig["rawADC"],
            pitch=orig["pitch"], roll=orig["roll"], yaw=orig["yaw"],
            effectiveCutoff=orig["effectiveCutoff"], deadband=orig["deadband"],
            lat=orig["lat"], lon=orig["lon"], alt=orig["alt"], temp=orig["temp"],
            sats=orig["sats"], fix=orig["fix"],
            rch1=orig["rch1"], rch2=orig["rch2"], rch3=orig["rch3"], rch4=orig["rch4"], rch5=orig["rch5"],
            rcSignalLost=orig["rcSignalLost"]
        ))

        # Select random number of bits to corrupt (from 1 to 20 bits)
        n_corrupt_bits = random.randint(1, 20)
        corrupted_encoded = bytearray(valid_encoded)

        # Pick distinct bit positions across the 42-byte (336 bits) payload
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
        valid_encoded = bytearray(encode_telemetry(
            batteryVoltage=orig["batteryVoltage"], rawADC=orig["rawADC"],
            pitch=orig["pitch"], roll=orig["roll"], yaw=orig["yaw"],
            effectiveCutoff=orig["effectiveCutoff"], deadband=orig["deadband"],
            lat=orig["lat"], lon=orig["lon"], alt=orig["alt"], temp=orig["temp"],
            sats=orig["sats"], fix=orig["fix"],
            rch1=orig["rch1"], rch2=orig["rch2"], rch3=orig["rch3"], rch4=orig["rch4"], rch5=orig["rch5"],
            rcSignalLost=orig["rcSignalLost"]
        ))

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


def test_invalid_packet_length_and_header():
    """Verifies that truncated packets or invalid headers are cleanly rejected."""
    assert decode_telemetry(b"") is None
    assert decode_telemetry(b"MT" + b"\x00" * 10) is None
    assert decode_telemetry(b"XX" + b"\x00" * 40) is None


def test_negative_yaw_angle_codec_normalization():
    """Verifies that negative yaw angles (e.g. -15.4 deg during left turn) are correctly normalized without wraparound."""
    data = generate_random_telemetry()
    data["yaw"] = -15.4
    encoded = encode_telemetry(**data)
    decoded = decode_telemetry(encoded)
    assert decoded is not None
    assert decoded["yaw"] == 344.6  # 360.0 - 15.4 = 344.6 deg

