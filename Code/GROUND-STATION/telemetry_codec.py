import struct

PACKET_FORMAT = "<2sHHhhHHBiihhBB5HBH"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)  # 42 bytes
HEADER_BYTES = b'MT'


def calculate_crc16(data: bytes) -> int:
    """Calculates CRC16-MODBUS checksum matching C++ telemetry_codec.h implementation."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def encode_telemetry(
    batteryVoltage: float, rawADC: float,
    pitch: float, roll: float, yaw: float,
    effectiveCutoff: float, deadband: int,
    lat: float, lon: float, alt: float, temp: float,
    sats: int, fix: int,
    rch1: int, rch2: int, rch3: int, rch4: int, rch5: int,
    rcSignalLost: bool, isCalibMode: bool = False
) -> bytes:
    """Serializes telemetry attributes into a 42-byte binary packet with CRC16 checksum."""
    bat_v_x100 = int(round(batteryVoltage * 100))
    bat_adc_x10 = int(round(rawADC * 10))
    pitch_x10 = int(round(pitch * 10))
    roll_x10 = int(round(roll * 10))
    norm_yaw = (yaw % 360.0 + 360.0) % 360.0
    yaw_x10 = int(round(norm_yaw * 10))
    cutoff_x100 = int(round(effectiveCutoff * 100))
    deadband_val = int(deadband)
    lat_x1e7 = int(round(lat * 1e7))
    lon_x1e7 = int(round(lon * 1e7))
    alt_x10 = int(round(alt * 10))
    temp_x10 = int(round(temp * 10))
    sats_val = int(sats)
    fix_val = int(fix)
    flags = (1 if rcSignalLost else 0) | (2 if isCalibMode else 0)

    data_bytes = struct.pack(
        "<2sHHhhHHBiihhBB5HB",
        HEADER_BYTES,
        bat_v_x100, bat_adc_x10,
        pitch_x10, roll_x10, yaw_x10,
        cutoff_x100, deadband_val,
        lat_x1e7, lon_x1e7,
        alt_x10, temp_x10,
        sats_val, fix_val,
        rch1, rch2, rch3, rch4, rch5,
        flags
    )

    crc = calculate_crc16(data_bytes)
    return data_bytes + struct.pack("<H", crc)


def decode_telemetry(packet_bytes: bytes):
    """Deserializes binary packet bytes and verifies CRC16 checksum. Returns dict or None if invalid/corrupt."""
    if not isinstance(packet_bytes, (bytes, bytearray)) or len(packet_bytes) != PACKET_SIZE:
        return None

    data_part = packet_bytes[:40]
    expected_crc = struct.unpack("<H", packet_bytes[40:42])[0]
    actual_crc = calculate_crc16(data_part)

    if expected_crc != actual_crc:
        return None  # CRC mismatch: corrupt transmission!

    unpacked = struct.unpack(PACKET_FORMAT, packet_bytes)
    if unpacked[0] != HEADER_BYTES:
        return None  # Invalid magic header

    flags = unpacked[19]
    return {
        "batteryVoltage": round(unpacked[1] / 100.0, 2),
        "rawADC": round(unpacked[2] / 10.0, 1),
        "pitch": round(unpacked[3] / 10.0, 1),
        "roll": round(unpacked[4] / 10.0, 1),
        "yaw": round(unpacked[5] / 10.0, 1),
        "effectiveCutoff": round(unpacked[6] / 100.0, 2),
        "deadband": unpacked[7],
        "lat": round(unpacked[8] / 1e7, 7),
        "lon": round(unpacked[9] / 1e7, 7),
        "alt": round(unpacked[10] / 10.0, 1),
        "temp": round(unpacked[11] / 10.0, 1),
        "sats": unpacked[12],
        "fix": unpacked[13],
        "rc": [unpacked[14], unpacked[15], unpacked[16], unpacked[17], unpacked[18]],
        "rcSignalLost": bool(flags & 1),
        "isCalibMode": bool(flags & 2)
    }
