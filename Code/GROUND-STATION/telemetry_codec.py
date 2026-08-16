import struct

# Supported Packet Formats:
# 4CH (Current default, 33 bytes): Magic 'MT' (2s), pitch_x10 (h), roll_x10 (h), 6 IMU (6h), 4 RC (4H: CH1,CH2,CH3,CH5), bat_v_x100 (H), alt_x10 (h), flags (B), crc16 (H)
# 3CH (Legacy, 31 bytes):          Magic 'MT' (2s), pitch_x10 (h), roll_x10 (h), 6 IMU (6h), 3 RC (3H: CH1,CH2,CH3), bat_v_x100 (H), alt_x10 (h), flags (B), crc16 (H)
# 5CH (Extended, 35 bytes):        Magic 'MT' (2s), pitch_x10 (h), roll_x10 (h), 6 IMU (6h), 5 RC (5H: CH1..CH5), bat_v_x100 (H), alt_x10 (h), flags (B), crc16 (H)

PACKET_FORMAT_4CH = "<2shh6h4HHhBH"  # 33 bytes
PACKET_FORMAT_3CH = "<2shh6h3HHhBH"  # 31 bytes
PACKET_FORMAT_5CH = "<2shh6h5HHhBH"  # 35 bytes

TELEMETRY_PACKET_FORMAT = PACKET_FORMAT_4CH
TELEMETRY_PACKET_SIZE = struct.calcsize(PACKET_FORMAT_4CH) # 33 bytes
PACKET_SIZE = TELEMETRY_PACKET_SIZE
SUPPORTED_PACKET_SIZES = [33, 31, 35]

def calculate_crc16(data: bytes) -> int:
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
    pitch: float, roll: float,
    accel_x: int, accel_y: int, accel_z: int,
    gyro_x: int, gyro_y: int, gyro_z: int,
    rc1: int, rc2: int, rc3: int, rc5: int = 1000,
    battery_v: float = 0.0, alt: float = 0.0,
    rc_signal_lost: bool = False
) -> bytes:
    pitch_x10 = int(round(pitch * 10))
    roll_x10 = int(round(roll * 10))
    bat_x100 = int(round(battery_v * 100))
    alt_x10 = int(round(alt * 10))
    flags = 1 if rc_signal_lost else 0

    header = b"MT"
    payload_without_crc = struct.pack(
        PACKET_FORMAT_4CH[:-1], # without trailing 'H' CRC
        header,
        pitch_x10, roll_x10,
        accel_x, accel_y, accel_z,
        gyro_x, gyro_y, gyro_z,
        rc1, rc2, rc3, rc5,
        bat_x100,
        alt_x10,
        flags
    )
    crc = calculate_crc16(payload_without_crc)
    return payload_without_crc + struct.pack("<H", crc)

def decode_telemetry(packet_bytes: bytes) -> dict | None:
    if len(packet_bytes) not in (31, 33, 35):
        return None

    header = packet_bytes[:2]
    if header != b"MT":
        return None

    received_crc = struct.unpack("<H", packet_bytes[-2:])[0]
    computed_crc = calculate_crc16(packet_bytes[:-2])
    if received_crc != computed_crc:
        return None

    if len(packet_bytes) == 33:
        unpacked = struct.unpack(PACKET_FORMAT_4CH, packet_bytes)
        return {
            "pitch": round(unpacked[1] / 10.0, 1),
            "roll": round(unpacked[2] / 10.0, 1),
            "accel_x": unpacked[3],
            "accel_y": unpacked[4],
            "accel_z": unpacked[5],
            "gyro_x": unpacked[6],
            "gyro_y": unpacked[7],
            "gyro_z": unpacked[8],
            "rc": [unpacked[9], unpacked[10], unpacked[11], unpacked[12]],
            "batteryVoltage": round(unpacked[13] / 100.0, 2),
            "alt": round(unpacked[14] / 10.0, 1),
            "rcSignalLost": bool(unpacked[15]),
            "packet_size": 33
        }
    elif len(packet_bytes) == 31:
        unpacked = struct.unpack(PACKET_FORMAT_3CH, packet_bytes)
        return {
            "pitch": round(unpacked[1] / 10.0, 1),
            "roll": round(unpacked[2] / 10.0, 1),
            "accel_x": unpacked[3],
            "accel_y": unpacked[4],
            "accel_z": unpacked[5],
            "gyro_x": unpacked[6],
            "gyro_y": unpacked[7],
            "gyro_z": unpacked[8],
            "rc": [unpacked[9], unpacked[10], unpacked[11], 1000],
            "batteryVoltage": round(unpacked[12] / 100.0, 2),
            "alt": round(unpacked[13] / 10.0, 1),
            "rcSignalLost": bool(unpacked[14]),
            "packet_size": 31
        }
    elif len(packet_bytes) == 35:
        unpacked = struct.unpack(PACKET_FORMAT_5CH, packet_bytes)
        return {
            "pitch": round(unpacked[1] / 10.0, 1),
            "roll": round(unpacked[2] / 10.0, 1),
            "accel_x": unpacked[3],
            "accel_y": unpacked[4],
            "accel_z": unpacked[5],
            "gyro_x": unpacked[6],
            "gyro_y": unpacked[7],
            "gyro_z": unpacked[8],
            "rc": [unpacked[9], unpacked[10], unpacked[11], unpacked[13]],
            "batteryVoltage": round(unpacked[14] / 100.0, 2),
            "alt": round(unpacked[15] / 10.0, 1),
            "rcSignalLost": bool(unpacked[16]),
            "packet_size": 35
        }
