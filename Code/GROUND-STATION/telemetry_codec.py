import struct

# Supported Packet Formats:
# PID / Adaptive Telemetry (Current default, 61 bytes):
#   Magic 'MT' (2s), pkt_seq (B), timestamp_ms (I), pitch_x10 (h), roll_x10 (h),
#   6 IMU (6h), 4 RC (4H: CH1,CH2,CH3,CH5), 5 Actuators (5H: BR,BL,FR,FL,ESC),
#   bat_v_x100 (H), alt_x10 (h), flags (B), flight_mode (B),
#   pitch_kp_x100 (H), pitch_ki_x100 (H), pitch_kd_x1000 (H),
#   roll_kp_x100 (H), roll_ki_x100 (H), roll_kd_x1000 (H), crc16 (H)
# Legacy PID / SysID (49 bytes): Magic 'MT' (2s), pkt_seq (B), timestamp_ms (I), pitch_x10 (h), roll_x10 (h), 6 IMU (6h), 4 RC (4H), 5 Actuators (5H), bat_v_x100 (H), alt_x10 (h), flags (B), flight_mode (B), crc16 (H)
# Legacy GPS (61 bytes): Magic 'MT' (2s), pkt_seq (B), timestamp_ms (I), pitch_x10 (h), roll_x10 (h), 6 IMU (6h), 4 RC (4H), 5 Actuators (5H), bat_v_x100 (H), alt_x10 (h), lat_e7 (i), lon_e7 (i), gps_alt_x10 (h), satellites (B), gps_fix (B), flags (B), reserved (B), crc16 (H)
# 4CH (Legacy, 33 bytes): Magic 'MT' (2s), pitch_x10 (h), roll_x10 (h), 6 IMU (6h), 4 RC (4H), bat_v_x100 (H), alt_x10 (h), flags (B), crc16 (H)
# 3CH (Legacy, 31 bytes): Magic 'MT' (2s), pitch_x10 (h), roll_x10 (h), 6 IMU (6h), 3 RC (3H), bat_v_x100 (H), alt_x10 (h), flags (B), crc16 (H)
# 5CH (Legacy, 35 bytes): Magic 'MT' (2s), pitch_x10 (h), roll_x10 (h), 6 IMU (6h), 5 RC (5H), bat_v_x100 (H), alt_x10 (h), flags (B), crc16 (H)

PACKET_FORMAT_PID = "<2sBIhh6h4H5HHhBB6HH"        # 61 bytes (Default with dynamic PID)
PACKET_FORMAT_61B = PACKET_FORMAT_PID              # 61 bytes (Default)
PACKET_FORMAT_49B = "<2sBIhh6h4H5HHhBBH"           # 49 bytes (Legacy without PID)
PACKET_FORMAT_61B_GPS = "<2sBIhh6h4H5HHhiihBBBBH"  # 61 bytes (Legacy with GPS)
PACKET_FORMAT_4CH = "<2shh6h4HHhBH"                # 33 bytes
PACKET_FORMAT_3CH = "<2shh6h3HHhBH"                # 31 bytes
PACKET_FORMAT_5CH = "<2shh6h5HHhBH"                # 35 bytes

TELEMETRY_PACKET_FORMAT = PACKET_FORMAT_PID
TELEMETRY_PACKET_SIZE = struct.calcsize(PACKET_FORMAT_PID)  # 61 bytes
PACKET_SIZE = TELEMETRY_PACKET_SIZE
SUPPORTED_PACKET_SIZES = [61, 49, 33, 31, 35]
SUPPORTED_PACKET_SIZES_DESC = (61, 49, 35, 33, 31)

# Pre-compiled Struct instances for zero-allocation parsing
STRUCT_PID = struct.Struct(PACKET_FORMAT_PID)
STRUCT_PID_NO_CRC = struct.Struct(PACKET_FORMAT_PID[:-1])
STRUCT_49B = struct.Struct(PACKET_FORMAT_49B)
STRUCT_49B_NO_CRC = struct.Struct(PACKET_FORMAT_49B[:-1])
STRUCT_61B_GPS = struct.Struct(PACKET_FORMAT_61B_GPS)
STRUCT_61B_GPS_NO_CRC = struct.Struct(PACKET_FORMAT_61B_GPS[:-1])
STRUCT_4CH = struct.Struct(PACKET_FORMAT_4CH)
STRUCT_4CH_NO_CRC = struct.Struct(PACKET_FORMAT_4CH[:-1])
STRUCT_3CH = struct.Struct(PACKET_FORMAT_3CH)
STRUCT_3CH_NO_CRC = struct.Struct(PACKET_FORMAT_3CH[:-1])
STRUCT_5CH = struct.Struct(PACKET_FORMAT_5CH)
STRUCT_5CH_NO_CRC = struct.Struct(PACKET_FORMAT_5CH[:-1])
STRUCT_CRC = struct.Struct("<H")

CRC16_TABLE = (
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
)


def decode_ch5_mode(ch5_pwm: int):
    """
    Decodes CH5 PWM into (mode_number: int, flaperon_on: bool, mode_name: str)
    SWC (Pos 1, 2, 3) + SWB (OFF, ON) medido e calibrado:
      SWC 1 + SWB OFF: ~1166 us -> Modo 1 + Flaperons OFF (Pitch Manual + Roll Assist)
      SWC 2 + SWB OFF: ~1328 us -> Modo 2 (FBW Fixo) + Flaperons OFF (Pitch & Roll FBW Fixo)
      SWC 3 + SWB OFF: ~1411 us -> Modo 3 (ESC PI-D) + Flaperons OFF (Pitch & Roll ESC PI-D)
      SWC 1 + SWB ON:  ~1541 us -> Modo 1 + Flaperons ON  (Pitch Manual + Roll Assist + Flaps 15 deg DOWN)
      SWC 2 + SWB ON:  ~1825 us -> Modo 2 (FBW Fixo) + Flaperons ON  (Pitch & Roll FBW Fixo + Flaps 15 deg DOWN)
      SWC 3 + SWB ON:  ~1942 us -> Modo 2 (Auto Flap-Safe) + Flaperons ON (Safety: Modo 3 demoted to 2)
    """
    if ch5_pwm < 1247:
        return 1, False, "Modo 1 + Flaperons OFF"
    elif ch5_pwm < 1370:
        return 2, False, "Modo 2 (FBW Fixo) + Flaperons OFF"
    elif ch5_pwm < 1476:
        return 3, False, "Modo 3 (ESC PI-D) + Flaperons OFF"
    elif ch5_pwm < 1683:
        return 1, True, "Modo 1 + Flaperons ON"
    elif ch5_pwm < 1884:
        return 2, True, "Modo 2 (FBW Fixo) + Flaperons ON"
    else:
        # Rule: When flaperons are ON, Mode 3 cannot be active -> Auto-demoted to Mode 2!
        return 2, True, "Modo 2 (Auto Flap-Safe) + Flaperons ON"


def calculate_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc = (crc >> 8) ^ CRC16_TABLE[(crc ^ byte) & 0xFF]
    return crc


def encode_telemetry(
    pitch: float, roll: float,
    accel_x: int, accel_y: int, accel_z: int,
    gyro_x: int, gyro_y: int, gyro_z: int,
    rc1: int, rc2: int, rc3: int, rc5: int = 1000,
    battery_v: float = 0.0, alt: float = 0.0,
    lat: float = 0.0, lon: float = 0.0, gps_alt: float = 0.0,
    satellites: int = 0, gps_fix: int = 0,
    rc_signal_lost: bool = False,
    pkt_seq: int = 0,
    timestamp_ms: int = 0,
    servo_br: int = 1500,
    servo_bl: int = 1500,
    servo_fr: int = 1500,
    servo_fl: int = 1500,
    esc_throttle: int = 1000,
    is_assist_mode: bool = False,
    flaperon_active: bool = False,
    roll_active: bool = False,
    is_low_volt: bool = False,
    is_esc_active: bool = False,
    flight_mode: int = 1,
    pitch_kp: float = 9.35,
    pitch_ki: float = 5.00,
    pitch_kd: float = 0.623,
    roll_kp: float = 15.00,
    roll_ki: float = 5.00,
    roll_kd: float = 1.500,
    legacy_format: bool = False,
    packet_format: str = "61B"
) -> bytes:
    if legacy_format or packet_format == "4CH":
        pitch_x10 = int(round(pitch * 10))
        roll_x10 = int(round(roll * 10))
        bat_x100 = int(round(battery_v * 100))
        alt_x10 = int(round(alt * 10))
        flags = 1 if rc_signal_lost else 0
        header = b"MT"
        payload_without_crc = struct.pack(
            PACKET_FORMAT_4CH[:-1],
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

    if packet_format == "3CH":
        pitch_x10 = int(round(pitch * 10))
        roll_x10 = int(round(roll * 10))
        bat_x100 = int(round(battery_v * 100))
        alt_x10 = int(round(alt * 10))
        flags = 1 if rc_signal_lost else 0
        header = b"MT"
        payload_without_crc = struct.pack(
            PACKET_FORMAT_3CH[:-1],
            header,
            pitch_x10, roll_x10,
            accel_x, accel_y, accel_z,
            gyro_x, gyro_y, gyro_z,
            rc1, rc2, rc3,
            bat_x100,
            alt_x10,
            flags
        )
        crc = calculate_crc16(payload_without_crc)
        return payload_without_crc + struct.pack("<H", crc)

    if packet_format == "5CH":
        pitch_x10 = int(round(pitch * 10))
        roll_x10 = int(round(roll * 10))
        bat_x100 = int(round(battery_v * 100))
        alt_x10 = int(round(alt * 10))
        flags = 1 if rc_signal_lost else 0
        header = b"MT"
        payload_without_crc = struct.pack(
            PACKET_FORMAT_5CH[:-1],
            header,
            pitch_x10, roll_x10,
            accel_x, accel_y, accel_z,
            gyro_x, gyro_y, gyro_z,
            rc1, rc2, rc3, 1500, rc5,
            bat_x100,
            alt_x10,
            flags
        )
        crc = calculate_crc16(payload_without_crc)
        return payload_without_crc + struct.pack("<H", crc)

    if legacy_format or packet_format == "49B":
        pitch_x10 = int(round(pitch * 10))
        roll_x10 = int(round(roll * 10))
        bat_x100 = int(round(battery_v * 100))
        alt_x10 = int(round(alt * 10))
        assist_flag = is_assist_mode or flaperon_active or roll_active
        flags = (
            (1 if rc_signal_lost else 0) |
            (2 if assist_flag else 0) |
            (4 if is_low_volt else 0) |
            (8 if is_esc_active else 0)
        )
        header = b"MT"
        payload_without_crc = struct.pack(
            PACKET_FORMAT_49B[:-1],
            header,
            pkt_seq & 0xFF,
            timestamp_ms & 0xFFFFFFFF,
            pitch_x10, roll_x10,
            accel_x, accel_y, accel_z,
            gyro_x, gyro_y, gyro_z,
            rc1, rc2, rc3, rc5,
            servo_br, servo_bl, servo_fr, servo_fl, esc_throttle,
            bat_x100,
            alt_x10,
            flags,
            flight_mode & 0xFF
        )
        crc = calculate_crc16(payload_without_crc)
        return payload_without_crc + struct.pack("<H", crc)

    if packet_format in ("61B_GPS", "GPS"):
        pitch_x10 = int(round(pitch * 10))
        roll_x10 = int(round(roll * 10))
        bat_x100 = int(round(battery_v * 100))
        alt_x10 = int(round(alt * 10))
        lat_e7 = int(round(lat * 1e7))
        lon_e7 = int(round(lon * 1e7))
        gps_alt_x10 = int(round(gps_alt * 10))
        assist_flag = is_assist_mode or flaperon_active or roll_active
        flags = (
            (1 if rc_signal_lost else 0) |
            (2 if assist_flag else 0) |
            (4 if is_low_volt else 0)
        )
        header = b"MT"
        payload_without_crc = struct.pack(
            PACKET_FORMAT_61B_GPS[:-1],
            header,
            pkt_seq & 0xFF,
            timestamp_ms & 0xFFFFFFFF,
            pitch_x10, roll_x10,
            accel_x, accel_y, accel_z,
            gyro_x, gyro_y, gyro_z,
            rc1, rc2, rc3, rc5,
            servo_br, servo_bl, servo_fr, servo_fl, esc_throttle,
            bat_x100,
            alt_x10,
            lat_e7,
            lon_e7,
            gps_alt_x10,
            satellites & 0xFF,
            gps_fix & 0xFF,
            flags,
            0  # reserved
        )
        crc = calculate_crc16(payload_without_crc)
        return payload_without_crc + struct.pack("<H", crc)

    # Default 61-byte format with dynamic PID gains
    pitch_x10 = int(round(pitch * 10))
    roll_x10 = int(round(roll * 10))
    bat_x100 = int(round(battery_v * 100))
    alt_x10 = int(round(alt * 10))
    assist_flag = is_assist_mode or flaperon_active or roll_active
    flags = (
        (1 if rc_signal_lost else 0) |
        (2 if assist_flag else 0) |
        (4 if is_low_volt else 0) |
        (8 if is_esc_active else 0)
    )
    pkp_x100 = int(round(pitch_kp * 100))
    pki_x100 = int(round(pitch_ki * 100))
    pkd_x1000 = int(round(pitch_kd * 1000))
    rkp_x100 = int(round(roll_kp * 100))
    rki_x100 = int(round(roll_ki * 100))
    rkd_x1000 = int(round(roll_kd * 1000))

    header = b"MT"
    payload_without_crc = struct.pack(
        PACKET_FORMAT_PID[:-1],
        header,
        pkt_seq & 0xFF,
        timestamp_ms & 0xFFFFFFFF,
        pitch_x10, roll_x10,
        accel_x, accel_y, accel_z,
        gyro_x, gyro_y, gyro_z,
        rc1, rc2, rc3, rc5,
        servo_br, servo_bl, servo_fr, servo_fl, esc_throttle,
        bat_x100,
        alt_x10,
        flags,
        flight_mode & 0xFF,
        pkp_x100,
        pki_x100,
        pkd_x1000,
        rkp_x100,
        rki_x100,
        rkd_x1000
    )
    crc = calculate_crc16(payload_without_crc)
    return payload_without_crc + struct.pack("<H", crc)


def decode_telemetry(packet_bytes: bytes) -> dict | None:
    if len(packet_bytes) not in SUPPORTED_PACKET_SIZES:
        return None

    header = packet_bytes[:2]
    if header != b"MT":
        return None

    received_crc = STRUCT_CRC.unpack(packet_bytes[-2:])[0]
    computed_crc = calculate_crc16(packet_bytes[:-2])
    if received_crc != computed_crc:
        return None

    if len(packet_bytes) == 61:
        # Check if legacy GPS packet (valid GPS fix/sats with non-zero lat/lon coordinates and zero reserved byte)
        is_gps_packet = False
        if packet_bytes[58] == 0 and packet_bytes[57] <= 15 and packet_bytes[56] <= 2 and packet_bytes[55] <= 32:
            unpacked_gps = STRUCT_61B_GPS.unpack(packet_bytes)
            if unpacked_gps[22] != 0 or unpacked_gps[23] != 0:
                is_gps_packet = True

        if is_gps_packet:
            unpacked = STRUCT_61B_GPS.unpack(packet_bytes)
            pkt_seq = unpacked[1]
            timestamp_ms = unpacked[2]
            pitch = round(unpacked[3] / 10.0, 1)
            roll = round(unpacked[4] / 10.0, 1)
            ax, ay, az = unpacked[5], unpacked[6], unpacked[7]
            gx, gy, gz = unpacked[8], unpacked[9], unpacked[10]
            rc1, rc2, rc3, rc5 = unpacked[11], unpacked[12], unpacked[13], unpacked[14]
            srv_br, srv_bl, srv_fr, srv_fl, esc_throt = (
                unpacked[15], unpacked[16], unpacked[17], unpacked[18], unpacked[19]
            )
            bat_v = round(unpacked[20] / 100.0, 2)
            alt = round(unpacked[21] / 10.0, 1)
            lat_e7 = unpacked[22]
            lon_e7 = unpacked[23]
            lat = round(lat_e7 / 1e7, 7)
            lon = round(lon_e7 / 1e7, 7)
            gps_alt = round(unpacked[24] / 10.0, 1)
            sats = unpacked[25]
            fix_type = unpacked[26]
            flags = unpacked[27]
            reserved = unpacked[28]

            sig_lost = bool(flags & 0x01)
            assist_m = bool(flags & 0x02)
            low_v = bool(flags & 0x04)
            esc_active = bool(flags & 0x08)
            f_mode = reserved if reserved in (1, 2, 3) else decode_ch5_mode(rc5)[0]

            return {
                "pkt_seq": pkt_seq,
                "timestamp_ms": timestamp_ms,
                "pitch": pitch,
                "roll": roll,
                "accel_x": ax,
                "accel_y": ay,
                "accel_z": az,
                "gyro_x": gx,
                "gyro_y": gy,
                "gyro_z": gz,
                "rc": [rc1, rc2, rc3, rc5],
                "rc1": rc1,
                "rc2": rc2,
                "rc3": rc3,
                "rc5": rc5,
                "servo_br": srv_br,
                "servo_bl": srv_bl,
                "servo_fr": srv_fr,
                "servo_fl": srv_fl,
                "esc_throttle": esc_throt,
                "servos": [srv_br, srv_bl, srv_fr, srv_fl, esc_throt],
                "batteryVoltage": bat_v,
                "battery_v": bat_v,
                "alt": alt,
                "lat": lat,
                "latitude": lat,
                "lat_e7": lat_e7,
                "lon": lon,
                "longitude": lon,
                "lon_e7": lon_e7,
                "gps_alt": gps_alt,
                "satellites": sats,
                "sats": sats,
                "fix_type": fix_type,
                "fixType": fix_type,
                "gps_fixed": (fix_type > 0),
                "rcSignalLost": sig_lost,
                "rc_signal_lost": sig_lost,
                "isAssistMode": assist_m,
                "is_assist_mode": assist_m,
                "rollActive": assist_m,
                "roll_active": assist_m,
                "flaperonActive": assist_m,
                "flaperon_active": assist_m,
                "flapsActive": assist_m,
                "flaps_active": assist_m,
                "isLowVolt": low_v,
                "is_low_volt": low_v,
                "isEscActive": esc_active,
                "is_esc_active": esc_active,
                "flightMode": f_mode,
                "flight_mode": f_mode,
                "pitch_kp": 9.35, "pitchKp": 9.35,
                "pitch_ki": 5.00, "pitchKi": 5.00,
                "pitch_kd": 0.623, "pitchKd": 0.623,
                "roll_kp": 15.00, "rollKp": 15.00,
                "roll_ki": 5.00, "rollKi": 5.00,
                "roll_kd": 1.500, "rollKd": 1.500,
                "packet_size": 61
            }

        unpacked = STRUCT_PID.unpack(packet_bytes)
        pkt_seq = unpacked[1]
        timestamp_ms = unpacked[2]
        pitch = round(unpacked[3] / 10.0, 1)
        roll = round(unpacked[4] / 10.0, 1)
        ax, ay, az = unpacked[5], unpacked[6], unpacked[7]
        gx, gy, gz = unpacked[8], unpacked[9], unpacked[10]
        rc1, rc2, rc3, rc5 = unpacked[11], unpacked[12], unpacked[13], unpacked[14]
        srv_br, srv_bl, srv_fr, srv_fl, esc_throt = (
            unpacked[15], unpacked[16], unpacked[17], unpacked[18], unpacked[19]
        )
        bat_v = round(unpacked[20] / 100.0, 2)
        alt = round(unpacked[21] / 10.0, 1)
        flags = unpacked[22]
        f_mode = unpacked[23] if unpacked[23] in (1, 2, 3) else decode_ch5_mode(rc5)[0]
        pitch_kp = round(unpacked[24] / 100.0, 2)
        pitch_ki = round(unpacked[25] / 100.0, 2)
        pitch_kd = round(unpacked[26] / 1000.0, 3)
        roll_kp = round(unpacked[27] / 100.0, 2)
        roll_ki = round(unpacked[28] / 100.0, 2)
        roll_kd = round(unpacked[29] / 1000.0, 3)

        sig_lost = bool(flags & 0x01)
        assist_m = bool(flags & 0x02)
        low_v = bool(flags & 0x04)
        esc_active = bool(flags & 0x08)

        return {
            "pkt_seq": pkt_seq,
            "timestamp_ms": timestamp_ms,
            "pitch": pitch,
            "roll": roll,
            "accel_x": ax,
            "accel_y": ay,
            "accel_z": az,
            "gyro_x": gx,
            "gyro_y": gy,
            "gyro_z": gz,
            "rc": [rc1, rc2, rc3, rc5],
            "rc1": rc1,
            "rc2": rc2,
            "rc3": rc3,
            "rc5": rc5,
            "servo_br": srv_br,
            "servo_bl": srv_bl,
            "servo_fr": srv_fr,
            "servo_fl": srv_fl,
            "esc_throttle": esc_throt,
            "servos": [srv_br, srv_bl, srv_fr, srv_fl, esc_throt],
            "batteryVoltage": bat_v,
            "battery_v": bat_v,
            "alt": alt,
            "lat": 0.0,
            "latitude": 0.0,
            "lat_e7": 0,
            "lon": 0.0,
            "longitude": 0.0,
            "lon_e7": 0,
            "gps_alt": 0.0,
            "satellites": 0,
            "sats": 0,
            "fix_type": 0,
            "fixType": 0,
            "gps_fixed": False,
            "rcSignalLost": sig_lost,
            "rc_signal_lost": sig_lost,
            "isAssistMode": assist_m,
            "is_assist_mode": assist_m,
            "rollActive": assist_m,
            "roll_active": assist_m,
            "flaperonActive": assist_m,
            "flaperon_active": assist_m,
            "flapsActive": assist_m,
            "flaps_active": assist_m,
            "isLowVolt": low_v,
            "is_low_volt": low_v,
            "isEscActive": esc_active,
            "is_esc_active": esc_active,
            "flightMode": f_mode,
            "flight_mode": f_mode,
            "pitch_kp": pitch_kp, "pitchKp": pitch_kp,
            "pitch_ki": pitch_ki, "pitchKi": pitch_ki,
            "pitch_kd": pitch_kd, "pitchKd": pitch_kd,
            "roll_kp": roll_kp, "rollKp": roll_kp,
            "roll_ki": roll_ki, "rollKi": roll_ki,
            "roll_kd": roll_kd, "rollKd": roll_kd,
            "packet_size": 61
        }

    elif len(packet_bytes) == 49:
        unpacked = STRUCT_49B.unpack(packet_bytes)
        pkt_seq = unpacked[1]
        timestamp_ms = unpacked[2]
        pitch = round(unpacked[3] / 10.0, 1)
        roll = round(unpacked[4] / 10.0, 1)
        ax, ay, az = unpacked[5], unpacked[6], unpacked[7]
        gx, gy, gz = unpacked[8], unpacked[9], unpacked[10]
        rc1, rc2, rc3, rc5 = unpacked[11], unpacked[12], unpacked[13], unpacked[14]
        srv_br, srv_bl, srv_fr, srv_fl, esc_throt = (
            unpacked[15], unpacked[16], unpacked[17], unpacked[18], unpacked[19]
        )
        bat_v = round(unpacked[20] / 100.0, 2)
        alt = round(unpacked[21] / 10.0, 1)
        flags = unpacked[22]
        reserved = unpacked[23]

        sig_lost = bool(flags & 0x01)
        assist_m = bool(flags & 0x02)
        low_v = bool(flags & 0x04)
        esc_active = bool(flags & 0x08)
        f_mode = reserved if reserved in (1, 2, 3) else decode_ch5_mode(rc5)[0]

        return {
            "pkt_seq": pkt_seq,
            "timestamp_ms": timestamp_ms,
            "pitch": pitch,
            "roll": roll,
            "accel_x": ax,
            "accel_y": ay,
            "accel_z": az,
            "gyro_x": gx,
            "gyro_y": gy,
            "gyro_z": gz,
            "rc": [rc1, rc2, rc3, rc5],
            "rc1": rc1,
            "rc2": rc2,
            "rc3": rc3,
            "rc5": rc5,
            "servo_br": srv_br,
            "servo_bl": srv_bl,
            "servo_fr": srv_fr,
            "servo_fl": srv_fl,
            "esc_throttle": esc_throt,
            "servos": [srv_br, srv_bl, srv_fr, srv_fl, esc_throt],
            "batteryVoltage": bat_v,
            "battery_v": bat_v,
            "alt": alt,
            "lat": 0.0,
            "latitude": 0.0,
            "lat_e7": 0,
            "lon": 0.0,
            "longitude": 0.0,
            "lon_e7": 0,
            "gps_alt": 0.0,
            "satellites": 0,
            "sats": 0,
            "fix_type": 0,
            "fixType": 0,
            "gps_fixed": False,
            "rcSignalLost": sig_lost,
            "rc_signal_lost": sig_lost,
            "isAssistMode": assist_m,
            "is_assist_mode": assist_m,
            "rollActive": assist_m,
            "roll_active": assist_m,
            "flaperonActive": assist_m,
            "flaperon_active": assist_m,
            "flapsActive": assist_m,
            "flaps_active": assist_m,
            "isLowVolt": low_v,
            "is_low_volt": low_v,
            "isEscActive": esc_active,
            "is_esc_active": esc_active,
            "flightMode": f_mode,
            "flight_mode": f_mode,
            "pitch_kp": 9.35, "pitchKp": 9.35,
            "pitch_ki": 5.00, "pitchKi": 5.00,
            "pitch_kd": 0.623, "pitchKd": 0.623,
            "roll_kp": 15.00, "rollKp": 15.00,
            "roll_ki": 5.00, "rollKi": 5.00,
            "roll_kd": 1.500, "rollKd": 1.500,
            "packet_size": 49
        }

    elif len(packet_bytes) == 33:
        unpacked = STRUCT_4CH.unpack(packet_bytes)
        flags = unpacked[15]
        rc1, rc2, rc3, rc5 = unpacked[9], unpacked[10], unpacked[11], unpacked[12]
        bat_v = round(unpacked[13] / 100.0, 2)
        sig_lost = bool(flags & 0x01)
        return {
            "pkt_seq": 0,
            "timestamp_ms": 0,
            "pitch": round(unpacked[1] / 10.0, 1),
            "roll": round(unpacked[2] / 10.0, 1),
            "accel_x": unpacked[3],
            "accel_y": unpacked[4],
            "accel_z": unpacked[5],
            "gyro_x": unpacked[6],
            "gyro_y": unpacked[7],
            "gyro_z": unpacked[8],
            "rc": [rc1, rc2, rc3, rc5],
            "rc1": rc1, "rc2": rc2, "rc3": rc3, "rc5": rc5,
            "servo_br": 1500, "servo_bl": 1500, "servo_fr": 1500, "servo_fl": 1500, "esc_throttle": 1000,
            "servos": [1500, 1500, 1500, 1500, 1000],
            "batteryVoltage": bat_v,
            "battery_v": bat_v,
            "alt": round(unpacked[14] / 10.0, 1),
            "lat": 0.0, "latitude": 0.0, "lat_e7": 0,
            "lon": 0.0, "longitude": 0.0, "lon_e7": 0,
            "gps_alt": 0.0, "satellites": 0, "sats": 0, "fix_type": 0, "fixType": 0, "gps_fixed": False,
            "rcSignalLost": sig_lost,
            "rc_signal_lost": sig_lost,
            "isAssistMode": False,
            "is_assist_mode": False,
            "isLowVolt": False,
            "is_low_volt": False,
            "isEscActive": False,
            "is_esc_active": False,
            "flightMode": decode_ch5_mode(rc5)[0],
            "flight_mode": decode_ch5_mode(rc5)[0],
            "pitch_kp": 9.35, "pitchKp": 9.35,
            "pitch_ki": 5.00, "pitchKi": 5.00,
            "pitch_kd": 0.623, "pitchKd": 0.623,
            "roll_kp": 15.00, "rollKp": 15.00,
            "roll_ki": 5.00, "rollKi": 5.00,
            "roll_kd": 1.500, "rollKd": 1.500,
            "packet_size": 33
        }
    elif len(packet_bytes) == 31:
        unpacked = STRUCT_3CH.unpack(packet_bytes)
        flags = unpacked[14]
        rc1, rc2, rc3 = unpacked[9], unpacked[10], unpacked[11]
        bat_v = round(unpacked[12] / 100.0, 2)
        sig_lost = bool(flags & 0x01)
        return {
            "pkt_seq": 0,
            "timestamp_ms": 0,
            "pitch": round(unpacked[1] / 10.0, 1),
            "roll": round(unpacked[2] / 10.0, 1),
            "accel_x": unpacked[3],
            "accel_y": unpacked[4],
            "accel_z": unpacked[5],
            "gyro_x": unpacked[6],
            "gyro_y": unpacked[7],
            "gyro_z": unpacked[8],
            "rc": [rc1, rc2, rc3, 1000],
            "rc1": rc1, "rc2": rc2, "rc3": rc3, "rc5": 1000,
            "servo_br": 1500, "servo_bl": 1500, "servo_fr": 1500, "servo_fl": 1500, "esc_throttle": 1000,
            "servos": [1500, 1500, 1500, 1500, 1000],
            "batteryVoltage": bat_v,
            "battery_v": bat_v,
            "alt": round(unpacked[13] / 10.0, 1),
            "lat": 0.0, "latitude": 0.0, "lat_e7": 0,
            "lon": 0.0, "longitude": 0.0, "lon_e7": 0,
            "gps_alt": 0.0, "satellites": 0, "sats": 0, "fix_type": 0, "fixType": 0, "gps_fixed": False,
            "rcSignalLost": sig_lost,
            "rc_signal_lost": sig_lost,
            "isAssistMode": False,
            "is_assist_mode": False,
            "isLowVolt": False,
            "is_low_volt": False,
            "isEscActive": False,
            "is_esc_active": False,
            "flightMode": 1,
            "flight_mode": 1,
            "pitch_kp": 9.35, "pitchKp": 9.35,
            "pitch_ki": 5.00, "pitchKi": 5.00,
            "pitch_kd": 0.623, "pitchKd": 0.623,
            "roll_kp": 15.00, "rollKp": 15.00,
            "roll_ki": 5.00, "rollKi": 5.00,
            "roll_kd": 1.500, "rollKd": 1.500,
            "packet_size": 31
        }
    elif len(packet_bytes) == 35:
        unpacked = STRUCT_5CH.unpack(packet_bytes)
        flags = unpacked[16]
        rc1, rc2, rc3, rc4, rc5 = unpacked[9], unpacked[10], unpacked[11], unpacked[12], unpacked[13]
        bat_v = round(unpacked[14] / 100.0, 2)
        sig_lost = bool(flags & 0x01)
        return {
            "pkt_seq": 0,
            "timestamp_ms": 0,
            "pitch": round(unpacked[1] / 10.0, 1),
            "roll": round(unpacked[2] / 10.0, 1),
            "accel_x": unpacked[3],
            "accel_y": unpacked[4],
            "accel_z": unpacked[5],
            "gyro_x": unpacked[6],
            "gyro_y": unpacked[7],
            "gyro_z": unpacked[8],
            "rc": [rc1, rc2, rc3, rc5],
            "rc1": rc1, "rc2": rc2, "rc3": rc3, "rc4": rc4, "rc5": rc5,
            "servo_br": 1500, "servo_bl": 1500, "servo_fr": 1500, "servo_fl": 1500, "esc_throttle": 1000,
            "servos": [1500, 1500, 1500, 1500, 1000],
            "batteryVoltage": bat_v,
            "battery_v": bat_v,
            "alt": round(unpacked[15] / 10.0, 1),
            "lat": 0.0, "latitude": 0.0, "lat_e7": 0,
            "lon": 0.0, "longitude": 0.0, "lon_e7": 0,
            "gps_alt": 0.0, "satellites": 0, "sats": 0, "fix_type": 0, "fixType": 0, "gps_fixed": False,
            "rcSignalLost": sig_lost,
            "rc_signal_lost": sig_lost,
            "isAssistMode": False,
            "is_assist_mode": False,
            "isLowVolt": False,
            "is_low_volt": False,
            "isEscActive": False,
            "is_esc_active": False,
            "flightMode": decode_ch5_mode(rc5)[0],
            "flight_mode": decode_ch5_mode(rc5)[0],
            "pitch_kp": 9.35, "pitchKp": 9.35,
            "pitch_ki": 5.00, "pitchKi": 5.00,
            "pitch_kd": 0.623, "pitchKd": 0.623,
            "roll_kp": 15.00, "rollKp": 15.00,
            "roll_ki": 5.00, "rollKi": 5.00,
            "roll_kd": 1.500, "rollKd": 1.500,
            "packet_size": 35
        }

