#ifndef TELEMETRY_CODEC_H
#define TELEMETRY_CODEC_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#pragma pack(push, 1)
typedef struct {
    uint8_t header[2];      // Magic Header: {'M', 'T'} (0x4D, 0x54)
    uint8_t pkt_seq;        // Sequence counter (0..255) for jitter/drop tracking
    uint32_t timestamp_ms;  // ESP32 onboard clock millis()
    int16_t pitch_x10;      // Estimated Pitch * 10 (deg)
    int16_t roll_x10;       // Estimated Roll * 10 (deg)
    int16_t accel_x;        // Raw Accel X LSB
    int16_t accel_y;        // Raw Accel Y LSB
    int16_t accel_z;        // Raw Accel Z LSB
    int16_t gyro_x;         // Raw Gyro X LSB (sensor X axis: pitch rate on -90° rotated MANTA PCB)
    int16_t gyro_y;         // Raw Gyro Y LSB (sensor Y axis: roll rate on -90° rotated MANTA PCB)
    int16_t gyro_z;         // Raw Gyro Z LSB (sensor Z axis: yaw rate)
    uint16_t rc[4];         // RC Inputs: CH1 (Roll), CH2 (Pitch), CH3 (Throttle), CH5 (Mode)
    uint16_t servo_br;      // Back Right Servo PWM (us)
    uint16_t servo_bl;      // Back Left Servo PWM (us)
    uint16_t servo_fr;      // Front Right Servo PWM (us)
    uint16_t servo_fl;      // Front Left Servo PWM (us)
    uint16_t esc_throttle;  // ESC Throttle Output PWM (us)
    uint16_t bat_v_x100;    // Battery Voltage * 100
    int16_t alt_x10;        // Altitude Barometric * 10 (m)
    uint8_t flags;          // Bit 0: RC Lost, Bit 1: Flaperons Active (Flaps ON), Bit 2: Low Volt Cutoff, Bit 3: ESC Active
    uint8_t flight_mode;    // Flight Mode: 1 (Manual), 2 (Fixed FBW), 3 (Adaptive ES PI-D)
    uint16_t pitch_kp_x100; // Pitch Kp * 100
    uint16_t pitch_ki_x100; // Pitch Ki * 100
    uint16_t pitch_kd_x1000;// Pitch Kd * 1000
    uint16_t roll_kp_x100;  // Roll Kp * 100
    uint16_t roll_ki_x100;  // Roll Ki * 100
    uint16_t roll_kd_x1000; // Roll Kd * 1000
    uint16_t crc16;         // CRC16-MODBUS checksum over preceding 59 bytes
} MantaTelemetryPacket;
#pragma pack(pop)

static const uint16_t CRC16_TABLE[256] = {
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
};

static inline uint16_t calculate_telemetry_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc = (crc >> 8) ^ CRC16_TABLE[(crc ^ data[i]) & 0xFF];
    }
    return crc;
}

static inline void encode_telemetry_packet(
    MantaTelemetryPacket *pkt,
    uint8_t seq,
    uint32_t timestampMs,
    float pitch, float roll,
    int16_t accelX, int16_t accelY, int16_t accelZ,
    int16_t gyroX, int16_t gyroY, int16_t gyroZ,
    uint16_t rch1, uint16_t rch2, uint16_t rch3, uint16_t rch5,
    uint16_t srvBR, uint16_t srvBL, uint16_t srvFR, uint16_t srvFL, uint16_t escThrot,
    float batteryVoltage, float alt,
    uint8_t flags,
    uint8_t flightMode = 1,
    float pitchKp = 9.35f, float pitchKi = 5.00f, float pitchKd = 0.623f,
    float rollKp = 15.00f, float rollKi = 5.00f, float rollKd = 1.500f
) {
    pkt->header[0] = 'M';
    pkt->header[1] = 'T';
    pkt->pkt_seq = seq;
    pkt->timestamp_ms = timestampMs;
    pkt->pitch_x10 = (int16_t)(pitch * 10.0f + (pitch >= 0 ? 0.5f : -0.5f));
    pkt->roll_x10 = (int16_t)(roll * 10.0f + (roll >= 0 ? 0.5f : -0.5f));
    pkt->accel_x = accelX;
    pkt->accel_y = accelY;
    pkt->accel_z = accelZ;
    pkt->gyro_x = gyroX;
    pkt->gyro_y = gyroY;
    pkt->gyro_z = gyroZ;
    pkt->rc[0] = rch1;
    pkt->rc[1] = rch2;
    pkt->rc[2] = rch3;
    pkt->rc[3] = rch5;
    pkt->servo_br = srvBR;
    pkt->servo_bl = srvBL;
    pkt->servo_fr = srvFR;
    pkt->servo_fl = srvFL;
    pkt->esc_throttle = escThrot;
    pkt->bat_v_x100 = (uint16_t)(batteryVoltage * 100.0f + 0.5f);
    pkt->alt_x10 = (int16_t)(alt * 10.0f + (alt >= 0 ? 0.5f : -0.5f));
    pkt->flags = flags;
    pkt->flight_mode = flightMode;
    pkt->pitch_kp_x100 = (uint16_t)(pitchKp * 100.0f + 0.5f);
    pkt->pitch_ki_x100 = (uint16_t)(pitchKi * 100.0f + 0.5f);
    pkt->pitch_kd_x1000 = (uint16_t)(pitchKd * 1000.0f + 0.5f);
    pkt->roll_kp_x100 = (uint16_t)(rollKp * 100.0f + 0.5f);
    pkt->roll_ki_x100 = (uint16_t)(rollKi * 100.0f + 0.5f);
    pkt->roll_kd_x1000 = (uint16_t)(rollKd * 1000.0f + 0.5f);

    // CRC16 calculated over first 59 bytes
    pkt->crc16 = calculate_telemetry_crc16((const uint8_t *)pkt, offsetof(MantaTelemetryPacket, crc16));
}

#endif // TELEMETRY_CODEC_H
