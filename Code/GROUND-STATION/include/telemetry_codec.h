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
    int16_t gyro_x;         // Raw Gyro X LSB (p - roll rate)
    int16_t gyro_y;         // Raw Gyro Y LSB (q - pitch rate)
    int16_t gyro_z;         // Raw Gyro Z LSB (r - yaw rate)
    uint16_t rc[4];         // RC Inputs: CH1 (Roll), CH2 (Pitch), CH3 (Throttle), CH5 (Mode)
    uint16_t servo_br;      // Back Right Servo PWM (us)
    uint16_t servo_bl;      // Back Left Servo PWM (us)
    uint16_t servo_fr;      // Front Right Servo PWM (us)
    uint16_t servo_fl;      // Front Left Servo PWM (us)
    uint16_t esc_throttle;  // ESC Throttle Output PWM (us)
    uint16_t bat_v_x100;    // Battery Voltage * 100
    int16_t alt_x10;        // Altitude Barometric * 10 (m)
    uint8_t flags;          // Bit 0: RC Lost, Bit 1: Roll Assist Active, Bit 2: Low Volt Cutoff, Bit 3: ESC Active
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

static inline uint16_t calculate_telemetry_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i];
        for (uint8_t b = 0; b < 8; ++b) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
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
