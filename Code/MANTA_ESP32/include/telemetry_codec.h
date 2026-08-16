#ifndef TELEMETRY_CODEC_H
#define TELEMETRY_CODEC_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#pragma pack(push, 1)
typedef struct {
    uint8_t header[2];      // Magic Header: {'M', 'T'} (0x4D, 0x54)
    int16_t pitch_x10;     // Estimated Pitch * 10 (deg)
    int16_t roll_x10;      // Estimated Roll * 10 (deg)
    int16_t accel_x;       // Raw Accel X LSB
    int16_t accel_y;       // Raw Accel Y LSB
    int16_t accel_z;       // Raw Accel Z LSB
    int16_t gyro_x;        // Raw Gyro X LSB
    int16_t gyro_y;        // Raw Gyro Y LSB
    int16_t gyro_z;        // Raw Gyro Z LSB
    uint16_t rc[4];        // RC PWM channels: CH1 (Roll), CH2 (Pitch), CH3 (Throttle), CH5 (Switch)
    uint16_t bat_v_x100;   // Battery Voltage * 100
    int16_t alt_x10;       // Altitude * 10 (m)
    uint8_t rc_sig_lost;   // RC signal lost flag
    uint16_t crc16;        // CRC16-MODBUS checksum over preceding 31 bytes
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
    float pitch, float roll,
    int16_t accelX, int16_t accelY, int16_t accelZ,
    int16_t gyroX, int16_t gyroY, int16_t gyroZ,
    uint16_t rch1, uint16_t rch2, uint16_t rch3, uint16_t rch5,
    float batteryVoltage, float alt,
    uint8_t flags
) {
    pkt->header[0] = 'M';
    pkt->header[1] = 'T';
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
    pkt->bat_v_x100 = (uint16_t)(batteryVoltage * 100.0f + 0.5f);
    pkt->alt_x10 = (int16_t)(alt * 10.0f + (alt >= 0 ? 0.5f : -0.5f));
    pkt->rc_sig_lost = flags;

    // CRC16 calculated over first 29 bytes
    pkt->crc16 = calculate_telemetry_crc16((const uint8_t *)pkt, offsetof(MantaTelemetryPacket, crc16));
}

#endif // TELEMETRY_CODEC_H
