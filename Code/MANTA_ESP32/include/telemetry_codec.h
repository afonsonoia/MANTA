#ifndef TELEMETRY_CODEC_H
#define TELEMETRY_CODEC_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#pragma pack(push, 1)
typedef struct {
    uint8_t header[2];      // Magic Header: {'M', 'T'} (0x4D, 0x54)
    uint16_t bat_v_x100;   // Battery Voltage * 100
    uint16_t bat_adc_x10;  // Raw ADC * 10
    int16_t pitch_x10;     // Pitch * 10 (deg)
    int16_t roll_x10;      // Roll * 10 (deg)
    uint16_t yaw_x10;      // Yaw * 10 (deg)
    uint16_t cutoff_x100;  // Cutoff Voltage * 100
    uint8_t deadband;      // Deadband in us
    int32_t lat_x1e7;      // Latitude * 1e7
    int32_t lon_x1e7;      // Longitude * 1e7
    int16_t alt_x10;       // Altitude * 10 (m)
    int16_t temp_x10;      // Temperature * 10 (deg C)
    uint8_t sats;          // Satellites count
    uint8_t fix;           // GPS Fix type
    uint16_t rc[5];        // RC PWM channels 1 to 5
    uint8_t rc_sig_lost;   // RC signal lost flag (0 or 1)
    uint16_t crc16;        // CRC16-MODBUS checksum over preceding 40 bytes
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
    float batteryVoltage, float rawADC,
    float pitch, float roll, float yaw,
    float effectiveCutoff, uint8_t deadband,
    double lat, double lon, float alt, float temp,
    int sats, int fix,
    uint16_t rch1, uint16_t rch2, uint16_t rch3, uint16_t rch4, uint16_t rch5,
    uint8_t flags
) {
    pkt->header[0] = 'M';
    pkt->header[1] = 'T';
    pkt->bat_v_x100 = (uint16_t)(batteryVoltage * 100.0f + 0.5f);
    pkt->bat_adc_x10 = (uint16_t)(rawADC * 10.0f + 0.5f);
    pkt->pitch_x10 = (int16_t)(pitch * 10.0f + (pitch >= 0 ? 0.5f : -0.5f));
    pkt->roll_x10 = (int16_t)(roll * 10.0f + (roll >= 0 ? 0.5f : -0.5f));
    float normYaw = (yaw >= 0.0f) ? fmodf(yaw, 360.0f) : fmodf(fmodf(yaw, 360.0f) + 360.0f, 360.0f);
    pkt->yaw_x10 = (uint16_t)(normYaw * 10.0f + 0.5f);
    pkt->cutoff_x100 = (uint16_t)(effectiveCutoff * 100.0f + 0.5f);
    pkt->deadband = deadband;
    pkt->lat_x1e7 = (int32_t)(lat * 1e7 + (lat >= 0 ? 0.5 : -0.5));
    pkt->lon_x1e7 = (int32_t)(lon * 1e7 + (lon >= 0 ? 0.5 : -0.5));
    pkt->alt_x10 = (int16_t)(alt * 10.0f + (alt >= 0 ? 0.5f : -0.5f));
    pkt->temp_x10 = (int16_t)(temp * 10.0f + (temp >= 0 ? 0.5f : -0.5f));
    pkt->sats = (uint8_t)sats;
    pkt->fix = (uint8_t)fix;
    pkt->rc[0] = rch1;
    pkt->rc[1] = rch2;
    pkt->rc[2] = rch3;
    pkt->rc[3] = rch4;
    pkt->rc[4] = rch5;
    pkt->rc_sig_lost = flags;
    
    // Compute CRC16 over the first 40 bytes (excluding crc16 field itself)
    pkt->crc16 = calculate_telemetry_crc16((const uint8_t*)pkt, offsetof(MantaTelemetryPacket, crc16));
}

#endif // TELEMETRY_CODEC_H
