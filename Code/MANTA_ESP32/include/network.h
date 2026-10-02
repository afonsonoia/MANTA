#ifndef NETWORK_H
#define NETWORK_H

#include <Arduino.h>

void initNetwork();
void sendTelemetry(
    uint32_t timestampMs,
    float pitch, float roll,
    int16_t accelX, int16_t accelY, int16_t accelZ,
    int16_t gyroX, int16_t gyroY, int16_t gyroZ,
    uint16_t rch1, uint16_t rch2, uint16_t rch3, uint16_t rch5,
    uint16_t srvBR, uint16_t srvBL, uint16_t srvFR, uint16_t srvFL, uint16_t escThrot,
    float batteryVoltage, float alt,
    int32_t latE7, int32_t lonE7, int16_t gpsAltX10,
    uint8_t satellites, uint8_t fixType,
    bool rcSignalLost,
    bool flaperonActive,
    bool isLowVolt,
    bool isEscActive = false,
    uint8_t flightMode = 1,
    uint8_t failsafeStage = 0,
    float pitchKp = 9.35f, float pitchKi = 5.00f, float pitchKd = 0.623f,
    float rollKp = 15.00f, float rollKi = 5.00f, float rollKd = 1.500f
);
void setLoRaTxPower(uint8_t powerDbm);
uint8_t getLoRaTxPower();

#endif // NETWORK_H
