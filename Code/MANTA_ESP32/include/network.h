#ifndef NETWORK_H
#define NETWORK_H

#include <Arduino.h>

void initNetwork();
void handleNetworkCommands();
void sendTelemetry(
    float pitch, float roll,
    int16_t accelX, int16_t accelY, int16_t accelZ,
    int16_t gyroX, int16_t gyroY, int16_t gyroZ,
    uint16_t rch1, uint16_t rch2, uint16_t rch3, uint16_t rch5,
    float batteryVoltage, float alt,
    bool rcSignalLost
);
void setLoRaTxPower(uint8_t powerDbm);
uint8_t getLoRaTxPower();

#endif // NETWORK_H
