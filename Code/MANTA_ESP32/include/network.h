#ifndef NETWORK_H
#define NETWORK_H

#include <Arduino.h>

void initNetwork();
void handleNetworkCommands();
void sendTelemetry(float rawADC, float batteryVoltage, float pitch, float roll, float yaw, float effectiveCutoff, double lat = 0.0, double lon = 0.0, float alt = 0.0f, float temp = 25.0f, int sats = 0, int fix = 0, uint16_t rch1 = 0, uint16_t rch2 = 0, uint16_t rch3 = 0, uint16_t rch4 = 0, uint16_t rch5 = 0, bool rcSignalLost = false);
void setLoRaTxPower(uint8_t powerDbm);
uint8_t getLoRaTxPower();


#endif // NETWORK_H
