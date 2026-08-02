#ifndef NETWORK_H
#define NETWORK_H

#include <Arduino.h>

void initNetwork();
void handleNetworkCommands();
void sendTelemetry(float rawADC, float batteryVoltage);

#endif // NETWORK_H
