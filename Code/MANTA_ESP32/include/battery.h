#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>

void initBatterySensor();
float readInstantaneousRawADC();
void sampleBatteryUniformly();
float getAndResetAverageADC();
float calculateBatteryVoltage(float rawInput);
void setCutoffThreshold(float targetVoltage);
float getEffectiveCutoffThreshold();
bool isLowVoltageCutoffTriggered();
bool checkLowVoltageSafety(float currentVoltage);

#endif // BATTERY_H
