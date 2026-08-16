#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>

void initControlSystem();
void setServoMaxAngle(uint8_t angleDeg);
uint8_t getServoMaxAngle();
void setServoTrims(int16_t br, int16_t bl, int16_t fr, int16_t fl);
void getServoTrims(int16_t &br, int16_t &bl, int16_t &fr, int16_t &fl);
void setServoInversion(bool br, bool bl, bool fr, bool fl);
void getServoInversion(bool &br, bool &bl, bool &fr, bool &fl);
void setServoUpdateInterval(uint16_t intervalMs);
uint16_t getServoUpdateInterval();

bool setThrottlePulse(int pulseWidthUs);
void emergencyCutoffESC();
int getCurrentThrottlePulse();

#endif // CONTROL_H
