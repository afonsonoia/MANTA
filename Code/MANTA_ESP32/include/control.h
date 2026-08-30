#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>

void initControlSystem();
bool setThrottlePulse(int pulseWidthUs);
void emergencyCutoffESC();
int getCurrentThrottlePulse();

#endif // CONTROL_H
