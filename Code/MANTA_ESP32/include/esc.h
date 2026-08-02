#ifndef ESC_H
#define ESC_H

#include <Arduino.h>

void initESC();
bool setThrottlePulse(int pulseWidthUs);
void emergencyCutoffESC();
int getCurrentThrottlePulse();

#endif // ESC_H
