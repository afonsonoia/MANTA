#include "esc.h"
#include "battery.h"
#include "config.h"
#include <ESP32Servo.h>

static Servo myESC;
static int currentThrottlePulse = THROTTLE_MIN_PULSE;

void initESC() {
  myESC.attach(PIN_ESC, THROTTLE_MIN_PULSE, THROTTLE_MAX_PULSE);
  myESC.writeMicroseconds(THROTTLE_MIN_PULSE);
  currentThrottlePulse = THROTTLE_MIN_PULSE;
}

bool setThrottlePulse(int pulseWidthUs) {
  if (isLowVoltageCutoffTriggered()) {
    emergencyCutoffESC();
    return false;
  }

  if (pulseWidthUs >= THROTTLE_MIN_PULSE &&
      pulseWidthUs <= THROTTLE_MAX_PULSE) {
    if (pulseWidthUs != currentThrottlePulse) {
      currentThrottlePulse = pulseWidthUs;
      myESC.writeMicroseconds(currentThrottlePulse);
    }
    return true;
  }
  return false;
}

void emergencyCutoffESC() {
  if (currentThrottlePulse != THROTTLE_MIN_PULSE) {
    currentThrottlePulse = THROTTLE_MIN_PULSE;
    myESC.writeMicroseconds(THROTTLE_MIN_PULSE);
  }
}

int getCurrentThrottlePulse() { return currentThrottlePulse; }
