#include "battery.h"
#include "config.h"

static bool lowVoltageCutoffTriggered = false;
static double accumulatedAdcSum = 0.0;
static long sampleCount = 0;

void initBatterySensor() {
  pinMode(PIN_BATTERY, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_BATTERY, ADC_11db);
  accumulatedAdcSum = 0.0;
  sampleCount = 0;
}

float readInstantaneousRawADC() {
  long sum = 0;
  for (int i = 0; i < ADC_OVERSAMPLE_PER_TICK; i++) {
    sum += analogRead(PIN_BATTERY);
    delayMicroseconds(50);
  }
  return (float)sum / (float)ADC_OVERSAMPLE_PER_TICK;
}

void sampleBatteryUniformly() {
  float raw = readInstantaneousRawADC();
  accumulatedAdcSum += raw;
  sampleCount++;

  // Continuous low-voltage protection check on every sample tick
  float currentVoltage = calculateBatteryVoltage(raw);
  checkLowVoltageSafety(currentVoltage);
}

float getAndResetAverageADC() {
  if (sampleCount == 0) {
    return readInstantaneousRawADC();
  }
  float avg = (float)(accumulatedAdcSum / (double)sampleCount);
  accumulatedAdcSum = 0.0;
  sampleCount = 0;
  return avg;
}

float calculateBatteryVoltage(float rawInput) {
  float voltage =
      -0.000000884f * rawInput * rawInput + 0.008835f * rawInput - 5.6904f;
  if (voltage < 0.0f) {
    voltage = 0.0f;
  }
  return voltage;
}

bool isLowVoltageCutoffTriggered() { return lowVoltageCutoffTriggered; }

bool checkLowVoltageSafety(float currentVoltage) {
  if (currentVoltage <= MIN_CUTOFF_VOLTAGE && currentVoltage > 0.0f) {
    if (!lowVoltageCutoffTriggered) {
      lowVoltageCutoffTriggered = true;
    }
  }
  return lowVoltageCutoffTriggered;
}
