#include "battery.h"
#include "config.h"

static volatile bool lowVoltageCutoffTriggered = false;
static float latestRawAdc = 0.0f;
static float latestBatteryVoltage = 0.0f;
static float configuredCutoffVoltage = DEFAULT_CUTOFF_VOLTAGE;
static uint8_t lowVoltConsecutiveHits = 0;

void initBatterySensor() {
  pinMode(PIN_BATTERY, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_BATTERY, ADC_11db);
  // Perform initial measurement on startup
  sampleBatteryUniformly();

  // Auto-detect LiPo cell count on boot: > 13.0V is 4S, <= 13.0V is 3S
  if (latestBatteryVoltage > 13.0f) {
    configuredCutoffVoltage = CUTOFF_VOLTAGE_4S;
  } else if (latestBatteryVoltage > 6.0f) {
    configuredCutoffVoltage = CUTOFF_VOLTAGE_3S;
  }
}

float readInstantaneousRawADC() {
  long sum = 0;
  for (int i = 0; i < 64; i++) {
    sum += analogRead(PIN_BATTERY);
    delayMicroseconds(100);
  }
  return (float)sum / 64.0f;
}

void sampleBatteryUniformly() {
  float raw = readInstantaneousRawADC();
  latestRawAdc = raw;
  latestBatteryVoltage = calculateBatteryVoltage(raw);
  checkLowVoltageSafety(latestBatteryVoltage);
}

float getAndResetAverageADC() {
  return latestRawAdc;
}

float getLatestBatteryVoltage() {
  return latestBatteryVoltage;
}


float calculateBatteryVoltage(float rawInput) {
  float voltage =
      -0.000000884f * rawInput * rawInput + 0.008835f * rawInput - 5.6904f;
  if (voltage < 0.0f) {
    voltage = 0.0f;
  }
  return voltage;
}

void setCutoffThreshold(float targetVoltage) {
  if (targetVoltage < ABSOLUTE_MIN_CUTOFF_VOLTAGE) {
    targetVoltage = ABSOLUTE_MIN_CUTOFF_VOLTAGE;
  }
  if (configuredCutoffVoltage != targetVoltage) {
    configuredCutoffVoltage = targetVoltage;
  }
}

float getEffectiveCutoffThreshold() {
  return max(ABSOLUTE_MIN_CUTOFF_VOLTAGE, configuredCutoffVoltage);
}

bool isLowVoltageCutoffTriggered() { return lowVoltageCutoffTriggered; }

bool checkLowVoltageSafety(float currentVoltage) {
  float effectiveCutoff = getEffectiveCutoffThreshold();
  if (currentVoltage <= effectiveCutoff && currentVoltage > 6.0f) {
    if (lowVoltConsecutiveHits < LOW_VOLT_CONFIRM_COUNT) {
      lowVoltConsecutiveHits++;
    }
    if (lowVoltConsecutiveHits >= LOW_VOLT_CONFIRM_COUNT) {
      lowVoltageCutoffTriggered = true;
    }
  } else if (currentVoltage > (effectiveCutoff + 0.5f)) {
    // Voltage recovered (e.g. throttle eased or fresh battery connected): reset cutoff trigger
    lowVoltConsecutiveHits = 0;
    lowVoltageCutoffTriggered = false;
  }
  return lowVoltageCutoffTriggered;
}
