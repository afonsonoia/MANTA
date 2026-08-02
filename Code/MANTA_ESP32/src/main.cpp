#include "battery.h"
#include "config.h"
#include "esc.h"
#include "network.h"
#include <Arduino.h>

static unsigned long lastSampleMillis = 0;
static unsigned long lastLoggingMillis = 0;

void setup() {
  Serial.begin(115200);
  initBatterySensor();
  initESC();
  initNetwork();
}

void loop() {
  // Handle incoming commands over LoRa
  handleNetworkCommands();

  unsigned long currentMillis = millis();

  // 1. Uniform sampling every 100ms (100 samples across the 10s window)
  if (currentMillis - lastSampleMillis >= SAMPLE_INTERVAL_MS) {
    lastSampleMillis = currentMillis;
    sampleBatteryUniformly();

    // Instant safety evaluation on every sample
    if (isLowVoltageCutoffTriggered()) {
      emergencyCutoffESC();
    }
  }

  // 2. Broadcast averaged telemetry every 10s over LoRa
  if (currentMillis - lastLoggingMillis >= LOGGING_INTERVAL_MS) {
    lastLoggingMillis = currentMillis;

    float avgADC = getAndResetAverageADC();
    float batteryVoltage = calculateBatteryVoltage(avgADC);
    sendTelemetry(avgADC, batteryVoltage);
  }
}
