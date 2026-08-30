#include "battery.h"
#include "bmp280.h"
#include "config.h"
#include "control.h"
#include "mpu6050.h"
#include "network.h"
#include "receiver.h"
#include <WiFi.h>

static unsigned long lastSampleMillis = 0;
static unsigned long lastLoggingMillis = 0;
static unsigned long lastBaroSampleMillis = 0;
static unsigned long lastBatterySampleMillis = 0;

void setup() {
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH); // Turn LED ON immediately on boot
  delay(400);
  digitalWrite(2, LOW);

  // Disable unused Wi-Fi & Bluetooth radios to minimize power consumption
  WiFi.mode(WIFI_OFF);
  btStop();

  initBatterySensor();
  initMPU6050();
  initBMP280();
  initReceiver();
  initControlSystem();
  initNetwork();
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. High frequency uniform IMU sampling every 10ms (100 Hz sampling rate)
  if (currentMillis - lastSampleMillis >= SAMPLE_INTERVAL_MS) {
    lastSampleMillis = currentMillis;
    sampleMPU6050Uniformly();

    // Instant safety evaluation on every sample tick
    if (isLowVoltageCutoffTriggered()) {
      emergencyCutoffESC();
    }
  }

  // Sample Battery Voltage every 1000ms (1 Hz) on independent non-blocking timer
  if (currentMillis - lastBatterySampleMillis >= 1000) {
    lastBatterySampleMillis = currentMillis;
    sampleBatteryUniformly();
  }

  // Sample BMP280 barometer every 1000ms (1 Hz) on independent timer
  if (currentMillis - lastBaroSampleMillis >= 1000) {
    lastBaroSampleMillis = currentMillis;
    sampleBMP280();
  }

  // 2. Broadcast telemetry at 20 Hz (every 50ms)
  if (currentMillis - lastLoggingMillis >= LOGGING_INTERVAL_MS) {
    lastLoggingMillis = currentMillis;

    float avgADC = getAndResetAverageADC();
    float batteryVoltage = calculateBatteryVoltage(avgADC);

    float pitch = 0.0f, roll = 0.0f;
    getFilteredMPUData(pitch, roll);

    int16_t accelX = 0, accelY = 0, accelZ = 0;
    int16_t gyroX = 0, gyroY = 0, gyroZ = 0;
    getRawMPUData(accelX, accelY, accelZ, gyroX, gyroY, gyroZ);

    float baroAlt = 0.0f, baroPressure = 0.0f, baroTemp = 0.0f;
    getBaroData(baroAlt, baroPressure, baroTemp);

    // RC channels: 0 means no signal received yet from the ISR
    uint16_t rch1 = 0, rch2 = 0, rch3 = 0, rch5 = 0;
    getReceiverChannels(rch1, rch2, rch3, rch5);

    bool rcLost = isRCSignalLost();

    sendTelemetry(pitch, roll, accelX, accelY, accelZ, gyroX, gyroY, gyroZ, rch1, rch2, rch3, rch5, batteryVoltage, baroAlt, rcLost, false);
  }

  // 3. Heartbeat LED pulse (500ms in Flight Mode)
  static unsigned long lastHeartbeatMillis = 0;
  static bool heartbeatState = false;
  if (currentMillis - lastHeartbeatMillis >= 500) {
    lastHeartbeatMillis = currentMillis;
    heartbeatState = !heartbeatState;
    digitalWrite(2, heartbeatState ? HIGH : LOW);
  }
}
