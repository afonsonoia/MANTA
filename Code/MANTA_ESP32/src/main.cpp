#include "battery.h"
#include "bmp280.h"
#include "config.h"
#include "control.h"
#include "gps.h"
#include "mpu6050.h"
#include "network.h"
#include "receiver.h"
#include <WiFi.h>

static unsigned long lastSampleMillis = 0;
static unsigned long lastLoggingMillis = 0;
static unsigned long lastBaroSampleMillis = 0;
static unsigned long lastBatterySampleMillis = 0;

void setup() {
  // Disable unused Wi-Fi & Bluetooth radios to minimize power consumption
  WiFi.mode(WIFI_OFF);
  btStop();

  Serial.begin(115200);
  initBatterySensor();
  initMPU6050();
  initBMP280();
  initGPS();
  initReceiver();
  initControlSystem();
  initNetwork();
}


void loop() {
  // Handle incoming commands over LoRa
  handleNetworkCommands();

  // Parse incoming GPS NMEA sentences continuously
  updateGPS();

  unsigned long currentMillis = millis();

  // 1. High frequency uniform IMU sampling every 10ms (100 Hz sampling rate)
  if (currentMillis - lastSampleMillis >= SAMPLE_INTERVAL_MS) {
    lastSampleMillis = currentMillis;
    sampleMPU6050Uniformly();

    // Sample battery sensor every 2000ms (0.5 Hz)
    if (currentMillis - lastBatterySampleMillis >= 2000) {
      lastBatterySampleMillis = currentMillis;
      sampleBatteryUniformly();
    }

    // Sample BMP280 barometer every 1000ms (1 Hz) to eliminate I2C bus contention with MPU6050
    if (currentMillis - lastBaroSampleMillis >= 1000) {
      lastBaroSampleMillis = currentMillis;
      sampleBMP280();
    }

    // Instant safety evaluation on every sample tick
    if (isLowVoltageCutoffTriggered()) {
      emergencyCutoffESC();
    }
  }

  // 2. Broadcast telemetry (4 Hz in flight mode, 0.5 Hz in calibration mode to eliminate RF collisions)
  bool inCalibMode = isCalibrationModeActive();
  unsigned long activeLoggingInterval = inCalibMode ? CALIB_LOGGING_INTERVAL_MS : LOGGING_INTERVAL_MS;

  if (currentMillis - lastLoggingMillis >= activeLoggingInterval) {
    if (isRCQuietPeriod() || (currentMillis - lastLoggingMillis >= (activeLoggingInterval + 50))) {
      lastLoggingMillis = currentMillis;

    float avgADC = getAndResetAverageADC();
    float batteryVoltage = calculateBatteryVoltage(avgADC);

    float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
    getFilteredMPUData(pitch, roll, yaw);

    double lat = 0.0, lon = 0.0;
    float gpsAlt = 0.0f;
    int sats = 0, fix = 0;
    getGPSData(lat, lon, gpsAlt, sats, fix);

    float baroAlt = 0.0f, baroPressure = 0.0f, baroTemp = 0.0f;
    getBaroData(baroAlt, baroPressure, baroTemp);

    // RC channels: 0 means no signal received yet from the ISR
    uint16_t rch1 = 0, rch2 = 0, rch3 = 0, rch4 = 0, rch5 = 0;
    getReceiverChannels(rch1, rch2, rch3, rch4, rch5);

    // Prefer BMP280 high-precision barometric altitude; fallback to GPS altitude
    float finalAlt = isBMP280Available() ? baroAlt : gpsAlt;

    // Autonomous LoRa Tx Power Adaptation:
    // - Failsafe (RC Lost) OR Flight Mode (CH5 < 1900): Maximum 20 dBm (PA_BOOST) for long-range telemetry.
    // - Debug / Calibration Mode on bench (CH5 >= 1900 & RC OK): Low 14 dBm power to save energy & reduce heat.
    bool rcLost = isRCSignalLost();
    if (rcLost || rch5 < 1900) {
      setLoRaTxPower(20);
    } else {
      setLoRaTxPower(14);
    }

    sendTelemetry(avgADC, batteryVoltage, pitch, roll, yaw, getEffectiveCutoffThreshold(), lat, lon, finalAlt, baroTemp, sats, fix, rch1, rch2, rch3, rch4, rch5, rcLost);
    }
  }
}



