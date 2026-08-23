#include "bmp280.h"
#include "config.h"
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <math.h>

static Adafruit_BMP280 bmp;
static bool bmpInitialized = false;
static float baselinePressure = -1.0f;
static float currentAltitude = -1.0f;
static float currentPressure = -1.0f;
static float currentTemperature = -1.0f;
static bool firstAltitudeSample = true;

void initBMP280() {
  Wire.begin(PIN_SDA, PIN_SCL, 400000); // Initialize I2C bus at 400kHz
  Wire.setTimeOut(20);                  // 20ms I2C timeout to prevent bus lockup
  delay(10);

  // Probe I2C addresses 0x76 and 0x77 safely before initializing
  uint8_t targetAddr = 0;
  Wire.beginTransmission(0x76);
  if (Wire.endTransmission() == 0) {
    targetAddr = 0x76;
  } else {
    Wire.beginTransmission(0x77);
    if (Wire.endTransmission() == 0) {
      targetAddr = 0x77;
    }
  }

  if (targetAddr != 0 && bmp.begin(targetAddr)) {
    bmpInitialized = true;
  } else {
    bmpInitialized = false;
    currentAltitude = -1.0f;
    currentPressure = -1.0f;
    currentTemperature = -1.0f;
    baselinePressure = -1.0f;
    firstAltitudeSample = true;
    return;
  }

  /* Default settings from datasheet for outdoor flight/drone navigation */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,    /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,    /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,   /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,     /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_63); /* Standby time. */

  // Zero ground-level baseline pressure over 20 readings
  delay(100);
  float sumP = 0.0f;
  int validSamples = 0;
  for (int i = 0; i < 20; i++) {
    float p = bmp.readPressure();
    if (p > 30000.0f && p < 120000.0f) { // Valid pressure between 300hPa and 1200hPa
      sumP += p;
      validSamples++;
    }
    delay(10);
  }
  if (validSamples > 0) {
    baselinePressure = (sumP / validSamples) / 100.0f; // Convert Pa to hPa
  } else {
    baselinePressure = -1.0f;
  }
}

void setBaselinePressure() {
  if (!bmpInitialized)
    return;
  float p = bmp.readPressure();
  if (p > 30000.0f && p < 120000.0f) {
    baselinePressure = p / 100.0f;
  }
}

void sampleBMP280() {
  if (!bmpInitialized) {
    return;
  }

  float pressureHPa = bmp.readPressure() / 100.0f;
  float tempC = bmp.readTemperature();

  if (pressureHPa > 300.0f && pressureHPa < 1200.0f) {
    currentPressure = pressureHPa;
    currentTemperature = tempC;

    // If baseline pressure was not set or invalid, adopt first valid reading
    if (baselinePressure <= 0.0f) {
      baselinePressure = pressureHPa;
    }

    // Barometric Altitude formula: h = 44330 * (1 - (P / P0)^(1 / 5.255))
    float calcAlt =
        44330.0f * (1.0f - pow(pressureHPa / baselinePressure, 0.1903f));

    // First sample assumes calculated altitude directly; subsequent samples are smoothed via EMA
    if (firstAltitudeSample) {
      currentAltitude = calcAlt;
      firstAltitudeSample = false;
    } else {
      currentAltitude = (0.2f * calcAlt) + (0.8f * currentAltitude);
    }
  }
}

void getBaroData(float &altitude, float &pressure, float &temperature) {
  altitude = currentAltitude;
  pressure = currentPressure;
  temperature = currentTemperature;
}

bool isBMP280Available() { return bmpInitialized; }
