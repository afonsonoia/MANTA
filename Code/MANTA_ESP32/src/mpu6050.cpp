#include "mpu6050.h"
#include "config.h"
#include <Wire.h>
#include <math.h>

#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define CONFIG_REG   0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43

static float currentPitch = 0.0f;
static float currentRoll = 0.0f;
static float currentYaw = 0.0f;

// Calibrated Gyro Static Offsets (measured on startup to eliminate drift)
static float gyroBiasX = 0.0f;
static float gyroBiasY = 0.0f;
static float gyroBiasZ = 0.0f;

static unsigned long lastSampleTimeUs = 0;
static bool mpuInitialized = false;
static bool initialAttitudeSet = false;

static void writeMPURegister(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void initMPU6050() {
  Wire.begin(PIN_SDA, PIN_SCL, 400000); // Initialize I2C bus at 400kHz
  Wire.setTimeOut(50); // 50ms I2C timeout to prevent bus lockup
  delay(100);

  // Wake up MPU6050
  writeMPURegister(PWR_MGMT_1, 0x00);
  delay(10);

  // Set Digital Low Pass Filter (DLPF) ~42Hz bandwidth for motor/vibration noise rejection (CONFIG_REG = 0x03)
  writeMPURegister(CONFIG_REG, 0x03);

  // Gyro full scale range +/- 250 deg/s (131 LSB / deg/s)
  writeMPURegister(GYRO_CONFIG, 0x00);

  // Accel full scale range +/- 2g (16384 LSB / g)
  writeMPURegister(ACCEL_CONFIG, 0x00);

  // Verify I2C communication
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() == 0) {
    mpuInitialized = true;
    Serial.println("[MPU6050] Sensor initialized successfully on I2C!");

    // Automatic Gyro Zero-Bias Offset Calibration over 100 samples
    long sumGx = 0, sumGy = 0, sumGz = 0;
    int samples = 100;
    int validCount = 0;
    for (int i = 0; i < samples; i++) {
      Wire.beginTransmission(MPU6050_ADDR);
      Wire.write(GYRO_XOUT_H);
      if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)6) == 6) {
        int16_t rawGx = (int16_t)((Wire.read() << 8) | Wire.read());
        int16_t rawGy = (int16_t)((Wire.read() << 8) | Wire.read());
        int16_t rawGz = (int16_t)((Wire.read() << 8) | Wire.read());
        sumGx += rawGx;
        sumGy += rawGy;
        sumGz += rawGz;
        validCount++;
      }
      delay(2);
    }
    if (validCount > 0) {
      gyroBiasX = (float)sumGx / (float)validCount;
      gyroBiasY = (float)sumGy / (float)validCount;
      gyroBiasZ = (float)sumGz / (float)validCount;
    }
    // Sanity check: clamp gyro bias to +/- 2000 LSB (~15 deg/s) to prevent startup corruption
    gyroBiasX = constrain(gyroBiasX, -2000.0f, 2000.0f);
    gyroBiasY = constrain(gyroBiasY, -2000.0f, 2000.0f);
    gyroBiasZ = constrain(gyroBiasZ, -2000.0f, 2000.0f);

    Serial.printf("[MPU6050 Calibration] Gyro Bias Offsets (LSB): X=%.1f Y=%.1f Z=%.1f\n", gyroBiasX, gyroBiasY, gyroBiasZ);
  } else {
    Serial.println("[MPU6050] WARNING: Sensor not detected on I2C bus!");
  }

  initialAttitudeSet = false;
  lastSampleTimeUs = 0;
}

void sampleMPU6050Uniformly() {
  if (!mpuInitialized) {
    Wire.beginTransmission(MPU6050_ADDR);
    if (Wire.endTransmission() == 0) {
      initMPU6050();
    } else {
      return;
    }
  }

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return;

  uint8_t bytesReceived = Wire.requestFrom((int)MPU6050_ADDR, (int)14, (int)true);
  if (bytesReceived < 14) return;

  unsigned long nowUs = micros();
  if (lastSampleTimeUs == 0) lastSampleTimeUs = nowUs;
  float dt = (nowUs - lastSampleTimeUs) / 1000000.0f;
  lastSampleTimeUs = nowUs;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.01f; // Safeguard dt to 10ms

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // Skip temperature bytes
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  // Convert raw readings and apply calibrated zero-bias gyro offset
  float accelX = ax / 16384.0f;
  float accelY = ay / 16384.0f;
  float accelZ = az / 16384.0f;

  float gyroX = (gx - gyroBiasX) / 131.0f; // deg/s
  float gyroY = (gy - gyroBiasY) / 131.0f; // deg/s
  float gyroZ = (gz - gyroBiasZ) / 131.0f; // deg/s

  // Correctly aligned Pitch & Roll from accelerometer:
  // Pitch (around Y axis) uses longitudinal accelX: atan2(-accelX, sqrt(accelY^2 + accelZ^2))
  // Roll (around X axis) uses lateral accelY: atan2(accelY, sqrt(accelX^2 + accelZ^2))
  float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * (180.0f / M_PI);
  float accelRoll  = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * (180.0f / M_PI);

  // Initialize attitude directly to accelerometer angles on first valid reading
  if (!initialAttitudeSet) {
    currentPitch = accelPitch;
    currentRoll  = accelRoll;
    initialAttitudeSet = true;
  }

  // Smooth Complementary Filter (98% Gyro + 2% Accel)
  float alpha = 0.98f;
  currentPitch = alpha * (currentPitch + gyroY * dt) + (1.0f - alpha) * accelPitch;
  currentRoll  = alpha * (currentRoll  + gyroX * dt) + (1.0f - alpha) * accelRoll;
  currentYaw  += gyroZ * dt;
}


void getFilteredMPUData(float &pitch, float &roll, float &yaw) {
  pitch = currentPitch;
  roll  = currentRoll;
  yaw   = currentYaw;
}
