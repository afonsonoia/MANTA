#include "mpu6050.h"
#include "config.h"
#include <Wire.h>
#include <math.h>

#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define CONFIG 0x1A
#define SMPLRT_DIV 0x19
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43

// Mahony AHRS Filter Parameters (Ultra-smooth Gyro Response + Gentle Anti-Drift
// Integral Bias Tracking)
constexpr float MAHONY_KP =
    0.40f; // Previous proportional gain (ultra-smooth gyro-dominated response)
constexpr float MAHONY_KI =
    0.01f; // Increased integral gain to softly eliminate slow thermal drift

// Pure 4D Quaternion Orientation state q = (q0, q1, q2, q3)
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
// Integral error accumulator for online gyro bias compensation
static float eIntX = 0.0f, eIntY = 0.0f, eIntZ = 0.0f;

static float currentPitch = 0.0f;
static float currentRoll = 0.0f;
static float currentYaw = 0.0f;

// Calibrated Gyro Static Offsets (in raw LSB)
static float gyroBiasX = 0.0f;
static float gyroBiasY = 0.0f;
static float gyroBiasZ = 0.0f;

static unsigned long lastSampleTimeUs = 0;
static bool mpuInitialized = false;
static bool initialAttitudeSet = false;

static int16_t lastRawAx = 0, lastRawAy = 0, lastRawAz = 0;
static int16_t lastRawGx = 0, lastRawGy = 0, lastRawGz = 0;

static bool writeMPURegister(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  return (Wire.endTransmission() == 0);
}

void initMPU6050() {
  Wire.begin(PIN_SDA, PIN_SCL, 400000); // 400 kHz Fast-Mode I2C
  Wire.setTimeOut(50);

  // Early wakeup to allow internal sensor clock to stabilize
  writeMPURegister(PWR_MGMT_1, 0x00);
  delay(15);

  // Device reset (bit 7 = 1)
  writeMPURegister(PWR_MGMT_1, 0x80);
  delay(100);

  // Wake up MPU6050 and select PLL with X-axis gyroscope reference
  writeMPURegister(PWR_MGMT_1, 0x01);
  delay(15);

  // Sample Rate Divider: 1kHz / (1 + 0) = 1kHz internal sample rate
  writeMPURegister(SMPLRT_DIV, 0x00);

  // Digital Low Pass Filter: 98Hz Accel / 98Hz Gyro (matching proven stable
  // temp_imu_test)
  writeMPURegister(CONFIG, 0x02);

  // Gyro full scale range +/- 1000 deg/s (32.8 LSB / deg/s) prevents saturation
  // during snappy hand rotations
  writeMPURegister(GYRO_CONFIG, 0x10);

  // Accel full scale range +/- 2g (16384 LSB / g) for maximum gravity accuracy
  writeMPURegister(ACCEL_CONFIG, 0x00);

  // Probe MPU6050 presence
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() != 0) {
    mpuInitialized = false;
    currentPitch = 0.0f;
    currentRoll = 0.0f;
    currentYaw = 0.0f;
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    initialAttitudeSet = false;
    lastSampleTimeUs = 0;
    Serial.println("[MPU6050 WARNING] IMU not detected on I2C bus! Continuing "
                   "flight loop.");
    return;
  }

  mpuInitialized = true;
  delay(100); // Allow sensor clock & regulator to settle before calibration

  // Automatic Zero-Bias Offset Calibration over 500 samples (1.0s)
  long sumGx = 0, sumGy = 0, sumGz = 0;
  int validCount = 0;
  for (int i = 0; i < 500; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(GYRO_XOUT_H);
    if (Wire.endTransmission(false) == 0 &&
        Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)6) == 6) {
      uint8_t b[6];
      for (int k = 0; k < 6; k++)
        b[k] = Wire.read();
      int16_t gx = (int16_t)((b[0] << 8) | b[1]);
      int16_t gy = (int16_t)((b[2] << 8) | b[3]);
      int16_t gz = (int16_t)((b[4] << 8) | b[5]);
      sumGx += gx;
      sumGy += gy;
      sumGz += gz;
      validCount++;
    }
    delay(2);
  }
  if (validCount > 0) {
    gyroBiasX = (float)sumGx / (float)validCount;
    gyroBiasY = (float)sumGy / (float)validCount;
    gyroBiasZ = (float)sumGz / (float)validCount;
  }
  gyroBiasX = constrain(gyroBiasX, -2000.0f, 2000.0f);
  gyroBiasY = constrain(gyroBiasY, -2000.0f, 2000.0f);
  gyroBiasZ = constrain(gyroBiasZ, -2000.0f, 2000.0f);

  q0 = 1.0f;
  q1 = 0.0f;
  q2 = 0.0f;
  q3 = 0.0f;
  eIntX = 0.0f;
  eIntY = 0.0f;
  eIntZ = 0.0f;
  initialAttitudeSet = false;
  lastSampleTimeUs = 0;
  Serial.printf("[MPU6050 Mahony AHRS] Initialized (+/-250 deg/s, 98Hz DLPF) | "
                "Gyro Bias: X=%.1f Y=%.1f Z=%.1f\n",
                gyroBiasX, gyroBiasY, gyroBiasZ);
}

void sampleMPU6050Uniformly() {
  static unsigned long lastMpuReconnectCheck = 0;
  static uint8_t consecutiveI2CErrors = 0;

  if (!mpuInitialized) {
    unsigned long now = millis();
    if (now - lastMpuReconnectCheck >= 3000) {
      lastMpuReconnectCheck = now;
      Wire.beginTransmission(MPU6050_ADDR);
      if (Wire.endTransmission() == 0) {
        initMPU6050();
        consecutiveI2CErrors = 0;
      }
    }
    return;
  }

  // 1. Blocking register select with hardware timeout
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) {
    consecutiveI2CErrors++;
    if (consecutiveI2CErrors >= 10) {
      mpuInitialized = false;
    }
    return;
  }

  // 2. Burst read all 14 sensor registers
  uint8_t bytesReceived =
      Wire.requestFrom((int)MPU6050_ADDR, (int)14, (int)true);
  if (bytesReceived < 14) {
    consecutiveI2CErrors++;
    if (consecutiveI2CErrors >= 10) {
      mpuInitialized = false;
    }
    return;
  }
  consecutiveI2CErrors = 0;

  uint8_t buffer[14];
  for (int i = 0; i < 14; i++) {
    buffer[i] = Wire.read();
  }

  // 3. Ultra-accurate time delta calculation
  unsigned long nowUs = micros();
  if (lastSampleTimeUs == 0) {
    lastSampleTimeUs = nowUs;
    return;
  }
  float dt = (nowUs - lastSampleTimeUs) / 1000000.0f;
  lastSampleTimeUs = nowUs;

  // Smoothly constrain time delta to guarantee continuous gyro integration
  dt = constrain(dt, 0.001f, 0.040f);

  int16_t ax = (int16_t)((buffer[0] << 8) | buffer[1]);
  int16_t ay = (int16_t)((buffer[2] << 8) | buffer[3]);
  int16_t az = (int16_t)((buffer[4] << 8) | buffer[5]);
  int16_t gx = (int16_t)((buffer[8] << 8) | buffer[9]);
  int16_t gy = (int16_t)((buffer[10] << 8) | buffer[11]);
  int16_t gz = (int16_t)((buffer[12] << 8) | buffer[13]);

  lastRawAx = ax;
  lastRawAy = ay;
  lastRawAz = az;
  lastRawGx = gx;
  lastRawGy = gy;
  lastRawGz = gz;

  // Normalize accelerometer reading (in units of 1g = 16384 LSB)
  float accelX = ax / 16384.0f;
  float accelY = ay / 16384.0f;
  float accelZ = -az / 16384.0f; // Invert Z so flat upright resting gives +1.0g
                                 // matching (0,0,1) unit gravity vector

  float norm = sqrtf(accelX * accelX + accelY * accelY + accelZ * accelZ);
  if (norm < 0.2f || norm > 3.0f || isnan(norm)) {
    accelX = 0.0f;
    accelY = 0.0f;
    accelZ = 0.0f;
  } else {
    accelX /= norm;
    accelY /= norm;
    accelZ /= norm;
  }

  // Convert raw gyro to rad/s (32.8 LSB / (deg/s) for +/- 1000 deg/s range)
  // Gyro X (Pitch) and Gyro Y (Roll) are inverted (-1.0f) to match physical
  // airframe axes on MANTA PCB
  float gx_rad = -1.0f * ((gx - gyroBiasX) / 32.8f) * DEG_TO_RAD;
  float gy_rad = -1.0f * ((gy - gyroBiasY) / 32.8f) * DEG_TO_RAD;
  float gz_rad = ((gz - gyroBiasZ) / 32.8f) * DEG_TO_RAD;

  // Initialize quaternion state directly from gravity vector on first valid
  // reading
  if (!initialAttitudeSet && norm > 0.5f) {
    float initPitch = atan2f(-accelX, sqrtf(accelY * accelY + accelZ * accelZ));
    float initRoll = atan2f(accelY, sqrtf(accelX * accelX + accelZ * accelZ));

    float cp = cosf(initPitch * 0.5f);
    float sp = sinf(initPitch * 0.5f);
    float cr = cosf(initRoll * 0.5f);
    float sr = sinf(initRoll * 0.5f);

    q0 = cr * cp;
    q1 = sr * cp;
    q2 = cr * sp;
    q3 = -sr * sp;

    float qInitNorm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (qInitNorm > 0.0f) {
      float invInit = 1.0f / qInitNorm;
      q0 *= invInit;
      q1 *= invInit;
      q2 *= invInit;
      q3 *= invInit;
    }
    initialAttitudeSet = true;
  }

  // ── MAHONY 6-AXIS SENSOR FUSION WITH SMOOTH ADAPTIVE WEIGHTING ───────
  float ex = 0.0f, ey = 0.0f, ez = 0.0f;
  if (!(accelX == 0.0f && accelY == 0.0f && accelZ == 0.0f)) {
    // Estimated gravity direction vector in body frame from quaternion q
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // Cross product between measured gravity (a) and estimated gravity (v)
    // gives 3D alignment error
    ex = (accelY * vz - accelZ * vy);
    ey = (accelZ * vx - accelX * vz);
    ez = (accelX * vy - accelY * vx);

    // Adaptive Kp weighting based on how close total accel magnitude is to 1.0g
    // (smooth fade between 0.6g and 1.4g)
    float accelDev = fabsf(norm - 1.0f);
    float kpWeight =
        (accelDev < 0.10f)
            ? 1.0f
            : (accelDev > 0.40f ? 0.0f : (0.40f - accelDev) / 0.30f);

    // Accumulate integral error with anti-windup clamping to guarantee zero
    // drift
    eIntX = constrain(eIntX + (ex * MAHONY_KI * dt * kpWeight), -0.05f, 0.05f);
    eIntY = constrain(eIntY + (ey * MAHONY_KI * dt * kpWeight), -0.05f, 0.05f);
    eIntZ = constrain(eIntZ + (ez * MAHONY_KI * dt * kpWeight), -0.05f, 0.05f);

    // Apply Proportional + Integral feedback to gyroscope rates
    gx_rad += (MAHONY_KP * ex * kpWeight) + eIntX;
    gy_rad += (MAHONY_KP * ey * kpWeight) + eIntY;
    gz_rad += (MAHONY_KP * ez * kpWeight) + eIntZ;
  }

  // Integrate quaternion rate of change: q_dot = 0.5 * q x omega
  float q0Last = q0, q1Last = q1, q2Last = q2, q3Last = q3;
  q0 += 0.5f * (-q1Last * gx_rad - q2Last * gy_rad - q3Last * gz_rad) * dt;
  q1 += 0.5f * (q0Last * gx_rad + q2Last * gz_rad - q3Last * gy_rad) * dt;
  q2 += 0.5f * (q0Last * gy_rad - q1Last * gz_rad + q3Last * gx_rad) * dt;
  q3 += 0.5f * (q0Last * gz_rad + q1Last * gy_rad - q2Last * gx_rad) * dt;

  // Robust Quaternion Normalization with Anti-Crash Protection
  float qNormSq = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
  if (isnan(qNormSq) || isinf(qNormSq) || qNormSq < 1e-6f) {
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
  } else {
    float invQNorm = 1.0f / sqrtf(qNormSq);
    q0 *= invQNorm;
    q1 *= invQNorm;
    q2 *= invQNorm;
    q3 *= invQNorm;
  }

  // Extract 3D Aerospace Euler Angles (in degrees) matching MANTA airframe
  // orientation:
  float sinp = 2.0f * (q0 * q1 + q2 * q3);
  sinp = constrain(sinp, -1.0f, 1.0f);
  currentPitch = (asinf(sinp) * (180.0f / M_PI)) -
                 9.2f; // Calibrated level mounting trim (-9.2 deg total, subtracted 3.7 deg)
  currentRoll = (-1.0f *
                 atan2f(2.0f * (q0 * q2 - q1 * q3),
                        q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) *
                 (180.0f / M_PI)) -
                1.9f; // Calibrated level mounting trim (-1.9 deg total, subtracted 0.9 deg)
  currentYaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                      q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) *
               (180.0f / M_PI);
}

void getFilteredMPUData(float &pitch, float &roll) {
  pitch = currentPitch;
  roll = currentRoll;
}

void getFilteredMPUData(float &pitch, float &roll, float &yaw) {
  pitch = currentPitch;
  roll = currentRoll;
  yaw = currentYaw;
}

void getFilteredMPUQuaternion(float &outQ0, float &outQ1, float &outQ2,
                              float &outQ3) {
  outQ0 = q0;
  outQ1 = q1;
  outQ2 = q2;
  outQ3 = q3;
}

bool isMPU6050Available() { return mpuInitialized; }

void getRawMPUData(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx,
                   int16_t &gy, int16_t &gz) {
  ax = lastRawAx;
  ay = lastRawAy;
  az = lastRawAz;
  gx = lastRawGx;
  gy = lastRawGy;
  gz = lastRawGz;
}

void getRawGyroData(int16_t &gx, int16_t &gy, int16_t &gz) {
  gx = lastRawGx;
  gy = lastRawGy;
  gz = lastRawGz;
}
