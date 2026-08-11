#include "control.h"
#include "battery.h"
#include "config.h"
#include "network.h"
#include "receiver.h"
#include <ESP32Servo.h>
#include <Preferences.h>

static Servo servoBR;  // Back Right / Traseira Direita (PIN_SERVO_BR / GPIO13)
static Servo servoBL;  // Back Left / Traseira Esquerda (PIN_SERVO_BL / GPIO14)
static Servo servoFR;  // Front Right / Frontal Direita (PIN_SERVO_FR / GPIO27)
static Servo servoFL;  // Front Left / Frontal Esquerda (PIN_SERVO_FL / GPIO26)
static Servo escMotor; // Throttle / ESC (PIN_ESC / GPIO25)

// Last written pulse (us) per output channel for 4-unit deadband filter (0:BR,
// 1:BL, 2:FR, 3:FL, 4:ESC)
static int lastWritePulseUs[5] = {1500, 1500, 1500, 1500, 1000};
static uint16_t currentServoUpdateInterval = DEFAULT_SERVO_MIN_UPDATE_INTERVAL_MS;

void setServoUpdateInterval(uint16_t intervalMs) {
  if (intervalMs < 10) intervalMs = 10;
  if (intervalMs > 200) intervalMs = 200;
  currentServoUpdateInterval = intervalMs;
}

uint16_t getServoUpdateInterval() {
  return currentServoUpdateInterval;
}

static void updateOutputChannel(uint8_t index, Servo &srv, int targetPulseUs,
                                int deadband) {
  targetPulseUs = constrain(targetPulseUs, 1000, 2000);
  int diff = targetPulseUs - lastWritePulseUs[index];
  if (diff > deadband || diff < -deadband) {
    srv.writeMicroseconds(targetPulseUs);
    lastWritePulseUs[index] = targetPulseUs;
  }
}

// Mathematically exact Linear Affine Interpolation:
static inline int mapRangeLinear(int val, int inMin, int inMax, int outMin,
                                 int outMax) {
  if (val <= inMin)
    return outMin;
  if (val >= inMax)
    return outMax;
  return outMin +
         (int)(((long)(val - inMin) * (outMax - outMin)) / (inMax - inMin));
}

// Calibrated Stick Neutral Center Points & Servo Range Parameters
static uint16_t centerCH1 = 1500;
static uint16_t centerCH2 = 1500;
static uint8_t servoMaxAngleDeg = DEFAULT_SERVO_MAX_ANGLE_DEG; // Default 30 deg
static int16_t trimBR = 0;
static int16_t trimBL = 0;
static int16_t trimFR = 0;
static int16_t trimFL = 0;

static bool invBR = false;
static bool invBL = false;
static bool invFR = false;
static bool invFL = false;

static bool calibModeActive = false;
static Preferences prefs;

bool isCalibrationModeActive() { return calibModeActive; }

uint8_t getServoMaxAngle() { return servoMaxAngleDeg; }

void setServoMaxAngle(uint8_t angleDeg) {
  if (angleDeg < 10)
    angleDeg = 10;
  if (angleDeg > 45)
    angleDeg = 45;
  if (servoMaxAngleDeg != angleDeg) {
    uint8_t oldVal = servoMaxAngleDeg;
    servoMaxAngleDeg = angleDeg;
    Serial.printf(
        "[CONFIG] Changed variable SERVO_MAX_ANGLE: [%d deg] -> [%d deg]\n",
        oldVal, servoMaxAngleDeg);
  }
}

void getServoTrims(int16_t &br, int16_t &bl, int16_t &fr, int16_t &fl) {
  br = trimBR;
  bl = trimBL;
  fr = trimFR;
  fl = trimFL;
}

void setServoTrims(int16_t br, int16_t bl, int16_t fr, int16_t fl) {
  int16_t newBR = constrain(br, -250, 250);
  int16_t newBL = constrain(bl, -250, 250);
  int16_t newFR = constrain(fr, -250, 250);
  int16_t newFL = constrain(fl, -250, 250);
  if (trimBR != newBR || trimBL != newBL || trimFR != newFR ||
      trimFL != newFL) {
    Serial.printf("[CONFIG] Changed variable SERVO_TRIMS (BR,BL,FR,FL): "
                  "[[%d,%d,%d,%d] us] -> [[%d,%d,%d,%d] us]\n",
                  trimBR, trimBL, trimFR, trimFL, newBR, newBL, newFR, newFL);
    trimBR = newBR;
    trimBL = newBL;
    trimFR = newFR;
    trimFL = newFL;
  }
}

void getServoInversion(bool &br, bool &bl, bool &fr, bool &fl) {
  br = invBR;
  bl = invBL;
  fr = invFR;
  fl = invFL;
}

void setServoInversion(bool br, bool bl, bool fr, bool fl) {
  if (invBR != br || invBL != bl || invFR != fr || invFL != fl) {
    Serial.printf("[CONFIG] Changed variable SERVO_INVERSION (BR,BL,FR,FL): "
                  "[[%d,%d,%d,%d]] -> [[%d,%d,%d,%d]]\n",
                  invBR, invBL, invFR, invFL, br, bl, fr, fl);
    invBR = br;
    invBL = bl;
    invFR = fr;
    invFL = fl;
  }
}

void saveCalibrationToNVS() {
  prefs.begin("manta_calib", false);
  prefs.putUShort("centerCH1", centerCH1);
  prefs.putUShort("centerCH2", centerCH2);
  prefs.putUChar("maxAngle", servoMaxAngleDeg);
  prefs.putShort("trimBR", trimBR);
  prefs.putShort("trimBL", trimBL);
  prefs.putShort("trimFR", trimFR);
  prefs.putShort("trimFL", trimFL);
  prefs.putBool("invBR", invBR);
  prefs.putBool("invBL", invBL);
  prefs.putBool("invFR", invFR);
  prefs.putBool("invFL", invFL);
  prefs.putUChar("deadband", getRCMarginDeadband());
  prefs.putUChar("loraPower", getLoRaTxPower());
  prefs.putUShort("servoRate", getServoUpdateInterval());

  uint8_t fType = 1;
  uint16_t wSize = 5;
  float alpha = 0.33f;
  getRCFilterConfig(fType, wSize, alpha);
  prefs.putUChar("fltType", fType);
  prefs.putUShort("fltWindow", wSize);
  prefs.putFloat("fltAlpha", alpha);

  prefs.end();
  Serial.printf("[NVS] SAVED ALL CALIBRATION: CH1=%d, CH2=%d, Angle=%d deg, "
                "Trims=[%d,%d,%d,%d], Invert=[%d,%d,%d,%d], Deadband=%dus, "
                "LoRaPower=%ddBm, ServoRate=%dms, RCFilter=[Type:%d, Win:%d, Alpha:%.2f]\n",
                centerCH1, centerCH2, servoMaxAngleDeg, trimBR, trimBL, trimFR,
                trimFL, invBR, invBL, invFR, invFL, getRCMarginDeadband(),
                getLoRaTxPower(), getServoUpdateInterval(), fType, wSize, alpha);
}

void loadNeutralCalibration() {
  prefs.begin("manta_calib", true);
  centerCH1 = prefs.getUShort("centerCH1", 1500);
  centerCH2 = prefs.getUShort("centerCH2", 1500);
  servoMaxAngleDeg = prefs.getUChar("maxAngle", DEFAULT_SERVO_MAX_ANGLE_DEG);
  trimBR = prefs.getShort("trimBR", 0);
  trimBL = prefs.getShort("trimBL", 0);
  trimFR = prefs.getShort("trimFR", 0);
  trimFL = prefs.getShort("trimFL", 0);
  invBR = prefs.getBool("invBR", false);
  invBL = prefs.getBool("invBL", false);
  invFR = prefs.getBool("invFR", false);
  invFL = prefs.getBool("invFL", false);
  setRCMarginDeadband(prefs.getUChar("deadband", DEFAULT_RC_MARGIN_DEADBAND));
  setLoRaTxPower(prefs.getUChar("loraPower", 17));
  setServoUpdateInterval(
      prefs.getUShort("servoRate", DEFAULT_SERVO_MIN_UPDATE_INTERVAL_MS));

  uint8_t fType = prefs.getUChar("fltType", 1);
  uint16_t wSize = prefs.getUShort("fltWindow", 5);
  float alpha = prefs.getFloat("fltAlpha", 0.33f);
  setRCFilterConfig(fType, wSize, alpha);

  prefs.end();
  Serial.printf("[NVS] Loaded Calibration: CH1=%d, CH2=%d, MaxAngle=%d deg, "
                "Trims=[%d,%d,%d,%d], Invert=[%d,%d,%d,%d], Deadband=%dus, "
                "LoRaPower=%ddBm, ServoRate=%dms, RCFilter=[Type:%d, Win:%d, Alpha:%.2f]\n",
                centerCH1, centerCH2, servoMaxAngleDeg, trimBR, trimBL, trimFR,
                trimFL, invBR, invBL, invFR, invFL, getRCMarginDeadband(),
                getLoRaTxPower(), getServoUpdateInterval(), fType, wSize, alpha);
}

void calibrateNeutralCenters() {
  uint16_t ch1 = 1500, ch2 = 1500, ch3 = 1000, ch4 = 1500, ch5 = 1500;
  getReceiverChannels(ch1, ch2, ch3, ch4, ch5);

  uint16_t oldCH1 = centerCH1;
  uint16_t oldCH2 = centerCH2;

  centerCH1 = ch1;
  centerCH2 = ch2;

  Serial.printf("[CONFIG] Changed variable NEUTRAL_CENTERS: CH1 [%d us] -> [%d "
                "us], CH2 [%d us] -> [%d us]\n",
                oldCH1, centerCH1, oldCH2, centerCH2);

  saveCalibrationToNVS();
}

static void controlTaskLoop(void *parameter) {
  Serial.println(
      "[CONTROL TASK] Core 0 Dedicated Real-Time Control Task running!");

  while (true) {
    // ── STEP 1: Wait dynamic interval then read RC channels (raw ISR values, clamped 1000-2000)
    vTaskDelay(pdMS_TO_TICKS(currentServoUpdateInterval));

    uint16_t ch1 = 0, ch2 = 0, ch3 = 0, ch4 = 0, ch5 = 0;
    getReceiverChannels(ch1, ch2, ch3, ch4, ch5);

    unsigned long nowMs = millis();

    // ── STEP 2: RC margin deadband from persistent config
    int servDeadband = (int)getRCMarginDeadband();

    // ── STEP 3: Safety state machine ─────────────────────────────────────────
    bool ch5High = (ch5 > 1900);
    bool throttleOff = (ch3 < 1050);

    if (!calibModeActive) {
      if (ch5High && throttleOff) {
        calibModeActive = true;
        Serial.println("[SAFETY LOG] Calibration Mode & Throttle Lock "
                       "ACTIVATED (CH5 > 1900 & Throttle < 1050)");
      }
    } else {
      if (!ch5High) {
        calibModeActive = false;
        Serial.println(
            "[SAFETY LOG] Calibration Mode DEACTIVATED (CH5 <= 1900)");
      }
    }

    // RC Failsafe: >500ms without RC pulse (getReceiverChannels returns 0s)
    bool rcSignalLost = isRCSignalLost();
    if (rcSignalLost) {
      static unsigned long lastFailsafeWarnMs = 0;
      if (nowMs - lastFailsafeWarnMs > 3000) {
        lastFailsafeWarnMs = nowMs;
        Serial.println("[FAILSAFE ALERT] RC Transmitter Signal Lost (> 500ms)! "
                       "Cutting throttle & centering surfaces.");
      }
      ch1 = centerCH1;
      ch2 = centerCH2;
    }

    // ── STEP 4: Throttle
    int targetThrottle;
    if (calibModeActive || rcSignalLost) {
      targetThrottle = THROTTLE_MIN_PULSE;
    } else {
      targetThrottle =
          mapRangeLinear((int)ch3, THROTTLE_INPUT_MIN_US, THROTTLE_INPUT_MAX_US,
                         THROTTLE_OUTPUT_MIN_US, THROTTLE_OUTPUT_MAX_US);
      if (isLowVoltageCutoffTriggered())
        targetThrottle = THROTTLE_MIN_PULSE;
    }

    // ── STEP 5: Direct Surface Control (Rear Elevators & Front Rollerons) ────
    int anglePulseLimit = (int)(servoMaxAngleDeg * US_PER_DEGREE);
    int minServoPulse = 1500 - anglePulseLimit;
    int maxServoPulse = 1500 + anglePulseLimit;

    int rollDiff = (int)ch1 - (int)centerCH1;
    int pitchDiff = (int)ch2 - (int)centerCH2;

    // BR + BL (Rear Elevators)  = PITCH (CH2)
    // FR + FL (Front Rollerons)  = ROLL (CH1)
    int pitchOffsetBR = pitchDiff;
    if (invBR) {
      pitchOffsetBR = -pitchDiff;
    }

    int pitchOffsetBL = pitchDiff;
    if (invBL) {
      pitchOffsetBL = -pitchDiff;
    }

    int rollOffsetFR = rollDiff;
    if (invFR) {
      rollOffsetFR = -rollDiff;
    }

    int rollOffsetFL = -rollDiff;
    if (invFL) {
      rollOffsetFL = rollDiff;
    }

    int targetBR = constrain(1500 + pitchOffsetBR + trimBR, minServoPulse, maxServoPulse);
    int targetBL = constrain(1500 + pitchOffsetBL + trimBL, minServoPulse, maxServoPulse);
    int targetFR = constrain(1500 + rollOffsetFR + trimFR, minServoPulse, maxServoPulse);
    int targetFL = constrain(1500 + rollOffsetFL + trimFL, minServoPulse, maxServoPulse);

    // ── STEP 6: Output — deadband applied (full for servos, half for throttle)
    updateOutputChannel(0, servoBR, targetBR, servDeadband);
    updateOutputChannel(1, servoBL, targetBL, servDeadband);
    updateOutputChannel(2, servoFR, targetFR, servDeadband);
    updateOutputChannel(3, servoFL, targetFL, servDeadband);
    updateOutputChannel(4, escMotor, targetThrottle, max(1, servDeadband / 2));

    // ── STEP 7: Serial diagnostics at 2Hz ────────────────────────────────────
    static unsigned long lastDebugMs = 0;
    if (nowMs - lastDebugMs >= 500) {
      lastDebugMs = nowMs;
      Serial.printf("[RC IN]  CH1=%d | CH2=%d | CH3=%d | CH5=%d | Deadband=%dus%s\n",
                    ch1, ch2, ch3, ch5, servDeadband, rcSignalLost ? " [SIGNAL LOST]" : "");
      Serial.printf("[SERVO]  BR=%d | BL=%d | FR=%d | FL=%d | ESC=%d (Calib=%d)\n",
                    targetBR, targetBL, targetFR, targetFL, targetThrottle, calibModeActive);
    }
  }
}

bool setThrottlePulse(int pulseWidthUs) {
  if (isLowVoltageCutoffTriggered()) {
    emergencyCutoffESC();
    return false;
  }

  if (pulseWidthUs >= THROTTLE_MIN_PULSE &&
      pulseWidthUs <= THROTTLE_MAX_PULSE) {
    if (pulseWidthUs != lastWritePulseUs[4]) {
      escMotor.writeMicroseconds(pulseWidthUs);
      lastWritePulseUs[4] = pulseWidthUs;
    }
    return true;
  }
  return false;
}

void emergencyCutoffESC() {
  if (lastWritePulseUs[4] != THROTTLE_MIN_PULSE) {
    escMotor.writeMicroseconds(THROTTLE_MIN_PULSE);
    lastWritePulseUs[4] = THROTTLE_MIN_PULSE;
  }
}

int getCurrentThrottlePulse() { return lastWritePulseUs[4]; }

void initControlSystem() {
  loadNeutralCalibration();

  // Allow allocation of all timers for ESP32Servo (Required for > 4 LEDC
  // channels!)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servoBR.setPeriodHertz(50);
  servoBL.setPeriodHertz(50);
  servoFR.setPeriodHertz(50);
  servoFL.setPeriodHertz(50);
  escMotor.setPeriodHertz(50);

  servoBR.attach(PIN_SERVO_BR, 1000, 2000);
  servoBL.attach(PIN_SERVO_BL, 1000, 2000);
  servoFR.attach(PIN_SERVO_FR, 1000, 2000);
  servoFL.attach(PIN_SERVO_FL, 1000, 2000);
  escMotor.attach(PIN_ESC, THROTTLE_MIN_PULSE, THROTTLE_MAX_PULSE);

  servoBR.writeMicroseconds(1500);
  servoBL.writeMicroseconds(1500);
  servoFR.writeMicroseconds(1500);
  servoFL.writeMicroseconds(1500);
  escMotor.writeMicroseconds(THROTTLE_MIN_PULSE);

  // Spawn FreeRTOS Control Task pinned exclusively to Core 0 (Priority 5)
  xTaskCreatePinnedToCore(controlTaskLoop, "ControlTask", 4096, NULL, 5, NULL,
                          0 // Core 0 execution
  );

  Serial.println("[CONTROL SYSTEM] V-Tail & Thermal Protection Control System "
                 "initialized on Core 0!");
}
