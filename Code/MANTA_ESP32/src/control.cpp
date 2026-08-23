#include "control.h"
#include "battery.h"
#include "config.h"
#include "mpu6050.h"
#include "network.h"
#include "receiver.h"
#include <ESP32Servo.h>
#include <Preferences.h>

static Servo servoBR;  // Back Right (PIN_SERVO_BR / GPIO13)
static Servo servoBL;  // Back Left (PIN_SERVO_BL / GPIO14)
static Servo servoFR;  // Front Right (PIN_SERVO_FR / GPIO27)
static Servo servoFL;  // Front Left (PIN_SERVO_FL / GPIO26)
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

static Preferences prefs;

uint8_t getServoMaxAngle() { return servoMaxAngleDeg; }

void setServoMaxAngle(uint8_t angleDeg) {
  if (angleDeg < 10)
    angleDeg = 10;
  if (angleDeg > 45)
    angleDeg = 45;
  if (servoMaxAngleDeg != angleDeg) {
    servoMaxAngleDeg = angleDeg;
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
    invBR = br;
    invBL = bl;
    invFR = fr;
    invFL = fl;
  }
}

static void controlTaskLoop(void *parameter) {
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(currentServoUpdateInterval));

    uint16_t ch1 = 0, ch2 = 0, ch3 = 0, ch5 = 0;
    getReceiverChannels(ch1, ch2, ch3, ch5);

    unsigned long nowMs = millis();

    // ── STEP 2: RC margin deadband from persistent config
    int receiverDeadband = (int)getRCMarginDeadband();

    // RC Failsafe: >500ms without RC pulse (getReceiverChannels returns 0s)
    bool rcSignalLost = isRCSignalLost();
    if (rcSignalLost) {
      ch1 = centerCH1;
      ch2 = centerCH2;
      ch5 = 1000;
    }

    // ── STEP 4: Throttle
    int targetThrottle;
    if (rcSignalLost) {
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

    int pitchDiff = (int)ch2 - (int)centerCH2;

    // ── ROLL CONTROL / FLIGHT ENVELOPE ASSIST (ACTIVE ONLY WHEN CH5 == 2000) ──
    int rollDiff = 0;
    if (ch5 > 1500) {
      // Assist Mode Active: Smart Roll Limitation & Auto-Recovery
      float curPitch = 0.0f, curRoll = 0.0f;
      getFilteredMPUData(curPitch, curRoll);

      float pilotCmdDeg = constrain(((float)((int)ch1 - (int)centerCH1) / 500.0f) * 20.0f, -20.0f, 20.0f);
      float absRoll = fabsf(curRoll);
      float effectiveRollCmdDeg = 0.0f;

      if (absRoll <= 60.0f) {
        // Linear reduction of authority in bank direction: 20 deg @ 0 -> 0 deg @ 60
        float maxIntoBank = 20.0f * (1.0f - (absRoll / 60.0f));

        if (curRoll >= 0.0f) {
          // Banked Right (+): Right turn capped at +maxIntoBank, Left (recovery) allowed up to -20
          effectiveRollCmdDeg = constrain(pilotCmdDeg, -20.0f, maxIntoBank);
        } else {
          // Banked Left (-): Left turn capped at -maxIntoBank, Right (recovery) allowed up to +20
          effectiveRollCmdDeg = constrain(pilotCmdDeg, -maxIntoBank, 20.0f);
        }
      } else {
        // Over-banked (> 60 deg): Auto-recovery of 5 deg towards level.
        // Pilot can command stronger recovery, but commands into bank are overridden.
        if (curRoll >= 0.0f) {
          // Banked Right (> 60 deg): apply at least -5 deg (LEFT roll).
          effectiveRollCmdDeg = constrain(pilotCmdDeg, -20.0f, -5.0f);
        } else {
          // Banked Left (> 60 deg): apply at least +5 deg (RIGHT roll).
          effectiveRollCmdDeg = constrain(pilotCmdDeg, 5.0f, 20.0f);
        }
      }

      // Convert effective degrees back to pulse offset (20 deg = 500us scale)
      rollDiff = (int)((effectiveRollCmdDeg / 20.0f) * 500.0f);
    } else {
      // Manual Direct Mode (CH5 == 1000)
      rollDiff = (int)ch1 - (int)centerCH1;
    }

    // BR + BL (Rear Elevators)  = PITCH (CH2)
    // FR + FL (Front Rollerons)  = ROLL (CH1)
    int pitchOffsetBR = pitchDiff;
    if (invBR) {
      pitchOffsetBR = -pitchDiff;
    }

    int pitchOffsetBL = -pitchDiff; // Inverted Back Left Elevator (BL)
    if (invBL) {
      pitchOffsetBL = pitchDiff;
    }

    int rollOffsetFR = -rollDiff; // Inverted Front Right Rolleron (FR)
    if (invFR) {
      rollOffsetFR = rollDiff;
    }

    int rollOffsetFL = -rollDiff;
    if (invFL) {
      rollOffsetFL = rollDiff;
    }

    int targetBR = constrain(1500 + pitchOffsetBR + trimBR, minServoPulse, maxServoPulse);
    int targetBL = constrain(1500 + pitchOffsetBL + trimBL, minServoPulse, maxServoPulse);
    int targetFR = constrain(1500 + rollOffsetFR + trimFR, minServoPulse, maxServoPulse);
    int targetFL = constrain(1500 + rollOffsetFL + trimFL, minServoPulse, maxServoPulse);

    // ── STEP 6: Output — uniform deadband applied to all channels (Servos + ESC)
    updateOutputChannel(0, servoBR, targetBR, receiverDeadband);
    updateOutputChannel(1, servoBL, targetBL, receiverDeadband);
    updateOutputChannel(2, servoFR, targetFR, receiverDeadband);
    updateOutputChannel(3, servoFL, targetFL, receiverDeadband);
    updateOutputChannel(4, escMotor, targetThrottle, receiverDeadband);

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
}
