#include "control.h"
#include "battery.h"
#include "config.h"
#include "mpu6050.h"
#include "network.h"
#include "receiver.h"
#include <ESP32Servo.h>

static Servo servoBR;  // Back Right (PIN_SERVO_BR / GPIO13)
static Servo servoBL;  // Back Left (PIN_SERVO_BL / GPIO14)
static Servo servoFR;  // Front Right (PIN_SERVO_FR / GPIO27)
static Servo servoFL;  // Front Left (PIN_SERVO_FL / GPIO26)
static Servo escMotor; // Throttle / ESC (PIN_ESC / GPIO25)

static int lastWritePulseUs[5] = {
    1500, 1500, 1500, 1500,
    1000}; // Initialized properly in initControlSystem()
static constexpr uint16_t SERVO_UPDATE_INTERVAL_MS =
    20; // 50 Hz fast servo update

static void updateOutputChannel(uint8_t index, Servo &srv, int targetPulseUs) {
  targetPulseUs = constrain(targetPulseUs, 1000, 2000);
  if (targetPulseUs != lastWritePulseUs[index]) {
    srv.writeMicroseconds(targetPulseUs);
    lastWritePulseUs[index] = targetPulseUs;
  }
}

// Linear Affine Interpolation
static inline int mapRangeLinear(int val, int inMin, int inMax, int outMin,
                                 int outMax) {
  if (val <= inMin)
    return outMin;
  if (val >= inMax)
    return outMax;
  return outMin +
         (int)(((long)(val - inMin) * (outMax - outMin)) / (inMax - inMin));
}

static constexpr uint16_t centerCH1 = 1500;
static constexpr uint16_t centerCH2 = 1500;
static constexpr uint8_t servoMaxAngleDeg =
    DEFAULT_SERVO_MAX_ANGLE_DEG; // Default 30 deg

// ── CALIBRAÇÃO ESTÁTICA DE SUPERFÍCIES DE CONTROLO (TRIM OFFSETS) ───────────
// Fator de conversão: ~11.11us por grau (1000us / 90 deg)
constexpr float TRIM_DEG_BR = 0.0f;
constexpr float TRIM_DEG_BL =
    -10.0f; // -10.0 deg (Soma 111us -> Neutro: 1611 us)
constexpr float TRIM_DEG_FR = 0.0f;
constexpr float TRIM_DEG_FL =
    4.0f; // +4.0 deg UP (Adiciona 44us: rolleron esquerdo para cima)

constexpr int TRIM_US_BR = (int)(TRIM_DEG_BR * US_PER_DEGREE); // 0 us
constexpr int TRIM_US_BL =
    -(int)(TRIM_DEG_BL * US_PER_DEGREE + (TRIM_DEG_BL >= 0 ? 0.5f : -0.5f)); // +111 us -> Neutro: 1611 us
constexpr int TRIM_US_FR = (int)(TRIM_DEG_FR * US_PER_DEGREE); // 0 us
constexpr int TRIM_US_FL =
    +(int)(TRIM_DEG_FL * US_PER_DEGREE + (TRIM_DEG_FL >= 0 ? 0.5f : -0.5f)); // +44 us -> Neutro: 1544 us

static void controlTaskLoop(void *parameter) {
  static bool assistModeActive = false;

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(SERVO_UPDATE_INTERVAL_MS));

    uint16_t ch1 = 0, ch2 = 0, ch3 = 0, ch5 = 0;
    getReceiverChannels(ch1, ch2, ch3, ch5);

    bool rcSignalLost = isRCSignalLost();

    // Trimmed neutral pulse for each servo (1500 + static trim offset)
    const int neutralBR = 1500 + TRIM_US_BR;
    const int neutralBL = 1500 + TRIM_US_BL;
    const int neutralFR = 1500 + TRIM_US_FR;
    const int neutralFL = 1500 + TRIM_US_FL;

    int targetThrottle = THROTTLE_MIN_PULSE;
    int targetBR = neutralBR;
    int targetBL = neutralBL;
    int targetFR = neutralFR;
    int targetFL = neutralFL;

    // Symmetric angular limit in us around each servo's own trimmed neutral
    int anglePulseLimit = (int)(servoMaxAngleDeg * US_PER_DEGREE);

    uint16_t c1 = (ch1 > 0) ? ch1 : 1500;
    uint16_t c2 = (ch2 > 0) ? ch2 : 1500;
    uint16_t c3 = (ch3 > 0) ? ch3 : 1000;

    // Clean Schmidt Trigger Hysteresis for CH5 Assist Mode Switch
    if (ch5 > 1600) {
      assistModeActive = true;
    } else if (ch5 < 1400 && ch5 > 0) {
      assistModeActive = false;
    }

    if (!rcSignalLost) {
      // Throttle (CH3)
      targetThrottle =
          mapRangeLinear((int)c3, THROTTLE_INPUT_MIN_US, THROTTLE_INPUT_MAX_US,
                         THROTTLE_OUTPUT_MIN_US, THROTTLE_OUTPUT_MAX_US);
      if (isLowVoltageCutoffTriggered()) {
        targetThrottle = THROTTLE_MIN_PULSE;
      }

      int pitchDiff = (int)c2 - (int)centerCH2;
      int rollDiff = 0;

      // ── ROLL CONTROL: DIRECT MANUAL OR IMU-ASSISTED ROLL ENVELOPE PROTECTION
      // ──
      if (assistModeActive) {
        float curPitch = 0.0f, curRoll = 0.0f;
        getFilteredMPUData(curPitch, curRoll);

        // Pilot commanded roll angle in degrees (-20 deg to +20 deg span for
        // 1000us - 2000us)
        float pilotCmdDeg =
            constrain(((float)((int)c1 - (int)centerCH1) / 500.0f) * 20.0f,
                      -20.0f, 20.0f);
        float absRoll = fabsf(curRoll);
        float effectiveRollCmdDeg = pilotCmdDeg;

        if (absRoll <= 60.0f) {
          // Progressive attenuation of authority into the turn: 20 deg @ 0° ->
          // 0 deg @ 60°
          float maxIntoBank = 20.0f * (1.0f - (absRoll / 60.0f));

          if (curRoll >= 0.0f) {
            // Banked Right (+): Right turn capped at +maxIntoBank, Left
            // (recovery) allowed up to -20 deg
            effectiveRollCmdDeg = constrain(pilotCmdDeg, -20.0f, maxIntoBank);
          } else {
            // Banked Left (-): Left turn capped at -maxIntoBank, Right
            // (recovery) allowed up to +20 deg
            effectiveRollCmdDeg = constrain(pilotCmdDeg, -maxIntoBank, 20.0f);
          }
        } else {
          // Over-banked (> 60 deg): Smoothly ramp auto-recovery bias from 0 deg
          // @ 60° to 10 deg @ 70°
          float recoveryBiasDeg =
              constrain((absRoll - 60.0f) * 1.0f, 0.0f, 10.0f);

          if (curRoll >= 0.0f) {
            // Banked Right (> 60 deg): apply smooth opposite recovery bias
            // (negative roll)
            effectiveRollCmdDeg = constrain(pilotCmdDeg - recoveryBiasDeg,
                                            -20.0f, -recoveryBiasDeg);
          } else {
            // Banked Left (> 60 deg): apply smooth opposite recovery bias
            // (positive roll)
            effectiveRollCmdDeg = constrain(pilotCmdDeg + recoveryBiasDeg,
                                            recoveryBiasDeg, 20.0f);
          }
        }

        // Convert effective degrees back to pulse offset (20 deg = 500us scale)
        rollDiff = (int)((effectiveRollCmdDeg / 20.0f) * 500.0f);
      } else {
        // Manual Direct Mode (CH5 == 1000us)
        rollDiff = (int)c1 - (int)centerCH1;
      }

      // BR + BL (Rear Elevators)  = PITCH (CH2)
      // FR + FL (Front Rollerons) = ROLL (CH1)
      int pitchOffsetBR = pitchDiff;
      int pitchOffsetBL = -pitchDiff; // Inverted BL (mirrored servo mounting)
      int rollOffsetFR =
          -rollDiff; // Servos physically mirrored: same PWM sign → opposite
                     // mechanical deflection on each wing
      int rollOffsetFL = -rollDiff; // idem

      // Each servo constrained symmetrically around its own trimmed neutral
      targetBR =
          constrain(neutralBR + pitchOffsetBR, neutralBR - anglePulseLimit,
                    neutralBR + anglePulseLimit);
      targetBL =
          constrain(neutralBL + pitchOffsetBL, neutralBL - anglePulseLimit,
                    neutralBL + anglePulseLimit);
      targetFR =
          constrain(neutralFR + rollOffsetFR, neutralFR - anglePulseLimit,
                    neutralFR + anglePulseLimit);
      targetFL =
          constrain(neutralFL + rollOffsetFL, neutralFL - anglePulseLimit,
                    neutralFL + anglePulseLimit);
    } else {
      // Failsafe: return to trimmed neutral, motor off
      targetThrottle = THROTTLE_MIN_PULSE;
      targetBR = neutralBR;
      targetBL = neutralBL;
      targetFR = neutralFR;
      targetFL = neutralFL;
    }

    updateOutputChannel(0, servoBR, targetBR);
    updateOutputChannel(1, servoBL, targetBL);
    updateOutputChannel(2, servoFR, targetFR);
    updateOutputChannel(3, servoFL, targetFL);
    updateOutputChannel(4, escMotor, targetThrottle);
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

  // Initialize lastWritePulseUs to match the trim neutral values we are about
  // to write
  lastWritePulseUs[0] = 1500 + TRIM_US_BR;
  lastWritePulseUs[1] = 1500 + TRIM_US_BL;
  lastWritePulseUs[2] = 1500 + TRIM_US_FR;
  lastWritePulseUs[3] = 1500 + TRIM_US_FL;
  lastWritePulseUs[4] = THROTTLE_MIN_PULSE;

  servoBR.writeMicroseconds(lastWritePulseUs[0]);
  servoBL.writeMicroseconds(lastWritePulseUs[1]);
  servoFR.writeMicroseconds(lastWritePulseUs[2]);
  servoFL.writeMicroseconds(lastWritePulseUs[3]);
  escMotor.writeMicroseconds(lastWritePulseUs[4]);

  xTaskCreatePinnedToCore(controlTaskLoop, "ControlTask", 4096, NULL, 5, NULL,
                          0);
}
