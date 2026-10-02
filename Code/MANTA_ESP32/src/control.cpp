#include "control.h"
#include "battery.h"
#include "bmp280.h"
#include "config.h"
#include "gps.h"
#include "mpu6050.h"
#include "network.h"
#include "receiver.h"
#include <ESP32Servo.h>

static Servo servoBR;  // Back Right (PIN_SERVO_BR / GPIO13)
static Servo servoBL;  // Back Left (PIN_SERVO_BL / GPIO14)
static Servo servoFR;  // Front Right (PIN_SERVO_FR / GPIO27)
static Servo servoFL;  // Front Left (PIN_SERVO_FL / GPIO26)
static Servo escMotor; // Throttle / ESC (PIN_ESC / GPIO25)

static portMUX_TYPE actuatorMux = portMUX_INITIALIZER_UNLOCKED;

static volatile int lastWritePulseUs[5] = {
    1500, 1500, 1500, 1500,
    1000}; // Initialized properly in initControlSystem()
static constexpr uint16_t SERVO_UPDATE_INTERVAL_MS =
    20; // 50 Hz fast servo update

static void updateOutputChannel(uint8_t index, Servo &srv, int targetPulseUs) {
  if (index == 4) {
    targetPulseUs = constrain(targetPulseUs, THROTTLE_MIN_PULSE, THROTTLE_MAX_PULSE);
  } else {
    targetPulseUs = constrain(targetPulseUs, 1000, 2000);
  }
  if (targetPulseUs != lastWritePulseUs[index]) {
    srv.writeMicroseconds(targetPulseUs);
    portENTER_CRITICAL(&actuatorMux);
    lastWritePulseUs[index] = targetPulseUs;
    portEXIT_CRITICAL(&actuatorMux);
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

// ── RC EXPO & SCALING UTILITY ───────────────────────────────────────────────
// Applies a cubic exponential response curve to normalized [-1.0, +1.0] stick inputs:
// f(x) = (1 - expo) * x + expo * x^3
// Maps full stick range (1000-2000us) smoothly across [-maxPulseLimitUs, +maxPulseLimitUs]
// with reduced sensitivity near the stick center for fine corrections.
static inline int applyExpoAndScale(int rawStickUs, int centerUs, int maxPulseLimitUs, float expoFactor) {
  float norm = constrain((float)(rawStickUs - centerUs) * 0.002f, -1.0f, 1.0f);
  float shaped = (1.0f - expoFactor) * norm + expoFactor * (norm * norm * norm);
  return (int)(shaped * (float)maxPulseLimitUs);
}

static constexpr uint16_t centerCH1 = 1500;
static constexpr uint16_t centerCH2 = 1500;
static constexpr uint8_t servoMaxAngleDeg =
    DEFAULT_SERVO_MAX_ANGLE_DEG; // Default 25 deg

// ── STATIC CONTROL SURFACE CALIBRATION (TRIM OFFSETS) ───────────────────────
// Elevators (BR & BL) trimmed +2.0 deg UP at neutral to counter nose-down tendency
constexpr float TRIM_DEG_BR = -2.0f; // -2.0 deg (-22 us -> Neutral: 1478 us, BR elevator up)
constexpr float TRIM_DEG_BL = -12.0f; // -12.0 deg (+133 us -> Neutral: 1633 us, BL elevator inverted up)
constexpr float TRIM_DEG_FR = 0.0f;   // 0.0 deg (0 us -> Neutral: 1500 us)
constexpr float TRIM_DEG_FL = 4.0f;   // +4.0 deg UP (+44 us -> Mechanical spline fit neutral: 1544 us)

constexpr int TRIM_US_BR =
    (int)(TRIM_DEG_BR * US_PER_DEGREE + (TRIM_DEG_BR >= 0 ? 0.5f : -0.5f));
constexpr int TRIM_US_BL =
    -(int)(TRIM_DEG_BL * US_PER_DEGREE + (TRIM_DEG_BL >= 0 ? 0.5f : -0.5f));
constexpr int TRIM_US_FR = (int)(TRIM_DEG_FR * US_PER_DEGREE);
constexpr int TRIM_US_FL =
    +(int)(TRIM_DEG_FL * US_PER_DEGREE + (TRIM_DEG_FL >= 0 ? 0.5f : -0.5f));

static volatile FlightMode currentFlightMode = FLIGHT_MODE_1;
static volatile bool currentFlaperonActive = false;
static volatile bool currentRollActive = true; // Roll control is permanently enabled
static volatile bool currentAssistModeActive = false;

// Fly-By-Wire PI-D State Variables
static float pitchIntegrator = 0.0f;
static float rollIntegrator = 0.0f;
static float lastPitchMeas = 0.0f;
static float lastRollMeas = 0.0f;
static float currentTargetPitch = 0.0f;
static float currentTargetRoll = 0.0f;
static FlightMode lastFlightMode = FLIGHT_MODE_1;

// Extremum Seeking Control (ESC) State Variables (Mode 2)
static float escPitchThetaHat = 1.0f; // Nominal baseline scale = 1.0 (100%)
static float escRollThetaHat = 1.0f;
static float escPitchTimeSec = 0.0f;
static float escRollTimeSec = 0.0f;
static float escPitchHPF = 0.0f;
static float escRollHPF = 0.0f;
static float escLastCostPitch = 0.0f;
static float escLastCostRoll = 0.0f;
static bool escPitchFirstTick = true;
static bool escRollFirstTick = true;
static volatile float escCurrentPitchScale = 1.0f;
static volatile float escCurrentRollScale = 1.0f;
static volatile bool escIsActive = false;

void resetExtremumSeeking(bool resetLearnedGains) {
  if (resetLearnedGains) {
    escPitchThetaHat = 1.0f;
    escRollThetaHat = 1.0f;
  }
  escPitchTimeSec = 0.0f;
  escRollTimeSec = 0.0f;
  escPitchHPF = 0.0f;
  escRollHPF = 0.0f;
  escLastCostPitch = 0.0f;
  escLastCostRoll = 0.0f;
  escPitchFirstTick = true;
  escRollFirstTick = true;
  escCurrentPitchScale = 1.0f;
  escCurrentRollScale = escRollThetaHat;
  escIsActive = false;
}

bool isExtremumSeekingActive() {
  return escIsActive;
}

void getExtremumSeekingScale(float &pitchScale, float &rollScale) {
  pitchScale = escCurrentPitchScale;
  rollScale = escCurrentRollScale;
}

void resetControlIntegrators() {
  pitchIntegrator = 0.0f;
  rollIntegrator = 0.0f;
}

void getFBWTargets(float &targetPitch, float &targetRoll) {
  targetPitch = currentTargetPitch;
  targetRoll = currentTargetRoll;
}

// Autonomous Emergency Failsafe State Variables (Signal Loss Recovery)
static volatile FailsafeStage activeFailsafeStage = FS_STAGE_INACTIVE;
static bool wasThrottleAbove1600Ever = false;
static unsigned long throttleZeroStartTimeMs = 0;
static unsigned long failsafeStartTimeMs = 0;
static unsigned long lastThrottleAltAdjustTimeMs = 0;
static unsigned long lastStage3StepTimeMs = 0;
static int failsafeDynamicThrottleUs = FAILSAFE_LOITER_THROTTLE_BASE_US;
static float currentSlewTargetPitch = 0.0f;
static float currentSlewTargetRoll = 0.0f;
static int currentSlewThrottle = 1000;

FailsafeStage getActiveFailsafeStage() {
  return activeFailsafeStage;
}

// Active in-flight PID gains
static volatile float currentActivePitchKp = PID_PITCH_KP;
static volatile float currentActivePitchKi = PID_PITCH_KI;
static volatile float currentActivePitchKd = PID_PITCH_KD;
static volatile float currentActiveRollKp = PID_ROLL_KP;
static volatile float currentActiveRollKi = PID_ROLL_KI;
static volatile float currentActiveRollKd = PID_ROLL_KD;

void getActivePIDGains(float &pitchKp, float &pitchKi, float &pitchKd,
                       float &rollKp, float &rollKi, float &rollKd) {
  pitchKp = currentActivePitchKp;
  pitchKi = currentActivePitchKi;
  pitchKd = currentActivePitchKd;
  rollKp = currentActiveRollKp;
  rollKi = currentActiveRollKi;
  rollKd = currentActiveRollKd;
}

// Quasi-linear Expo utility for attitude angles in degrees
static inline float applyExpoAndScaleDeg(int rawStickUs, int centerUs, float maxDeg, float expoFactor) {
  float norm = constrain((float)(rawStickUs - centerUs) * 0.002f, -1.0f, 1.0f);
  float shaped = (1.0f - expoFactor) * norm + expoFactor * (norm * norm * norm);
  return shaped * maxDeg;
}

void decodeCH5(uint16_t ch5Pulse, FlightMode &mode, bool &flaperonActive) {
  // SWC (3-position switch) + SWB (2-position switch) transmitter calibration:
  // SWC 1 + SWB OFF: ~1166 us -> Mode 1 + Flaperons OFF (Pitch Manual + Roll Assist)
  // SWC 2 + SWB OFF: ~1328 us -> Mode 2 + Flaperons OFF (Fixed FBW Pitch & Roll)
  // SWC 3 + SWB OFF: ~1411 us -> Mode 3 + Flaperons OFF (Adaptive FBW Pitch & Roll)
  // SWC 1 + SWB ON:  ~1541 us -> Mode 1 + Flaperons ON  (Pitch Manual + Roll Assist + Flaps 20 deg DOWN)
  // SWC 2 + SWB ON:  ~1825 us -> Mode 2 + Flaperons ON  (Fixed FBW Pitch & Roll + Flaps 20 deg DOWN)
  // SWC 3 + SWB ON:  ~1942 us -> Mode 2 Auto Flap-Safe + Flaperons ON (Safety: Mode 3 demoted to 2)
  if (ch5Pulse < 1247) {
    mode = FLIGHT_MODE_1;
    flaperonActive = false;
  } else if (ch5Pulse < 1370) {
    mode = FLIGHT_MODE_2;
    flaperonActive = false;
  } else if (ch5Pulse < 1476) {
    mode = FLIGHT_MODE_3;
    flaperonActive = false;
  } else if (ch5Pulse < 1683) {
    mode = FLIGHT_MODE_1;
    flaperonActive = true;
  } else if (ch5Pulse < 1884) {
    mode = FLIGHT_MODE_2;
    flaperonActive = true;
  } else {
    // Mode 3 is prohibited when Flaperons is ON to prevent corrupting adaptive learning.
    // Automatically demotes to Mode 2.
    mode = FLIGHT_MODE_2;
    flaperonActive = true;
  }
}

void decodeCH5WithHysteresis(uint16_t ch5Pulse, FlightMode currentMode, bool currentFlaperon, FlightMode &outMode, bool &outFlaperon) {
  if (ch5Pulse < 850 || ch5Pulse > 2150) {
    outMode = currentMode;
    outFlaperon = currentFlaperon;
    return;
  }

  // Map current state: 0 to 5
  uint8_t curState = 0;
  if (currentMode == FLIGHT_MODE_1) {
    curState = currentFlaperon ? 3 : 0;
  } else if (currentMode == FLIGHT_MODE_2) {
    curState = currentFlaperon ? 4 : 1;
  } else {
    curState = currentFlaperon ? 5 : 2;
  }

  uint8_t nextState = curState;

  // Direct midpoint decoding with symmetric hysteresis (+/- 3us to +/- 5us):
  if (ch5Pulse < (curState == 0 ? CH5_THRES_M1_OFF_TO_M2_OFF : CH5_THRES_M2_OFF_TO_M1_OFF)) {
    nextState = 0;
  } else if (ch5Pulse < (curState <= 1 ? CH5_THRES_M2_OFF_TO_M3_OFF : CH5_THRES_M3_OFF_TO_M2_OFF)) {
    nextState = 1;
  } else if (ch5Pulse < (curState <= 2 ? CH5_THRES_M3_OFF_TO_M1_ON : CH5_THRES_M1_ON_TO_M3_OFF)) {
    nextState = 2;
  } else if (ch5Pulse < (curState <= 3 ? CH5_THRES_M1_ON_TO_M2_ON : CH5_THRES_M2_ON_TO_M1_ON)) {
    nextState = 3;
  } else if (ch5Pulse < (curState <= 4 ? CH5_THRES_M2_ON_TO_M3_ON : CH5_THRES_M3_ON_TO_M2_ON)) {
    nextState = 4;
  } else {
    nextState = 5;
  }

  switch (nextState) {
    case 0:
      outMode = FLIGHT_MODE_1;
      outFlaperon = false;
      break;
    case 1:
      outMode = FLIGHT_MODE_2;
      outFlaperon = false;
      break;
    case 2:
      outMode = FLIGHT_MODE_3;
      outFlaperon = false;
      break;
    case 3:
      outMode = FLIGHT_MODE_1;
      outFlaperon = true;
      break;
    case 4:
      outMode = FLIGHT_MODE_2;
      outFlaperon = true;
      break;
    case 5:
    default:
      // Safety rule: Flaperons active prohibits Mode 3 -> Auto-demote to Mode 2
      outMode = FLIGHT_MODE_2;
      outFlaperon = true;
      break;
  }
}

FlightMode getCurrentFlightMode() {
  return currentFlightMode;
}

bool isRollActive() {
  return currentRollActive;
}

bool isFlaperonActive() {
  return currentFlaperonActive;
}

void getActuatorOutputs(int &br, int &bl, int &fr, int &fl, int &throttle, bool &flaperonActive) {
  portENTER_CRITICAL(&actuatorMux);
  br = lastWritePulseUs[0];
  bl = lastWritePulseUs[1];
  fr = lastWritePulseUs[2];
  fl = lastWritePulseUs[3];
  throttle = lastWritePulseUs[4];
  flaperonActive = currentFlaperonActive;
  portEXIT_CRITICAL(&actuatorMux);
}

void getActuatorOutputs(int &br, int &bl, int &fr, int &fl, int &throttle, FlightMode &flightMode, bool &flaperonActive) {
  portENTER_CRITICAL(&actuatorMux);
  br = lastWritePulseUs[0];
  bl = lastWritePulseUs[1];
  fr = lastWritePulseUs[2];
  fl = lastWritePulseUs[3];
  throttle = lastWritePulseUs[4];
  flightMode = currentFlightMode;
  flaperonActive = currentFlaperonActive;
  portEXIT_CRITICAL(&actuatorMux);
}

static void controlTaskLoop(void *parameter) {
  constexpr float dt = (float)SERVO_UPDATE_INTERVAL_MS / 1000.0f; // 0.020s
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(SERVO_UPDATE_INTERVAL_MS);

  while (true) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

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

    // Symmetric angular limit in us around each servo's own trimmed neutral (+/-278us for 25 deg)
    int anglePulseLimit = (int)(servoMaxAngleDeg * US_PER_DEGREE + 0.5f);

    uint16_t c1 = (ch1 > 0) ? ch1 : 1500;
    uint16_t c2 = (ch2 > 0) ? ch2 : 1500;
    uint16_t c3 = (ch3 > 0) ? ch3 : 1000;
    uint16_t c5 = (ch5 > 0) ? ch5 : 0;

    // ── CH5 SCHMITT-TRIGGER HYSTERESIS & TEMPORAL DEBOUNCE ──────────────────
    static FlightMode pendingMode = FLIGHT_MODE_1;
    static bool pendingFlaperon = false;
    static uint8_t debounceCount = 0;
    static bool firstLoopTick = true;
    static bool lastFlaperonActive = false;
    static bool wasInFailsafe = false;

    if (!rcSignalLost && c5 > 0) {
      FlightMode candidateMode = currentFlightMode;
      bool candidateFlaperon = currentFlaperonActive;
      decodeCH5WithHysteresis(c5, currentFlightMode, currentFlaperonActive, candidateMode, candidateFlaperon);

      if (firstLoopTick) {
        pendingMode = candidateMode;
        pendingFlaperon = candidateFlaperon;
        currentFlightMode = candidateMode;
        currentFlaperonActive = candidateFlaperon;
        currentAssistModeActive = candidateFlaperon;
        lastFlightMode = candidateMode;
        lastFlaperonActive = candidateFlaperon;
        debounceCount = CH5_DEBOUNCE_CONFIRM_TICKS;
      } else {
        if (candidateMode == pendingMode && candidateFlaperon == pendingFlaperon) {
          if (debounceCount < CH5_DEBOUNCE_CONFIRM_TICKS) {
            debounceCount++;
            if (debounceCount >= CH5_DEBOUNCE_CONFIRM_TICKS) {
              currentFlightMode = candidateMode;
              currentFlaperonActive = candidateFlaperon;
              currentAssistModeActive = candidateFlaperon;
            }
          }
        } else {
          pendingMode = candidateMode;
          pendingFlaperon = candidateFlaperon;
          debounceCount = 1;
        }
      }
    }

    FlightMode flightMode = currentFlightMode;
    bool flaperonActive = currentFlaperonActive;
    bool rollActive = true; // Roll control is PERMANENTLY ENABLED

    // Safety fallback: if MPU6050 IMU is offline, fallback to Mode 1 (Manual)
    bool mpuHealthy = isMPU6050Available();
    if (!mpuHealthy) {
      flightMode = FLIGHT_MODE_1;
      rollActive = false;
      resetControlIntegrators();
      resetExtremumSeeking(false);
    }

    // Reset integrators and ESC on flight mode change (bumpless transfer)
    bool modeJustChanged = false;
    if (flightMode != lastFlightMode) {
      resetControlIntegrators();
      resetExtremumSeeking(false);
      lastFlightMode = flightMode;
      lastFlaperonActive = flaperonActive;
      modeJustChanged = true;
    } else if (flaperonActive != lastFlaperonActive) {
      rollIntegrator = 0.0f;
      lastFlaperonActive = flaperonActive;
    }

    float curPitch = 0.0f, curRoll = 0.0f;
    getFilteredMPUData(curPitch, curRoll);

    // Defensive validation: reject NaN / Inf from IMU
    if (isnan(curPitch) || isnan(curRoll) || isinf(curPitch) || isinf(curRoll)) {
      mpuHealthy = false;
      curPitch = 0.0f;
      curRoll = 0.0f;
      flightMode = FLIGHT_MODE_1;
      rollActive = false;
      resetControlIntegrators();
      resetExtremumSeeking(false);
    }

    if (firstLoopTick || modeJustChanged) {
      lastPitchMeas = curPitch;
      lastRollMeas = curRoll;
      firstLoopTick = false;
    }

    // Angular rate calculation for derivative on measurement (prevents derivative kick)
    float pitchRateDegS = (curPitch - lastPitchMeas) / dt;
    float rollRateDegS = (curRoll - lastRollMeas) / dt;
    lastPitchMeas = curPitch;
    lastRollMeas = curRoll;

    if (!rcSignalLost) {
      unsigned long nowMs = millis();
      // Arm in-flight failsafe only after throttle exceeds 1600us
      if (c3 > FAILSAFE_ARM_THROTTLE_THRESHOLD_US) {
        wasThrottleAbove1600Ever = true;
      }

      // Auto-disarm after landing: idle throttle for > 20s disarms in-flight failsafe
      if (c3 < FAILSAFE_DISARM_THROTTLE_THRESHOLD_US) {
        if (throttleZeroStartTimeMs == 0) {
          throttleZeroStartTimeMs = nowMs;
        } else if (nowMs - throttleZeroStartTimeMs >= FAILSAFE_LANDING_DISARM_TIMEOUT_MS) {
          wasThrottleAbove1600Ever = false;
        }
      } else {
        throttleZeroStartTimeMs = 0;
      }

      // Reset failsafe active stage
      if (wasInFailsafe) {
        resetControlIntegrators();
        wasInFailsafe = false;
      }
      activeFailsafeStage = FS_STAGE_INACTIVE;
      failsafeStartTimeMs = 0;
      lastThrottleAltAdjustTimeMs = 0;
      lastStage3StepTimeMs = 0;
      failsafeDynamicThrottleUs = FAILSAFE_LOITER_THROTTLE_BASE_US;

      // Throttle (CH3)
      targetThrottle =
          mapRangeLinear((int)c3, THROTTLE_INPUT_MIN_US, THROTTLE_INPUT_MAX_US,
                         THROTTLE_OUTPUT_MIN_US, THROTTLE_OUTPUT_MAX_US);
      if (isLowVoltageCutoffTriggered()) {
        // Soft power ceiling: allow up to ~35-40% throttle (1350us) to maintain flight and glide back safely
        if (targetThrottle > THROTTLE_LOW_VOLT_CEILING_PULSE) {
          targetThrottle = THROTTLE_LOW_VOLT_CEILING_PULSE;
        }
      }

      currentSlewThrottle = targetThrottle;
      currentSlewTargetPitch = currentTargetPitch;
      currentSlewTargetRoll = currentTargetRoll;

      int pitchDiff = 0;
      int rollDiff = 0;

      // ── MODE 1: 100% MANUAL (WITH ROLL ASSIST) ────────────────────────────
      if (flightMode == FLIGHT_MODE_1) {
        escIsActive = false;
        float rollScale = constrain(escRollThetaHat, ESC_ROLL_MIN_SCALE, ESC_ROLL_MAX_SCALE);
        escCurrentPitchScale = 1.0f;
        escCurrentRollScale = rollScale;

        // Pitch: Direct manual stick with expo
        pitchDiff = applyExpoAndScale((int)c2, (int)centerCH2, anglePulseLimit, RC_EXPO_FACTOR);
        currentTargetPitch = 0.0f;

        currentActivePitchKp = 0.0f;
        currentActivePitchKi = 0.0f;
        currentActivePitchKd = 0.0f;
        currentActiveRollKp = PID_ROLL_KP * rollScale;
        currentActiveRollKi = PID_ROLL_KI;
        currentActiveRollKd = PID_ROLL_KD * sqrtf(rollScale);

        if (rollActive) {
          // Roll Assist / Envelope Protection (permanently active in normal flight)
          float normC1 = constrain((float)((int)c1 - (int)centerCH1) * 0.002f, -1.0f, 1.0f);
          float shapedC1 = (1.0f - RC_EXPO_FACTOR) * normC1 + RC_EXPO_FACTOR * (normC1 * normC1 * normC1);
          float pilotCmdDeg = constrain(shapedC1 * 20.0f, -20.0f, 20.0f);
          float absRoll = fabsf(curRoll);
          float effectiveRollCmdDeg = pilotCmdDeg;

          if (absRoll <= 60.0f) {
            float maxIntoBank = 20.0f * (1.0f - (absRoll / 60.0f));
            if (curRoll >= 0.0f) {
              effectiveRollCmdDeg = constrain(pilotCmdDeg, -20.0f, maxIntoBank);
            } else {
              effectiveRollCmdDeg = constrain(pilotCmdDeg, -maxIntoBank, 20.0f);
            }
          } else {
            float recoveryBiasDeg = constrain((absRoll - 60.0f) * 1.0f, 0.0f, 10.0f);
            if (curRoll >= 0.0f) {
              effectiveRollCmdDeg = constrain(pilotCmdDeg - recoveryBiasDeg, -20.0f, -recoveryBiasDeg);
            } else {
              effectiveRollCmdDeg = constrain(pilotCmdDeg + recoveryBiasDeg, recoveryBiasDeg, 20.0f);
            }
          }
          rollDiff = (int)((effectiveRollCmdDeg / 20.0f) * (float)anglePulseLimit);
        } else {
          // Direct 100% manual roll fallback if IMU is offline
          rollDiff = applyExpoAndScale((int)c1, (int)centerCH1, anglePulseLimit, RC_EXPO_FACTOR);
        }
        currentTargetRoll = 0.0f;
        resetControlIntegrators();
      }

      // ── MODE 2: FIXED FLY-BY-WIRE (WITH MODE 3 SHARED GAINS) ──────────────
      else if (flightMode == FLIGHT_MODE_2) {
        escIsActive = false;

        float pitchScale = constrain(escPitchThetaHat, ESC_PITCH_MIN_SCALE, ESC_PITCH_MAX_SCALE);
        float rollScale = constrain(escRollThetaHat, ESC_ROLL_MIN_SCALE, ESC_ROLL_MAX_SCALE);
        escCurrentPitchScale = pitchScale;
        escCurrentRollScale = rollScale;

        float activePitchKp = PID_PITCH_KP * pitchScale;
        float activePitchKi = PID_PITCH_KI;
        float activePitchKd = PID_PITCH_KD * sqrtf(pitchScale);

        float activeRollKp = PID_ROLL_KP * rollScale;
        float activeRollKi = PID_ROLL_KI;
        float activeRollKd = PID_ROLL_KD * sqrtf(rollScale);

        currentActivePitchKp = activePitchKp;
        currentActivePitchKi = activePitchKi;
        currentActivePitchKd = activePitchKd;
        currentActiveRollKp = activeRollKp;
        currentActiveRollKi = activeRollKi;
        currentActiveRollKd = activeRollKd;

        // Pitch FBW PI-D (Closed-loop with strictly negative feedback):
        // Pull stick (c2 < 1500us) commands Pitch UP (+deg)
        // Push stick (c2 > 1500us) commands Pitch DOWN (-deg)
        float pilotTargetPitchDeg = -applyExpoAndScaleDeg((int)c2, (int)centerCH2, FBW_MAX_PITCH_DEG, FBW_EXPO_FACTOR);
        float rollRad = fabsf(curRoll) * DEG_TO_RAD;
        float turnPitchCompDeg = constrain(TURN_PITCH_COMP_GAIN * (1.0f - cosf(rollRad)), 0.0f, 3.5f);
        float targetPitchDeg = constrain(pilotTargetPitchDeg + turnPitchCompDeg, -FBW_MAX_PITCH_DEG, FBW_MAX_PITCH_DEG);
        currentTargetPitch = targetPitchDeg;

        float pitchError = targetPitchDeg - curPitch;

        // Conditional anti-windup: integrate only with active motor and inside envelope
        if (c3 > 1050 && fabsf(curPitch) <= 45.0f && fabsf(curRoll) <= 60.0f) {
          pitchIntegrator += activePitchKi * pitchError * dt;
          pitchIntegrator = constrain(pitchIntegrator, -MAX_INTEGRAL_PULSE_PITCH_US, MAX_INTEGRAL_PULSE_PITCH_US);
        } else {
          pitchIntegrator = 0.0f;
        }

        float pitchPidOut = (activePitchKp * pitchError) + pitchIntegrator - (activePitchKd * pitchRateDegS);
        // V-Tail Kinematics: pitchDiff < 0 is Pitch UP (BR decreases, BL increases).
        // Therefore pitchDiff = -pitchPidOut enforces stable negative feedback.
        pitchDiff = -constrain((int)pitchPidOut, -anglePulseLimit, anglePulseLimit);

        // Roll FBW:
        if (rollActive) {
          float targetRollDeg = applyExpoAndScaleDeg((int)c1, (int)centerCH1, FBW_MAX_ROLL_DEG, FBW_EXPO_FACTOR);
          currentTargetRoll = targetRollDeg;

          float rollError = targetRollDeg - curRoll;

          if (c3 > 1050 && fabsf(curRoll) <= 60.0f) {
            rollIntegrator += activeRollKi * rollError * dt;
            rollIntegrator = constrain(rollIntegrator, -MAX_INTEGRAL_PULSE_ROLL_US, MAX_INTEGRAL_PULSE_ROLL_US);
          } else {
            rollIntegrator = 0.0f;
          }

          float rollPidOut = (activeRollKp * rollError) + rollIntegrator - (activeRollKd * rollRateDegS);
          rollDiff = constrain((int)rollPidOut, -anglePulseLimit, anglePulseLimit);
        } else {
          rollDiff = applyExpoAndScale((int)c1, (int)centerCH1, anglePulseLimit, FBW_EXPO_FACTOR);
          rollIntegrator = 0.0f;
          currentTargetRoll = 0.0f;
        }
      }

      // ── MODE 3: ADAPTIVE FLY-BY-WIRE (EXTREMUM SEEKING PI-D) ───────────────
      else {
        escIsActive = rollActive && !flaperonActive; // Extremum Seeking active on roll (disabled if flaperons ON)

        float pitchScale = constrain(escPitchThetaHat, ESC_PITCH_MIN_SCALE, ESC_PITCH_MAX_SCALE);
        float activePitchKp = PID_PITCH_KP * pitchScale;
        float activePitchKi = PID_PITCH_KI;
        float activePitchKd = PID_PITCH_KD * sqrtf(pitchScale);
        currentActivePitchKp = activePitchKp;
        currentActivePitchKi = activePitchKi;
        currentActivePitchKd = activePitchKd;
        escCurrentPitchScale = pitchScale;

        // Pitch FBW PI-D (Closed-loop with strictly negative feedback)
        float pilotTargetPitchDeg = -applyExpoAndScaleDeg((int)c2, (int)centerCH2, FBW_MAX_PITCH_DEG, FBW_EXPO_FACTOR);
        float rollRad = fabsf(curRoll) * DEG_TO_RAD;
        float turnPitchCompDeg = constrain(TURN_PITCH_COMP_GAIN * (1.0f - cosf(rollRad)), 0.0f, 3.5f);
        float targetPitchDeg = constrain(pilotTargetPitchDeg + turnPitchCompDeg, -FBW_MAX_PITCH_DEG, FBW_MAX_PITCH_DEG);
        currentTargetPitch = targetPitchDeg;

        float pitchError = targetPitchDeg - curPitch;

        if (c3 > 1050 && fabsf(curPitch) <= 45.0f && fabsf(curRoll) <= 60.0f) {
          pitchIntegrator += activePitchKi * pitchError * dt;
          pitchIntegrator = constrain(pitchIntegrator, -MAX_INTEGRAL_PULSE_PITCH_US, MAX_INTEGRAL_PULSE_PITCH_US);
        } else {
          pitchIntegrator = 0.0f;
        }

        float pitchPidOut = (activePitchKp * pitchError) + pitchIntegrator - (activePitchKd * pitchRateDegS);
        pitchDiff = -constrain((int)pitchPidOut, -anglePulseLimit, anglePulseLimit);

        // Roll FBW with Extremum Seeking
        if (rollActive) {
          float targetRollDeg = applyExpoAndScaleDeg((int)c1, (int)centerCH1, FBW_MAX_ROLL_DEG, FBW_EXPO_FACTOR);
          currentTargetRoll = targetRollDeg;
          float rollError = targetRollDeg - curRoll;

          escRollTimeSec += dt;
          constexpr float ESC_ROLL_PERIOD = 2.0f * (float)M_PI / ESC_ROLL_OMEGA; // ~0.6667s for 1.5Hz
          if (escRollTimeSec >= ESC_ROLL_PERIOD) {
            escRollTimeSec -= ESC_ROLL_PERIOD;
          }
          float ditherRoll = ESC_ROLL_DITHER_AMP * sinf(ESC_ROLL_OMEGA * escRollTimeSec);
          float effectiveRollTheta = constrain(escRollThetaHat + ditherRoll, ESC_ROLL_MIN_SCALE, ESC_ROLL_MAX_SCALE);
          escCurrentRollScale = effectiveRollTheta;

          float activeRollKp = PID_ROLL_KP * effectiveRollTheta;
          float activeRollKd = PID_ROLL_KD * sqrtf(effectiveRollTheta); // Preserves natural damping ratio zeta
          float activeRollKi = PID_ROLL_KI;

          currentActiveRollKp = activeRollKp;
          currentActiveRollKi = activeRollKi;
          currentActiveRollKd = activeRollKd;

          if (c3 > 1050 && fabsf(curRoll) <= 60.0f) {
            rollIntegrator += activeRollKi * rollError * dt;
            rollIntegrator = constrain(rollIntegrator, -MAX_INTEGRAL_PULSE_ROLL_US, MAX_INTEGRAL_PULSE_ROLL_US);
          } else {
            rollIntegrator = 0.0f;
          }

          float rollPidOut = (activeRollKp * rollError) + rollIntegrator - (activeRollKd * rollRateDegS);
          rollDiff = constrain((int)rollPidOut, -anglePulseLimit, anglePulseLimit);

          bool allowRollAdaptation = (c3 > 1050) && (fabsf(curPitch) <= 50.0f) && (fabsf(curRoll) <= 60.0f);
          float costRoll = (rollError * rollError) + (ESC_ROLL_RATE_WEIGHT * rollRateDegS * rollRateDegS);

          if (escRollFirstTick) {
            escLastCostRoll = costRoll;
            escRollHPF = 0.0f;
            escRollFirstTick = false;
          } else {
            constexpr float alphaHPF = 1.0f / (1.0f + (ESC_HPF_CUTOFF * dt));
            escRollHPF = alphaHPF * (escRollHPF + costRoll - escLastCostRoll);
            escLastCostRoll = costRoll;

            if (allowRollAdaptation) {
              float demodRoll = escRollHPF * sinf(ESC_ROLL_OMEGA * escRollTimeSec);
              escRollThetaHat -= (ESC_ROLL_GAMMA * demodRoll * dt);
              escRollThetaHat = constrain(escRollThetaHat, ESC_ROLL_MIN_SCALE, ESC_ROLL_MAX_SCALE);
            }
          }
        } else {
          // Roll OFF: Direct manual roll
          rollDiff = applyExpoAndScale((int)c1, (int)centerCH1, anglePulseLimit, FBW_EXPO_FACTOR);
          rollIntegrator = 0.0f;
          currentTargetRoll = 0.0f;
          float rollScale = constrain(escRollThetaHat, ESC_ROLL_MIN_SCALE, ESC_ROLL_MAX_SCALE);
          escCurrentRollScale = rollScale;
          escRollTimeSec = 0.0f;
          escRollHPF = 0.0f;
          escRollFirstTick = true;
          currentActiveRollKp = PID_ROLL_KP * rollScale;
          currentActiveRollKi = PID_ROLL_KI;
          currentActiveRollKd = PID_ROLL_KD * sqrtf(rollScale);
        }
      }

      // BR + BL (Rear Elevators)  = PITCH (CH2)
      // FR + FL (Front Rollerons) = ROLL (CH1)
      int pitchOffsetBR = pitchDiff;
      int pitchOffsetBL = -pitchDiff; // Inverted BL (mirrored servo mounting)

      // Flaperons (Landing Flaps): Up to 20 deg DOWN offset on both surfaces,
      // governed smoothly by throttle between 1500us and 1200us (linear uniform transition).
      float flaperonScale = 0.0f;
      if (flaperonActive) {
        if (c3 <= FLAPERON_THROTTLE_MIN_US) {
          flaperonScale = 1.0f;
        } else if (c3 >= FLAPERON_THROTTLE_MAX_US) {
          flaperonScale = 0.0f;
        } else {
          flaperonScale = (float)(FLAPERON_THROTTLE_MAX_US - (int)c3) /
                          (float)(FLAPERON_THROTTLE_MAX_US - FLAPERON_THROTTLE_MIN_US);
        }
      }

      // Dynamic Anti-Saturation & Roll Priority:
      // When roll demand is high, flaperon offset is dynamically scaled by remaining headroom
      // ensuring zero control clipping at stick extremes and protecting mechanical servo limits.
      // With FLAPERON_MAX_PULSE_LIMIT_US (~356us / 32.0 deg), flaperons maintain 100% deflection (20.0 deg)
      // during normal steering and gracefully yield only when roll demand approaches extreme limits.
      float availableHeadroomUs = (float)FLAPERON_MAX_PULSE_LIMIT_US - fabsf((float)rollDiff);
      float flaperonHeadroom = constrain(availableHeadroomUs / (float)FLAPERON_OFFSET_US, 0.0f, 1.0f);
      float effectiveFlaperonScale = flaperonScale * flaperonHeadroom;

      int flaperonOffsetFR = (int)roundf((float)FLAPERON_US_FR * effectiveFlaperonScale);
      int flaperonOffsetFL = (int)roundf((float)FLAPERON_US_FL * effectiveFlaperonScale);

      int rollOffsetFR = flaperonOffsetFR - rollDiff;
      int rollOffsetFL = flaperonOffsetFL - rollDiff;

      int rollLimitFRFL = (flaperonActive && effectiveFlaperonScale > 0.0f) ? FLAPERON_MAX_PULSE_LIMIT_US : anglePulseLimit;

      // Each servo constrained symmetrically around its own trimmed neutral and within hardware limits [1000, 2000] us
      targetBR =
          constrain(neutralBR + pitchOffsetBR, neutralBR - anglePulseLimit,
                    neutralBR + anglePulseLimit);
      targetBL =
          constrain(neutralBL + pitchOffsetBL, neutralBL - anglePulseLimit,
                    neutralBL + anglePulseLimit);
      targetFR =
          constrain(neutralFR + rollOffsetFR, neutralFR - rollLimitFRFL,
                    neutralFR + rollLimitFRFL);
      targetFL =
          constrain(neutralFL + rollOffsetFL, neutralFL - rollLimitFRFL,
                    neutralFL + rollLimitFRFL);

      targetBR = constrain(targetBR, 1000, 2000);
      targetBL = constrain(targetBL, 1000, 2000);
      targetFR = constrain(targetFR, 1000, 2000);
      targetFL = constrain(targetFL, 1000, 2000);
    } else {
      wasInFailsafe = true;
      // ── AUTONOMOUS EMERGENCY FAILSAFE (SIGNAL LOSS RECOVERY) ──────────────
      if (!wasThrottleAbove1600Ever) {
        // Ground / Bench Safety Guard: If signal is lost before throttle ever exceeded 1600us,
        // keep motor strictly OFF (1000us) and control surfaces at trimmed neutral.
        activeFailsafeStage = FS_STAGE_GROUND;
        targetThrottle = THROTTLE_MIN_PULSE;
        targetBR = neutralBR;
        targetBL = neutralBL;
        targetFR = neutralFR;
        targetFL = neutralFL;
        resetControlIntegrators();
        resetExtremumSeeking(false);
        currentTargetPitch = 0.0f;
        currentTargetRoll = 0.0f;
        float rollScale = constrain(escRollThetaHat, ESC_ROLL_MIN_SCALE, ESC_ROLL_MAX_SCALE);
        escCurrentPitchScale = 1.0f;
        currentActivePitchKp = 0.0f;
        currentActivePitchKi = 0.0f;
        currentActivePitchKd = 0.0f;
        currentActiveRollKp = PID_ROLL_KP * rollScale;
        currentActiveRollKi = PID_ROLL_KI;
        currentActiveRollKd = PID_ROLL_KD * sqrtf(rollScale);
      } else {
        // In-flight active emergency failsafe:
        // Stage 1 (Climb): Climb to safe altitude (+20 deg pitch, 1800us) if altitude < 25m
        // Stage 2 (Loiter): Closed-loop loiter (Roll -30 deg, Pitch +5 deg, Throttle 1500us with altitude adjust)
        // Stage 3 (Descend): Timeout > 30s or baro failure -> decrement throttle 1us/sec continuously until touchdown
        float curAlt = 0.0f, baroP = 0.0f, baroT = 0.0f;
        getBaroData(curAlt, baroP, baroT);
        bool baroHealthy = isBMP280Available() && !isnan(curAlt) && !isinf(curAlt) && (curAlt > -100.0f);
        if (!baroHealthy) {
          int32_t latE7 = 0, lonE7 = 0;
          int16_t gpsAltX10 = 0;
          uint8_t sats = 0, fix = 0;
          getGPSData(latE7, lonE7, gpsAltX10, sats, fix);
          if (fix >= 2) {
            curAlt = (float)gpsAltX10 / 10.0f;
            baroHealthy = true; // Fallback to GPS altitude if barometer is offline
          }
        }

        unsigned long nowMs = millis();

        if (activeFailsafeStage == FS_STAGE_INACTIVE || activeFailsafeStage == FS_STAGE_GROUND) {
          resetControlIntegrators();
          resetExtremumSeeking(false);
          failsafeStartTimeMs = nowMs;
          lastThrottleAltAdjustTimeMs = nowMs;
          lastStage3StepTimeMs = nowMs;
          failsafeDynamicThrottleUs = FAILSAFE_LOITER_THROTTLE_BASE_US;

          if (!baroHealthy) {
            activeFailsafeStage = FS_STAGE_DESCEND;
          } else if (curAlt < FAILSAFE_CLIMB_ALT_M) {
            activeFailsafeStage = FS_STAGE_CLIMB;
          } else {
            activeFailsafeStage = FS_STAGE_LOITER;
          }
        }

        if (activeFailsafeStage == FS_STAGE_CLIMB) {
          if (!baroHealthy) {
            activeFailsafeStage = FS_STAGE_DESCEND;
            lastStage3StepTimeMs = nowMs;
          } else if (curAlt >= FAILSAFE_CLIMB_ALT_M) {
            activeFailsafeStage = FS_STAGE_LOITER;
            lastThrottleAltAdjustTimeMs = nowMs;
          }
        } else if (activeFailsafeStage == FS_STAGE_LOITER) {
          if (!baroHealthy || (nowMs - failsafeStartTimeMs >= FAILSAFE_STAGE2_TIMEOUT_MS)) {
            activeFailsafeStage = FS_STAGE_DESCEND;
            lastStage3StepTimeMs = nowMs;
          }
        }

        float desiredTargetPitch = FAILSAFE_LOITER_PITCH_DEG; // +5.0 deg (Loiter/Descend default)
        float desiredTargetRoll = FAILSAFE_LOITER_ROLL_DEG;   // -30.0 deg (Loiter/Descend default)
        int desiredTargetThrottle = failsafeDynamicThrottleUs;

        if (activeFailsafeStage == FS_STAGE_CLIMB) {
          desiredTargetPitch = FAILSAFE_CLIMB_PITCH_DEG;   // +20.0 deg
          desiredTargetRoll = FAILSAFE_CLIMB_ROLL_DEG;     // 0.0 deg (wings level)
          desiredTargetThrottle = FAILSAFE_CLIMB_THROTTLE_US; // 1800 us
        } else if (activeFailsafeStage == FS_STAGE_LOITER) {
          // Stage 2: Adjust throttle smoothly by 1us every 100ms based on altitude
          if (nowMs - lastThrottleAltAdjustTimeMs >= 100) {
            lastThrottleAltAdjustTimeMs = nowMs;
            if (curAlt > FAILSAFE_LOITER_ALT_TARGET_M) {
              failsafeDynamicThrottleUs -= 1;
            } else if (curAlt < FAILSAFE_LOITER_ALT_TARGET_M) {
              failsafeDynamicThrottleUs += 1;
            }
            failsafeDynamicThrottleUs = constrain(failsafeDynamicThrottleUs, FAILSAFE_LOITER_MIN_THROTTLE_US, THROTTLE_MAX_PULSE);
          }
          desiredTargetThrottle = failsafeDynamicThrottleUs;
        } else {
          // Stage 3: Smooth descent ramping down 1us every second
          if (nowMs - lastStage3StepTimeMs >= 1000) {
            lastStage3StepTimeMs = nowMs;
            if (failsafeDynamicThrottleUs > THROTTLE_MIN_PULSE) {
              failsafeDynamicThrottleUs -= 1;
            }
            failsafeDynamicThrottleUs = constrain(failsafeDynamicThrottleUs, THROTTLE_MIN_PULSE, THROTTLE_MAX_PULSE);
          }
          desiredTargetThrottle = failsafeDynamicThrottleUs;
        }

        // Linear slew rate ramps for smooth attitude and throttle transitions
        if (currentSlewThrottle < desiredTargetThrottle) {
          currentSlewThrottle = min(desiredTargetThrottle, currentSlewThrottle + FAILSAFE_THROTTLE_SLEW_US_PER_TICK);
        } else if (currentSlewThrottle > desiredTargetThrottle) {
          currentSlewThrottle = max(desiredTargetThrottle, currentSlewThrottle - FAILSAFE_THROTTLE_SLEW_US_PER_TICK);
        }

        if (currentSlewTargetPitch < desiredTargetPitch) {
          currentSlewTargetPitch = min(desiredTargetPitch, currentSlewTargetPitch + FAILSAFE_PITCH_SLEW_DEG_PER_TICK);
        } else if (currentSlewTargetPitch > desiredTargetPitch) {
          currentSlewTargetPitch = max(desiredTargetPitch, currentSlewTargetPitch - FAILSAFE_PITCH_SLEW_DEG_PER_TICK);
        }

        if (currentSlewTargetRoll < desiredTargetRoll) {
          currentSlewTargetRoll = min(desiredTargetRoll, currentSlewTargetRoll + FAILSAFE_ROLL_SLEW_DEG_PER_TICK);
        } else if (currentSlewTargetRoll > desiredTargetRoll) {
          currentSlewTargetRoll = max(desiredTargetRoll, currentSlewTargetRoll - FAILSAFE_ROLL_SLEW_DEG_PER_TICK);
        }

        targetThrottle = currentSlewThrottle;
        if (isLowVoltageCutoffTriggered() && targetThrottle > THROTTLE_LOW_VOLT_CEILING_PULSE) {
          targetThrottle = THROTTLE_LOW_VOLT_CEILING_PULSE;
        }

        currentTargetPitch = currentSlewTargetPitch;
        currentTargetRoll = currentSlewTargetRoll;

        // Closed-loop PI-D attitude control with nominal gains
        currentActivePitchKp = PID_PITCH_KP;
        currentActivePitchKi = PID_PITCH_KI;
        currentActivePitchKd = PID_PITCH_KD;
        currentActiveRollKp = PID_ROLL_KP;
        currentActiveRollKi = PID_ROLL_KI;
        currentActiveRollKd = PID_ROLL_KD;

        float pitchError = currentTargetPitch - curPitch;
        if (fabsf(curPitch) <= 45.0f && fabsf(curRoll) <= 60.0f) {
          pitchIntegrator += currentActivePitchKi * pitchError * dt;
          pitchIntegrator = constrain(pitchIntegrator, -MAX_INTEGRAL_PULSE_PITCH_US, MAX_INTEGRAL_PULSE_PITCH_US);
        } else {
          pitchIntegrator = 0.0f;
        }
        float pitchPidOut = (currentActivePitchKp * pitchError) + pitchIntegrator - (currentActivePitchKd * pitchRateDegS);
        int pitchDiff = -constrain((int)pitchPidOut, -anglePulseLimit, anglePulseLimit);

        float rollError = currentTargetRoll - curRoll;
        if (fabsf(curRoll) <= 60.0f) {
          rollIntegrator += currentActiveRollKi * rollError * dt;
          rollIntegrator = constrain(rollIntegrator, -MAX_INTEGRAL_PULSE_ROLL_US, MAX_INTEGRAL_PULSE_ROLL_US);
        } else {
          rollIntegrator = 0.0f;
        }
        float rollPidOut = (currentActiveRollKp * rollError) + rollIntegrator - (currentActiveRollKd * rollRateDegS);
        int rollDiff = constrain((int)rollPidOut, -anglePulseLimit, anglePulseLimit);

        // V-Tail & Rollerons Actuator Mixer (Flaperons retracted in failsafe)
        int pitchOffsetBR = pitchDiff;
        int pitchOffsetBL = -pitchDiff;
        int rollOffsetFR = -rollDiff;
        int rollOffsetFL = -rollDiff;

        targetBR = constrain(neutralBR + pitchOffsetBR, neutralBR - anglePulseLimit, neutralBR + anglePulseLimit);
        targetBL = constrain(neutralBL + pitchOffsetBL, neutralBL - anglePulseLimit, neutralBL + anglePulseLimit);
        targetFR = constrain(neutralFR + rollOffsetFR, neutralFR - anglePulseLimit, neutralFR + anglePulseLimit);
        targetFL = constrain(neutralFL + rollOffsetFL, neutralFL - anglePulseLimit, neutralFL + anglePulseLimit);

        targetBR = constrain(targetBR, 1000, 2000);
        targetBL = constrain(targetBL, 1000, 2000);
        targetFR = constrain(targetFR, 1000, 2000);
        targetFL = constrain(targetFL, 1000, 2000);
      }
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
    // Soft power ceiling: allow up to ~35-40% throttle (1350us) to maintain flight and glide back safely
    if (pulseWidthUs > THROTTLE_LOW_VOLT_CEILING_PULSE) {
      pulseWidthUs = THROTTLE_LOW_VOLT_CEILING_PULSE;
    }
  }
  if (pulseWidthUs >= THROTTLE_MIN_PULSE &&
      pulseWidthUs <= THROTTLE_MAX_PULSE) {
    if (pulseWidthUs != lastWritePulseUs[4]) {
      escMotor.writeMicroseconds(pulseWidthUs);
      portENTER_CRITICAL(&actuatorMux);
      lastWritePulseUs[4] = pulseWidthUs;
      portEXIT_CRITICAL(&actuatorMux);
    }
    return true;
  }
  return false;
}

void emergencyCutoffESC() {
  if (lastWritePulseUs[4] != THROTTLE_MIN_PULSE) {
    escMotor.writeMicroseconds(THROTTLE_MIN_PULSE);
    portENTER_CRITICAL(&actuatorMux);
    lastWritePulseUs[4] = THROTTLE_MIN_PULSE;
    portEXIT_CRITICAL(&actuatorMux);
  }
}

int getCurrentThrottlePulse() {
  portENTER_CRITICAL(&actuatorMux);
  int val = lastWritePulseUs[4];
  portEXIT_CRITICAL(&actuatorMux);
  return val;
}

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

  resetExtremumSeeking(true); // Cold boot initialization
  wasThrottleAbove1600Ever = false;

  xTaskCreatePinnedToCore(controlTaskLoop, "ControlTask", 4096, NULL, 5, NULL,
                          0);
}
