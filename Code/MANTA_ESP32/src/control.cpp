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

static volatile int lastWritePulseUs[5] = {
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

// ── RC EXPO & SCALING UTILITY ───────────────────────────────────────────────
// Applies a cubic exponential response curve to normalized [-1.0, +1.0] stick inputs:
// f(x) = (1 - expo) * x + expo * x^3
// Maps full stick range (1000-2000us) smoothly across [-maxPulseLimitUs, +maxPulseLimitUs]
// with reduced sensitivity near the stick center for fine corrections.
static inline int applyExpoAndScale(int rawStickUs, int centerUs, int maxPulseLimitUs, float expoFactor) {
  float norm = constrain((float)(rawStickUs - centerUs) / 500.0f, -1.0f, 1.0f);
  float shaped = (1.0f - expoFactor) * norm + expoFactor * (norm * norm * norm);
  return (int)(shaped * (float)maxPulseLimitUs);
}

static constexpr uint16_t centerCH1 = 1500;
static constexpr uint16_t centerCH2 = 1500;
static constexpr uint8_t servoMaxAngleDeg =
    DEFAULT_SERVO_MAX_ANGLE_DEG; // Default 20 deg

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

static volatile FlightMode currentFlightMode = FLIGHT_MODE_1;
static volatile bool currentRollActive = false;
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

// Active in-flight PID gains (Pitch PID is completely deactivated for anti-stall safety; Roll remains active)
static volatile float currentActivePitchKp = 0.0f;
static volatile float currentActivePitchKi = 0.0f;
static volatile float currentActivePitchKd = 0.0f;
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
  float norm = constrain((float)(rawStickUs - centerUs) / 500.0f, -1.0f, 1.0f);
  float shaped = (1.0f - expoFactor) * norm + expoFactor * (norm * norm * norm);
  return shaped * maxDeg;
}

void decodeCH5(uint16_t ch5Pulse, FlightMode &mode, bool &rollActive) {
  // SWC (3-pos switch) + SWB (2-pos switch) calibrado:
  // SWC 1 + SWB OFF: ~1076 us -> Modo 1 + Roll OFF (100% Manual Direto)
  // SWC 1 + SWB ON:  ~1288 us -> Modo 1 + Roll ON  (Manual + Roll Assist)
  // SWC 2 + SWB OFF: ~1400 us -> Modo 2 + Roll OFF (ESC Pitch FBW + Roll Manual)
  // SWC 2 + SWB ON:  ~1530 us -> Modo 2 + Roll ON  (ESC PI-D Completo Pitch & Roll)
  // SWC 3 + SWB OFF: ~1693 us -> Modo 3 + Roll OFF (FBW Fixo Pitch + Roll Manual)
  // SWC 3 + SWB ON:  ~1760 us -> Modo 3 + Roll ON  (FBW Fixo Completo Pitch & Roll)
  if (ch5Pulse < 1180) {
    mode = FLIGHT_MODE_1;
    rollActive = false;
  } else if (ch5Pulse < 1345) {
    mode = FLIGHT_MODE_1;
    rollActive = true;
  } else if (ch5Pulse < 1465) {
    mode = FLIGHT_MODE_2;
    rollActive = false;
  } else if (ch5Pulse < 1610) {
    mode = FLIGHT_MODE_2;
    rollActive = true;
  } else if (ch5Pulse < 1725) {
    mode = FLIGHT_MODE_3;
    rollActive = false;
  } else {
    mode = FLIGHT_MODE_3;
    rollActive = true;
  }
}

void decodeCH5WithHysteresis(uint16_t ch5Pulse, FlightMode currentMode, bool currentRoll, FlightMode &outMode, bool &outRoll) {
  // Rejeita pulsos fora do range válido de RC (ruído de EMI extremo ou canal desconectado)
  if (ch5Pulse < 850 || ch5Pulse > 2150) {
    outMode = currentMode;
    outRoll = currentRoll;
    return;
  }

  // Mapeia estado atual: 0 a 5
  // Map current active state:
  // 0: Modo 1 OFF (~1166 us), 1: Modo 2 OFF (~1328 us)
  // 2: Modo 3 OFF (~1411 us), 3: Modo 1 ON (~1541 us)
  // 4: Modo 2 ON (~1825 us),  5: Modo 3 ON (~1942 us)
  uint8_t curState = 0;
  if (currentMode == FLIGHT_MODE_1) {
    curState = currentRoll ? 3 : 0;
  } else if (currentMode == FLIGHT_MODE_2) {
    curState = currentRoll ? 4 : 1;
  } else {
    curState = currentRoll ? 5 : 2;
  }

  uint8_t nextState = curState;

  // Direct midpoint decoding with symmetric hysteresis (+/- 3us to +/- 5us):
  // State 0 (Mode 1 OFF): nominal 1166 us (< 1247)
  // State 1 (Mode 2 OFF): nominal 1328 us (1247 - 1370)
  // State 2 (Mode 3 OFF): nominal 1411 us (1370 - 1476)
  // State 3 (Mode 1 ON):  nominal 1541 us (1476 - 1683)
  // State 4 (Mode 2 ON):  nominal 1825 us (1683 - 1884)
  // State 5 (Mode 3 ON):  nominal 1942 us (>= 1884)
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
    case 0: outMode = FLIGHT_MODE_1; outRoll = false; break;
    case 1: outMode = FLIGHT_MODE_2; outRoll = false; break;
    case 2: outMode = FLIGHT_MODE_3; outRoll = false; break;
    case 3: outMode = FLIGHT_MODE_1; outRoll = true;  break;
    case 4: outMode = FLIGHT_MODE_2; outRoll = true;  break;
    case 5: default: outMode = FLIGHT_MODE_3; outRoll = true;  break;
  }
}

FlightMode getCurrentFlightMode() {
  return currentFlightMode;
}

bool isRollActive() {
  return currentRollActive;
}

void getActuatorOutputs(int &br, int &bl, int &fr, int &fl, int &throttle, bool &assistActive) {
  br = lastWritePulseUs[0];
  bl = lastWritePulseUs[1];
  fr = lastWritePulseUs[2];
  fl = lastWritePulseUs[3];
  throttle = lastWritePulseUs[4];
  assistActive = currentRollActive;
}

void getActuatorOutputs(int &br, int &bl, int &fr, int &fl, int &throttle, FlightMode &flightMode, bool &rollActive) {
  br = lastWritePulseUs[0];
  bl = lastWritePulseUs[1];
  fr = lastWritePulseUs[2];
  fl = lastWritePulseUs[3];
  throttle = lastWritePulseUs[4];
  flightMode = currentFlightMode;
  rollActive = currentRollActive;
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

    // Symmetric angular limit in us around each servo's own trimmed neutral
    int anglePulseLimit = (int)(servoMaxAngleDeg * US_PER_DEGREE);

    uint16_t c1 = (ch1 > 0) ? ch1 : 1500;
    uint16_t c2 = (ch2 > 0) ? ch2 : 1500;
    uint16_t c3 = (ch3 > 0) ? ch3 : 1000;
    uint16_t c5 = (ch5 > 0) ? ch5 : 0;

    // ── DECODIFICAÇÃO CH5 COM HISTERESE SCHMITT-TRIGGER & DEBOUNCE TEMPORAL ──
    // Só avalia e atualiza o candidato de modo se o sinal RC estiver ativo e houver pulso válido no CH5.
    // Durante failsafe / perda de sinal, a máquina de modos congela o último estado válido do comando.
    static FlightMode pendingMode = FLIGHT_MODE_1;
    static bool pendingRoll = false;
    static uint8_t debounceCount = 0;
    static bool firstLoopTick = true;
    static bool lastRollActive = false;

    if (!rcSignalLost && c5 > 0) {
      FlightMode candidateMode = currentFlightMode;
      bool candidateRoll = currentRollActive;
      decodeCH5WithHysteresis(c5, currentFlightMode, currentRollActive, candidateMode, candidateRoll);

      if (firstLoopTick) {
        pendingMode = candidateMode;
        pendingRoll = candidateRoll;
        currentFlightMode = candidateMode;
        currentRollActive = candidateRoll;
        currentAssistModeActive = candidateRoll;
        lastFlightMode = candidateMode;
        lastRollActive = candidateRoll;
        debounceCount = CH5_DEBOUNCE_CONFIRM_TICKS;
      } else {
        if (candidateMode == pendingMode && candidateRoll == pendingRoll) {
          if (debounceCount < CH5_DEBOUNCE_CONFIRM_TICKS) {
            debounceCount++;
            if (debounceCount >= CH5_DEBOUNCE_CONFIRM_TICKS) {
              currentFlightMode = candidateMode;
              currentRollActive = candidateRoll;
              currentAssistModeActive = candidateRoll;
            }
          }
        } else {
          pendingMode = candidateMode;
          pendingRoll = candidateRoll;
          debounceCount = 1;
        }
      }
    }

    FlightMode flightMode = currentFlightMode;
    bool rollActive = currentRollActive;

    // Safety Failsafe: if MPU6050 IMU is offline/unavailable, force fallback to Mode 1 (Manual)
    // to prevent integrator windup and uncontrolled dive!
    bool mpuHealthy = isMPU6050Available();
    if (!mpuHealthy) {
      flightMode = FLIGHT_MODE_1;
      rollActive = false;
      resetControlIntegrators();
      resetExtremumSeeking(false);
    }

    // Reset integrators and ESC on flight mode change (bumpless transfer)
    // Preserva os ganhos aprendidos escPitchThetaHat e escRollThetaHat para partilha entre Modo 2 e Modo 3
    bool modeJustChanged = false;
    if (flightMode != lastFlightMode) {
      resetControlIntegrators();
      resetExtremumSeeking(false);
      lastFlightMode = flightMode;
      lastRollActive = rollActive;
      modeJustChanged = true;
    } else if (!lastRollActive && rollActive) {
      // SWB toggled ON (Roll Assist/FBW/ESC engaged) within same flight mode
      rollIntegrator = 0.0f;
      escRollTimeSec = 0.0f;
      escRollHPF = 0.0f;
      escRollFirstTick = true;
      lastRollActive = rollActive;
    } else if (lastRollActive && !rollActive) {
      // SWB toggled OFF (Roll Manual engaged) within same flight mode
      rollIntegrator = 0.0f;
      escRollTimeSec = 0.0f;
      escRollHPF = 0.0f;
      escRollFirstTick = true;
      lastRollActive = rollActive;
    }

    // Read attitude estimation from MPU6050
    float curPitch = 0.0f, curRoll = 0.0f;
    getFilteredMPUData(curPitch, curRoll);

    // Initial startup check and mode switch: initialize lastPitchMeas / lastRollMeas to eliminate derivative kick
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

      int pitchDiff = 0;
      int rollDiff = 0;

      // ── MODO 1: 100% MANUAL (COM OU SEM ROLL ASSIST) ──────────────────────
      if (flightMode == FLIGHT_MODE_1) {
        escIsActive = false;
        float rollScale = constrain(escRollThetaHat, ESC_ROLL_MIN_SCALE, ESC_ROLL_MAX_SCALE);
        escCurrentPitchScale = 1.0f;
        escCurrentRollScale = rollScale;

        // Pitch: Manual direto com expo padrão
        pitchDiff = applyExpoAndScale((int)c2, (int)centerCH2, anglePulseLimit, RC_EXPO_FACTOR);
        currentTargetPitch = 0.0f;

        currentActivePitchKp = 0.0f;
        currentActivePitchKi = 0.0f;
        currentActivePitchKd = 0.0f;
        currentActiveRollKp = PID_ROLL_KP * rollScale;
        currentActiveRollKi = PID_ROLL_KI;
        currentActiveRollKd = PID_ROLL_KD * sqrtf(rollScale);

        if (rollActive) {
          // Roll ON (SWB ON, 1288 us): Roll Assist / Envelope Protection
          float normC1 = constrain((float)((int)c1 - (int)centerCH1) / 500.0f, -1.0f, 1.0f);
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
          // Roll OFF (SWB OFF, 1076 us): Manual direto 100%
          rollDiff = applyExpoAndScale((int)c1, (int)centerCH1, anglePulseLimit, RC_EXPO_FACTOR);
        }
        currentTargetRoll = 0.0f;
        resetControlIntegrators();
      }

      // ── MODO 2: FLY-BY-WIRE COM GANHOS PARTILHADOS / APREENDIDOS (MODO 3 SHARING) ──
      else if (flightMode == FLIGHT_MODE_2) {
        escIsActive = false; // Modo 2 opera como FBW fixo (sem dither) com os ganhos otimizados do Modo 3

        float rollScale = constrain(escRollThetaHat, ESC_ROLL_MIN_SCALE, ESC_ROLL_MAX_SCALE);
        escCurrentPitchScale = 1.0f;
        escCurrentRollScale = rollScale;

        float activeRollKp = PID_ROLL_KP * rollScale;
        float activeRollKi = PID_ROLL_KI;
        float activeRollKd = PID_ROLL_KD * sqrtf(rollScale);

        // Desativação total do PID de Pitch para segurança anti-stall:
        // O profundor é 100% manual direto pelo stick CH2 com a mesma curva do Modo 1
        currentActivePitchKp = 0.0f;
        currentActivePitchKi = 0.0f;
        currentActivePitchKd = 0.0f;
        currentActiveRollKp = activeRollKp;
        currentActiveRollKi = activeRollKi;
        currentActiveRollKd = activeRollKd;

        // 1. PITCH: CONTROLO MANUAL DIRETO 100% (Pitch PID Desativado por Segurança contra Stall)
        pitchDiff = applyExpoAndScale((int)c2, (int)centerCH2, anglePulseLimit, RC_EXPO_FACTOR);
        currentTargetPitch = 0.0f;
        pitchIntegrator = 0.0f;

        // 2. ROLL FBW:
        if (rollActive) {
          float targetRollDeg = applyExpoAndScaleDeg((int)c1, (int)centerCH1, FBW_MAX_ROLL_DEG, FBW_EXPO_FACTOR);
          currentTargetRoll = targetRollDeg;

          float rollError = targetRollDeg - curRoll;

          if (c3 > 1050) {
            rollIntegrator += activeRollKi * rollError * dt;
            rollIntegrator = constrain(rollIntegrator, -MAX_INTEGRAL_PULSE_US, MAX_INTEGRAL_PULSE_US);
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

      // ── MODO 3: FLY-BY-WIRE COM APERFEIÇOAMENTO ADAPTATIVO (EXTREMUM SEEKING PI-D) ──
      else {
        escIsActive = rollActive; // Extremum Seeking ativo exclusivamente no eixo de Roll quando rollActive está ligado

        // 1. PITCH: CONTROLO MANUAL DIRETO 100% (Pitch PID e Calibração/ESC Desativados por Segurança contra Stall)
        pitchDiff = applyExpoAndScale((int)c2, (int)centerCH2, anglePulseLimit, RC_EXPO_FACTOR);
        currentTargetPitch = 0.0f;
        pitchIntegrator = 0.0f;

        // Desativação total dos ganhos de Pitch e congelamento da perturbação
        currentActivePitchKp = 0.0f;
        currentActivePitchKi = 0.0f;
        currentActivePitchKd = 0.0f;
        escCurrentPitchScale = 1.0f;
        escPitchTimeSec = 0.0f;
        escPitchHPF = 0.0f;
        escPitchFirstTick = true;

        // 2. ROLL FBW COM EXTREMUM SEEKING (Se Roll Active):
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
          float activeRollKd = PID_ROLL_KD * sqrtf(effectiveRollTheta); // Preserva amortecimento natural zeta
          float activeRollKi = PID_ROLL_KI;

          currentActiveRollKp = activeRollKp;
          currentActiveRollKi = activeRollKi;
          currentActiveRollKd = activeRollKd;

          if (c3 > 1050) {
            rollIntegrator += activeRollKi * rollError * dt;
            rollIntegrator = constrain(rollIntegrator, -MAX_INTEGRAL_PULSE_US, MAX_INTEGRAL_PULSE_US);
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
          // Roll OFF: Roll em modo manual direto
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

  resetExtremumSeeking(true); // Cold boot initialization

  xTaskCreatePinnedToCore(controlTaskLoop, "ControlTask", 4096, NULL, 5, NULL,
                          0);
}
