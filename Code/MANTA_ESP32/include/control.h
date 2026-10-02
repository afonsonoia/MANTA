#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>

// Flight Modes selected via CH5 (SWC 3-position switch: Pos 1 -> Mode 1, Pos 2 -> Mode 2, Pos 3 -> Mode 3)
enum FlightMode : uint8_t {
  FLIGHT_MODE_1 = 1,
  FLIGHT_MODE_2 = 2,
  FLIGHT_MODE_3 = 3
};

// Autonomous Emergency Failsafe Stages on RC Signal Loss
enum FailsafeStage : uint8_t {
  FS_STAGE_INACTIVE = 0, // Normal RC flight active
  FS_STAGE_GROUND   = 1, // RC lost on ground/bench: motor OFF, surfaces neutral
  FS_STAGE_CLIMB    = 2, // Stage 1: Closed-loop climb
  FS_STAGE_LOITER   = 3, // Stage 2: Closed-loop loiter at roll -30 deg, pitch 5 deg, dynamic 25m alt hold
  FS_STAGE_DESCEND  = 4  // Stage 3: Loiter at roll -30 deg, pitch 5 deg, continuous 1us/s throttle decay (timeout >30s or baro failure)
};

FailsafeStage getActiveFailsafeStage();

void initControlSystem();
bool setThrottlePulse(int pulseWidthUs);
void emergencyCutoffESC();
int getCurrentThrottlePulse();
void getActuatorOutputs(int &br, int &bl, int &fr, int &fl, int &throttle, bool &flaperonActive);
void getActuatorOutputs(int &br, int &bl, int &fr, int &fl, int &throttle, FlightMode &flightMode, bool &flaperonActive);
FlightMode getCurrentFlightMode();
bool isRollActive();
bool isFlaperonActive();
void decodeCH5(uint16_t ch5Pulse, FlightMode &mode, bool &flaperonActive);
void decodeCH5WithHysteresis(uint16_t ch5Pulse, FlightMode currentMode, bool currentFlaperon, FlightMode &outMode, bool &outFlaperon);
void resetControlIntegrators();
void getFBWTargets(float &targetPitch, float &targetRoll);
bool isExtremumSeekingActive();
void getExtremumSeekingScale(float &pitchScale, float &rollScale);
void resetExtremumSeeking(bool resetLearnedGains = false);
void getActivePIDGains(float &pitchKp, float &pitchKi, float &pitchKd,
                       float &rollKp, float &rollKi, float &rollKd);

#endif // CONTROL_H
