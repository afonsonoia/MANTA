#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>

// Flight Modes selected via CH5 (SWC 3-position switch: Pos 1 -> Mode 1, Pos 2 -> Mode 2, Pos 3 -> Mode 3)
enum FlightMode : uint8_t {
  FLIGHT_MODE_1 = 1,
  FLIGHT_MODE_2 = 2,
  FLIGHT_MODE_3 = 3
};

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
