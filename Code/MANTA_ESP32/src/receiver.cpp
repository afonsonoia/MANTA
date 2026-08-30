#include "receiver.h"
#include "config.h"

// Raw pulse durations directly from ISR: 0 = no signal received yet
static volatile uint16_t rawChannelVector[5] = {0, 0, 0, 0, 0};
static volatile uint16_t savedChannelVector[5] = {0, 0, 0, 0, 0};
static volatile uint8_t rcMarginDeadband = DEFAULT_RC_MARGIN_DEADBAND;

void setRCMarginDeadband(uint8_t deadbandUs) {
  if (deadbandUs < 1) deadbandUs = 1;
  if (deadbandUs > 50) deadbandUs = 50;
  rcMarginDeadband = deadbandUs;
}

uint8_t getRCMarginDeadband() {
  return rcMarginDeadband;
}

// Last rising edge timestamps for each channel
static volatile uint32_t rcStartUs[5] = {0, 0, 0, 0, 0};
static volatile uint32_t lastRcPulseMicros = 0;

bool isRCSignalLost() {
  if (lastRcPulseMicros == 0) {
    return (millis() > 3000); // 3s grace period after boot
  }
  return (micros() - lastRcPulseMicros > 600000); // Failsafe: > 600ms without RC pulse
}

bool isRCQuietPeriod() {
  if (isRCSignalLost() || lastRcPulseMicros == 0) return true;
  return (micros() - lastRcPulseMicros >= 2500);
}

uint16_t getNoiseFloorUs() {
  return 10;
}

static inline void processChannelSample(uint8_t index, uint32_t dt) {
  // Valid RC PWM pulse range (850–2150us)
  if (dt < 850 || dt > 2150) return;

  lastRcPulseMicros = micros();

  // 1. 3-Sample Circular Buffer for Median Filtering (eliminates single-sample interrupt spikes & jitter)
  static uint16_t medBuf[5][3] = {{0}};
  static uint8_t medIdx[5] = {0};
  medBuf[index][medIdx[index]] = (uint16_t)dt;
  medIdx[index] = (medIdx[index] + 1) % 3;

  // Fast Median-of-3 calculation
  uint16_t a = medBuf[index][0], b = medBuf[index][1], c = medBuf[index][2];
  uint16_t medianVal;
  if (a == 0 || b == 0 || c == 0) {
    medianVal = (uint16_t)dt;
  } else {
    medianVal = (a > b) ? ((b > c) ? b : ((a > c) ? c : a))
                        : ((a > c) ? a : ((b > c) ? c : b));
  }

  rawChannelVector[index] = medianVal;

  // 2. Deadband / Hysteresis: Locks value when stationary; updates instantly (0 ms lag) on real stick movement
  uint16_t current = savedChannelVector[index];
  uint8_t db = (rcMarginDeadband > 0) ? rcMarginDeadband : 4;
  if (current == 0 || abs((int)medianVal - (int)current) >= (int)db) {
    savedChannelVector[index] = medianVal;
  }
}

// ISR Handlers for each RC channel
void IRAM_ATTR isrCH1() {
  uint32_t now = micros();
  if (digitalRead(PIN_RC_CH1) == HIGH) {
    rcStartUs[0] = now;
  } else if (rcStartUs[0] > 0) {
    uint32_t dur = now - rcStartUs[0];
    rcStartUs[0] = 0;
    processChannelSample(0, dur);
  }
}

void IRAM_ATTR isrCH2() {
  uint32_t now = micros();
  if (digitalRead(PIN_RC_CH2) == HIGH) {
    rcStartUs[1] = now;
  } else if (rcStartUs[1] > 0) {
    uint32_t dur = now - rcStartUs[1];
    rcStartUs[1] = 0;
    processChannelSample(1, dur);
  }
}

void IRAM_ATTR isrCH3() {
  uint32_t now = micros();
  if (digitalRead(PIN_RC_CH3) == HIGH) {
    rcStartUs[2] = now;
  } else if (rcStartUs[2] > 0) {
    uint32_t dur = now - rcStartUs[2];
    rcStartUs[2] = 0;
    processChannelSample(2, dur);
  }
}

void IRAM_ATTR isrCH4() {
  uint32_t now = micros();
  if (digitalRead(PIN_RC_CH4) == HIGH) {
    rcStartUs[3] = now;
  } else if (rcStartUs[3] > 0) {
    uint32_t dur = now - rcStartUs[3];
    rcStartUs[3] = 0;
    processChannelSample(3, dur);
  }
}

void IRAM_ATTR isrCH5() {
  uint32_t now = micros();
  if (digitalRead(PIN_RC_CH5) == HIGH) {
    rcStartUs[4] = now;
  } else if (rcStartUs[4] > 0) {
    uint32_t dur = now - rcStartUs[4];
    rcStartUs[4] = 0;
    processChannelSample(4, dur);
  }
}

void initReceiver() {
  pinMode(PIN_RC_CH1, INPUT_PULLDOWN);
  pinMode(PIN_RC_CH2, INPUT_PULLDOWN);
  pinMode(PIN_RC_CH3, INPUT_PULLDOWN);
  pinMode(PIN_RC_CH4, INPUT_PULLDOWN);
  pinMode(PIN_RC_CH5, INPUT_PULLDOWN);

  attachInterrupt(digitalPinToInterrupt(PIN_RC_CH1), isrCH1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_CH2), isrCH2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_CH3), isrCH3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_CH4), isrCH4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RC_CH5), isrCH5, CHANGE);
}

void getReceiverChannels(uint16_t &ch1, uint16_t &ch2, uint16_t &ch3) {
  noInterrupts();
  uint16_t v1 = savedChannelVector[0];
  uint16_t v2 = savedChannelVector[1];
  uint16_t v3 = savedChannelVector[2];
  interrupts();

  if (isRCSignalLost()) {
    ch1 = 0; ch2 = 0; ch3 = 0;
    return;
  }

  ch1 = (v1 > 0) ? constrain(v1, 1000, 2000) : 0;
  ch2 = (v2 > 0) ? constrain(v2, 1000, 2000) : 0;
  ch3 = (v3 > 0) ? constrain(v3, 1000, 2000) : 0;
}

void getReceiverChannels(uint16_t &ch1, uint16_t &ch2, uint16_t &ch3, uint16_t &ch5) {
  noInterrupts();
  uint16_t v1 = savedChannelVector[0];
  uint16_t v2 = savedChannelVector[1];
  uint16_t v3 = savedChannelVector[2];
  uint16_t v5 = savedChannelVector[4];
  interrupts();

  if (isRCSignalLost()) {
    ch1 = 0; ch2 = 0; ch3 = 0; ch5 = 0;
    return;
  }

  ch1 = (v1 > 0) ? constrain(v1, 1000, 2000) : 0;
  ch2 = (v2 > 0) ? constrain(v2, 1000, 2000) : 0;
  ch3 = (v3 > 0) ? constrain(v3, 1000, 2000) : 0;
  ch5 = (v5 > 0) ? constrain(v5, 1000, 2000) : 0;
}

void getReceiverChannels(uint16_t &ch1, uint16_t &ch2, uint16_t &ch3, uint16_t &ch4, uint16_t &ch5) {
  noInterrupts();
  uint16_t v1 = savedChannelVector[0];
  uint16_t v2 = savedChannelVector[1];
  uint16_t v3 = savedChannelVector[2];
  uint16_t v4 = savedChannelVector[3];
  uint16_t v5 = savedChannelVector[4];
  interrupts();

  if (isRCSignalLost()) {
    ch1 = 0; ch2 = 0; ch3 = 0; ch4 = 0; ch5 = 0;
    return;
  }

  ch1 = (v1 > 0) ? constrain(v1, 1000, 2000) : 0;
  ch2 = (v2 > 0) ? constrain(v2, 1000, 2000) : 0;
  ch3 = (v3 > 0) ? constrain(v3, 1000, 2000) : 0;
  ch4 = (v4 > 0) ? constrain(v4, 1000, 2000) : 0;
  ch5 = (v5 > 0) ? constrain(v5, 1000, 2000) : 0;
}

void getRawReceiverChannels(uint16_t &ch1, uint16_t &ch2, uint16_t &ch3, uint16_t &ch4, uint16_t &ch5) {
  getReceiverChannels(ch1, ch2, ch3, ch4, ch5);
}
