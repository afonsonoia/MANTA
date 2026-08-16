#include "receiver.h"
#include "config.h"

#define MAX_FILTER_WINDOW 100

// Saved channel vector: 0 = no signal received yet (ISR not triggered)
static volatile uint16_t savedChannelVector[5] = {0, 0, 0, 0, 0};

// Dynamic RC Margin Deadband (default 4us)
static volatile uint8_t rcMarginDeadband = DEFAULT_RC_MARGIN_DEADBAND;

// Dynamic RC Noise Filter Configuration
static volatile uint8_t rcFilterType = RC_FILTER_NONE; // Default: Raw (no filter)
static volatile uint16_t rcFilterWindow = 5;            // Window size (used only when filter is active)
static volatile float rcFilterAlpha = 0.33f;            // EMA alpha (used only when filter is active)

// Ring buffers for SMA/WMA and state for EMA per channel (0..4)
static uint16_t channelHistory[5][MAX_FILTER_WINDOW];
static uint16_t historyHead[5] = {0, 0, 0, 0, 0};
static uint16_t historyCount[5] = {0, 0, 0, 0, 0};
static uint16_t emaState[5] = {1500, 1500, 1000, 1500, 1500};

void setRCFilterConfig(uint8_t filterType, uint16_t windowSize, float alpha) {
  if (filterType > 3) filterType = 1;
  if (windowSize < 1) windowSize = 1;
  if (windowSize > 20) windowSize = 20;
  if (alpha < 0.01f) alpha = 0.01f;
  if (alpha > 1.00f) alpha = 1.00f;

  rcFilterType = filterType;
  rcFilterWindow = windowSize;
  rcFilterAlpha = alpha;
}

void getRCFilterConfig(uint8_t &filterType, uint16_t &windowSize, float &alpha) {
  filterType = rcFilterType;
  windowSize = rcFilterWindow;
  alpha = rcFilterAlpha;
}

void setRCMarginDeadband(uint8_t deadbandUs) {
  if (deadbandUs < 1) deadbandUs = 1;
  if (deadbandUs > 50) deadbandUs = 50;
  if (rcMarginDeadband != deadbandUs) {
    rcMarginDeadband = deadbandUs;
  }
}

uint8_t getRCMarginDeadband() {
  return rcMarginDeadband;
}

// Last rising edge timestamps for each channel
static volatile uint32_t rcStartUs[5] = {0, 0, 0, 0, 0};
static volatile uint32_t lastRcPulseMicros = 0;

bool isRCSignalLost() {
  if (lastRcPulseMicros == 0) {
    return (millis() > 2000); // Failsafe: if 2s pass after boot without any RC pulse, signal is lost
  }
  return (micros() - lastRcPulseMicros > 500000); // Failsafe: > 500ms without RC pulse
}

bool isRCQuietPeriod() {
  if (isRCSignalLost() || lastRcPulseMicros == 0) return true;

  // Verify all 5 RC channel input pins are LOW (no active pulse transmission)
  if (digitalRead(PIN_RC_CH1) == HIGH ||
      digitalRead(PIN_RC_CH2) == HIGH ||
      digitalRead(PIN_RC_CH3) == HIGH ||
      digitalRead(PIN_RC_CH4) == HIGH ||
      digitalRead(PIN_RC_CH5) == HIGH) {
    return false; // Active pulse in progress!
  }

  uint32_t elapsed = micros() - lastRcPulseMicros;
  // Guaranteed quiet idle gap between 2.5ms (2500us) and 16.5ms (16500us) after last pulse
  return (elapsed >= 2500 && elapsed <= 16500);
}

// CH5-based adaptive noise floor estimator.
// CH5 is a binary switch (true values: ~1000us LOW or ~2000us HIGH).
// Any deviation from those two values is pure measurable noise.
static volatile uint16_t noiseFloorUs = 10; // Starts at 10us (clean environment)

uint16_t getNoiseFloorUs() {
  return noiseFloorUs;
}

// Outlier rejection state memory per channel (0..4)
static uint16_t spikeCandidate[5] = {0, 0, 0, 0, 0};
static uint8_t spikeCount[5] = {0, 0, 0, 0, 0};

static inline void processChannelSample(uint8_t index, uint32_t dt) {
  // 1. Sanity check: valid RC PWM pulse range (900–2100us)
  if (dt < 900 || dt > 2100) return;

  // CH5 (index 4) is a binary switch (~1000us LOW or ~2000us HIGH).
  // Strictly filter out any middle floating values or electrical noise < 800 or > 2200.
  if (index == 4) {
    if (dt > 1300 && dt < 1700) return; // Ignore invalid floating switch state
    uint16_t ch5_true = (dt < 1500) ? 1000 : 2000;
    uint16_t noise = (dt > ch5_true) ? (uint16_t)(dt - ch5_true) : (uint16_t)(ch5_true - dt);
    // Integer EMA (alpha=1/8): ~8-sample noise environment memory
    noiseFloorUs = (uint16_t)((noiseFloorUs * 7 + noise) / 8);
  }

  // 2. Single-Frame Outlier / Spike Rejection (max allowed single-frame jump = 80us)
  uint16_t baseline = savedChannelVector[index];
  if (baseline > 0 && index != 4) { // Apply spike filter to CH1..CH4
    int stepDiff = (int)dt - (int)baseline;
    if (stepDiff > 80 || stepDiff < -80) {
      // Single frame sudden jump > 80us: potential electrical noise spike!
      if (spikeCount[index] == 0) {
        // First frame of a jump: store candidate and ignore this spike frame
        spikeCandidate[index] = (uint16_t)dt;
        spikeCount[index] = 1;
        return;
      } else {
        // Second consecutive frame: check if candidate is consistent with new position
        int candDiff = (int)dt - (int)spikeCandidate[index];
        if (candDiff >= -30 && candDiff <= 30) {
          // Real intentional stick movement: accept new position!
          spikeCount[index] = 0;
        } else {
          // Continuous random noise: update candidate and ignore
          spikeCandidate[index] = (uint16_t)dt;
          return;
        }
      }
    } else {
      spikeCount[index] = 0; // Normal continuous movement within 80us limit
    }
  }

  lastRcPulseMicros = micros();

  if (rcFilterType != RC_FILTER_NONE) {
    // Push sample to channel history ring buffer
    uint16_t head = historyHead[index];
    channelHistory[index][head] = (uint16_t)dt;
    historyHead[index] = (head + 1) % MAX_FILTER_WINDOW;
    if (historyCount[index] < MAX_FILTER_WINDOW) {
      historyCount[index]++;
    }
  }

  uint16_t filteredVal = (uint16_t)dt; // Default: raw value

  if (rcFilterType == RC_FILTER_SMA) {
    uint16_t N = min((uint16_t)rcFilterWindow, historyCount[index]);
    if (N == 0) N = 1;
    uint32_t sum = 0;
    for (uint16_t i = 0; i < N; i++) {
      int idx = (historyHead[index] - 1 - i + MAX_FILTER_WINDOW) % MAX_FILTER_WINDOW;
      sum += channelHistory[index][idx];
    }
    filteredVal = (uint16_t)(sum / N);
  } else if (rcFilterType == RC_FILTER_EMA) {
    uint32_t alphaPct = (uint32_t)(rcFilterAlpha * 100.0f);
    if (alphaPct < 1) alphaPct = 1;
    if (alphaPct > 100) alphaPct = 100;
    uint32_t prevEma = (uint32_t)emaState[index];
    uint32_t newEma = (alphaPct * (uint32_t)dt + (100 - alphaPct) * prevEma) / 100;
    emaState[index] = (uint16_t)newEma;
    filteredVal = (uint16_t)newEma;
  } else if (rcFilterType == RC_FILTER_WMA) {
    uint16_t N = min((uint16_t)rcFilterWindow, historyCount[index]);
    if (N == 0) N = 1;
    uint32_t weightedSum = 0;
    uint32_t weightTotal = 0;
    for (uint16_t i = 0; i < N; i++) {
      uint32_t weight = N - i;
      int idx = (historyHead[index] - 1 - i + MAX_FILTER_WINDOW) % MAX_FILTER_WINDOW;
      weightedSum += channelHistory[index][idx] * weight;
      weightTotal += weight;
    }
    filteredVal = (uint16_t)(weightedSum / weightTotal);
  }

  // Apply RC Margin Deadband filter: ignore variations smaller than rcMarginDeadband
  uint16_t currentSaved = savedChannelVector[index];
  if (currentSaved == 0) {
    savedChannelVector[index] = filteredVal;
  } else {
    int diff = (int)filteredVal - (int)currentSaved;
    uint8_t db = rcMarginDeadband;
    if (diff > (int)db || diff < -((int)db)) {
      savedChannelVector[index] = filteredVal;
    }
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
  // Set pin modes for input-only and I/O pins
  pinMode(PIN_RC_CH1, INPUT);
  pinMode(PIN_RC_CH2, INPUT);
  pinMode(PIN_RC_CH3, INPUT);
  pinMode(PIN_RC_CH4, INPUT_PULLDOWN);
  pinMode(PIN_RC_CH5, INPUT_PULLDOWN);

  // Attach non-blocking hardware interrupts on CHANGE
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
  // CH5 is a 2-position switch: >1500 -> 2000us, otherwise 1000us
  ch5 = (v5 > 0) ? ((v5 > 1500) ? 2000 : 1000) : 0;
}

void getReceiverChannels(uint16_t &ch1, uint16_t &ch2, uint16_t &ch3, uint16_t &ch4, uint16_t &ch5) {
  noInterrupts();
  uint16_t v1 = savedChannelVector[0];
  uint16_t v2 = savedChannelVector[1];
  uint16_t v3 = savedChannelVector[2];
  uint16_t v4 = savedChannelVector[3];
  uint16_t v5 = savedChannelVector[4];
  interrupts();

  // If RC signal was lost (>500ms without pulse), return 0 on all channels
  if (isRCSignalLost()) {
    ch1 = 0; ch2 = 0; ch3 = 0; ch4 = 0; ch5 = 0;
    return;
  }

  // Clamp to valid RC PWM range (1000–2000µs). Values of 0 = no signal yet.
  ch1 = (v1 > 0) ? constrain(v1, 1000, 2000) : 0;
  ch2 = (v2 > 0) ? constrain(v2, 1000, 2000) : 0;
  ch3 = (v3 > 0) ? constrain(v3, 1000, 2000) : 0;
  ch4 = (v4 > 0) ? constrain(v4, 1000, 2000) : 0;
  // CH5 is a 2-position switch: >1500 -> 2000us, otherwise 1000us
  ch5 = (v5 > 0) ? ((v5 > 1500) ? 2000 : 1000) : 0;
}
