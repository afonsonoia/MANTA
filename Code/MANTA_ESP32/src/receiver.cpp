#include "receiver.h"
#include "config.h"

// 1. Raw pulse durations directly from ISR: 0 = no signal received yet
static volatile uint16_t rawChannelVector[5] = {0, 0, 0, 0, 0};

// 2. Low-Pass Filter (80% previous / 20% new) - Auxiliary internal state only
static float lpfChannelVector[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// 3. Saved channel vector with deadband applied (sent outside as real receiver values)
static volatile uint16_t savedChannelVector[5] = {0, 0, 0, 0, 0};

// Dynamic RC Margin Deadband (default from config.h)
static volatile uint8_t rcMarginDeadband = DEFAULT_RC_MARGIN_DEADBAND;

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

  // 3. Store raw channel sample from receiver
  rawChannelVector[index] = (uint16_t)dt;

  // 4. Low-Pass Filter (80% previous / 20% new) - Auxiliary internal state only
  if (lpfChannelVector[index] <= 0.0f) {
    lpfChannelVector[index] = (float)dt;
  } else {
    lpfChannelVector[index] = (0.80f * lpfChannelVector[index]) + (0.20f * (float)dt);
  }

  // 5. Apply RC Margin Deadband on top of LPF output to update real receiver values
  uint16_t lpfRounded = (uint16_t)(lpfChannelVector[index] + 0.5f);
  uint16_t currentSaved = savedChannelVector[index];
  if (currentSaved == 0) {
    savedChannelVector[index] = lpfRounded;
  } else {
    int diff = (int)lpfRounded - (int)currentSaved;
    uint8_t db = rcMarginDeadband;
    if (diff > (int)db || diff < -((int)db)) {
      savedChannelVector[index] = lpfRounded;
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

void getRawReceiverChannels(uint16_t &ch1, uint16_t &ch2, uint16_t &ch3, uint16_t &ch4, uint16_t &ch5) {
  noInterrupts();
  uint16_t r1 = rawChannelVector[0];
  uint16_t r2 = rawChannelVector[1];
  uint16_t r3 = rawChannelVector[2];
  uint16_t r4 = rawChannelVector[3];
  uint16_t r5 = rawChannelVector[4];
  interrupts();

  if (isRCSignalLost()) {
    ch1 = 0; ch2 = 0; ch3 = 0; ch4 = 0; ch5 = 0;
    return;
  }

  ch1 = (r1 > 0) ? constrain(r1, 1000, 2000) : 0;
  ch2 = (r2 > 0) ? constrain(r2, 1000, 2000) : 0;
  ch3 = (r3 > 0) ? constrain(r3, 1000, 2000) : 0;
  ch4 = (r4 > 0) ? constrain(r4, 1000, 2000) : 0;
  ch5 = (r5 > 0) ? ((r5 > 1500) ? 2000 : 1000) : 0;
}
