#include "receiver.h"
#include "config.h"
#include <HardwareSerial.h>

// FlySky FS-iA6B i-Bus protocol operates over UART at 115200 baud, 8N1
// Frame structure: 32 bytes every ~7.7 ms
// Byte 0: 0x20 (frame length: 32)
// Byte 1: 0x40 (command: servo channels)
// Bytes 2-29: 14 channels (16-bit little-endian, PWM values ~1000 - 2000 us)
// Bytes 30-31: 16-bit checksum (little-endian): 0xFFFF - sum(bytes[0..29])

static HardwareSerial ibusSerial(1);

// Decoded channel values (channels 1..14)
static constexpr uint8_t IBUS_MAX_CHANNELS = 14;
static volatile uint16_t rawChannelVector[IBUS_MAX_CHANNELS] = {0};
static volatile uint16_t savedChannelVector[IBUS_MAX_CHANNELS] = {0};
static volatile uint8_t rcMarginDeadband = DEFAULT_RC_MARGIN_DEADBAND;

void setRCMarginDeadband(uint8_t deadbandUs) {
  if (deadbandUs < 1) deadbandUs = 1;
  if (deadbandUs > 50) deadbandUs = 50;
  rcMarginDeadband = deadbandUs;
}

uint8_t getRCMarginDeadband() {
  return rcMarginDeadband;
}

static portMUX_TYPE rcMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t lastRcPulseMicros = 0;

bool isRCSignalLost() {
  portENTER_CRITICAL(&rcMux);
  uint32_t lastPulse = lastRcPulseMicros;
  portEXIT_CRITICAL(&rcMux);

  if (lastPulse == 0) {
    return (millis() > 3000); // 3s grace period after boot
  }
  return (micros() - lastPulse > 600000); // Failsafe: > 600ms without valid i-Bus packet
}

bool isRCDataReceived() {
  portENTER_CRITICAL(&rcMux);
  uint32_t lastPulse = lastRcPulseMicros;
  portEXIT_CRITICAL(&rcMux);

  if (lastPulse == 0) {
    return false; // No packets received yet from transmitter
  }
  return (micros() - lastPulse <= 600000); // Active valid packet stream within 600ms
}

void initReceiver() {
  // Configured as INPUT for GPIO 39 (PCB CH1 input with built-in 5V->3.3V voltage divider)
  pinMode(PIN_IBUS_RX, INPUT);

  // Initialize HardwareSerial 1 on PIN_IBUS_RX (RX only, TX not used: -1)
  ibusSerial.begin(IBUS_BAUD, SERIAL_8N1, PIN_IBUS_RX, -1);
}

// Non-blocking i-Bus state machine parser (called exclusively from loop() on Core 1)
void updateReceiver() {
  static uint8_t rxBuffer[32];
  static uint8_t rxIndex = 0;
  static uint32_t lastByteMicros = 0;

  while (ibusSerial.available() > 0) {
    uint8_t b = (uint8_t)ibusSerial.read();
    uint32_t now = micros();

    // Inter-byte timeout: i-Bus packets are sent continuously (~7.7ms interval).
    // An idle gap > 3.0ms between bytes indicates a new frame boundary.
    if (rxIndex > 0 && (now - lastByteMicros > 3000)) {
      rxIndex = 0;
    }
    lastByteMicros = now;

    if (rxIndex == 0) {
      if (b == 0x20) { // Valid i-Bus header length byte
        rxBuffer[0] = b;
        rxIndex = 1;
      }
      continue;
    }

    if (rxIndex == 1) {
      if (b == 0x40) { // Valid i-Bus command byte
        rxBuffer[1] = b;
        rxIndex = 2;
      } else {
        rxIndex = 0; // Desync recovery
      }
      continue;
    }

    rxBuffer[rxIndex++] = b;

    // Full 32-byte frame received
    if (rxIndex == 32) {
      rxIndex = 0; // Reset index for next frame

      // Calculate 16-bit checksum
      uint16_t calcChk = 0xFFFF;
      for (uint8_t i = 0; i < 30; i++) {
        calcChk -= rxBuffer[i];
      }

      uint16_t frameChk = (uint16_t)rxBuffer[30] | ((uint16_t)rxBuffer[31] << 8);

      if (calcChk == frameChk) {
        portENTER_CRITICAL(&rcMux);
        lastRcPulseMicros = micros();

        // Extract 14 channels (little-endian uint16)
        uint8_t db = (rcMarginDeadband > 0) ? rcMarginDeadband : 4;
        for (uint8_t ch = 0; ch < IBUS_MAX_CHANNELS; ch++) {
          uint16_t val = (uint16_t)rxBuffer[2 + ch * 2] | ((uint16_t)rxBuffer[3 + ch * 2] << 8);
          // Valid servo pulse span in us
          if (val >= 850 && val <= 2150) {
            rawChannelVector[ch] = val;
            uint16_t cur = savedChannelVector[ch];
            if (cur == 0 || abs((int)val - (int)cur) >= (int)db) {
              savedChannelVector[ch] = val;
            }
          }
        }
        portEXIT_CRITICAL(&rcMux);
      }
    }
  }
}

void getReceiverChannels(uint16_t &ch1, uint16_t &ch2, uint16_t &ch3, uint16_t &ch5) {
  portENTER_CRITICAL(&rcMux);
  uint16_t v1 = savedChannelVector[0];
  uint16_t v2 = savedChannelVector[1];
  uint16_t v3 = savedChannelVector[2];
  uint16_t v5 = savedChannelVector[4];
  portEXIT_CRITICAL(&rcMux);

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
  portENTER_CRITICAL(&rcMux);
  uint16_t v1 = savedChannelVector[0];
  uint16_t v2 = savedChannelVector[1];
  uint16_t v3 = savedChannelVector[2];
  uint16_t v4 = savedChannelVector[3];
  uint16_t v5 = savedChannelVector[4];
  portEXIT_CRITICAL(&rcMux);

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

