#include "network.h"
#include "battery.h"
#include "config.h"
#include "control.h"
#include "receiver.h"
#include "telemetry_codec.h"
#include <LoRa.h>
#include <SPI.h>

static uint8_t currentLoRaPower = LORA_TX_POWER;
static bool loraOnline = false;
static unsigned long lastLoRaReconnectAttempt = 0;
static unsigned long lastSuccessfulTxTime = 0;

void setLoRaTxPower(uint8_t powerDbm) {
  if (powerDbm < 2)
    powerDbm = 2;
  if (powerDbm > 20)
    powerDbm = 20;
  if (currentLoRaPower != powerDbm) {
    currentLoRaPower = powerDbm;
    if (loraOnline) {
      LoRa.setTxPower(currentLoRaPower, PA_OUTPUT_PA_BOOST_PIN);
    }
  }
}

uint8_t getLoRaTxPower() { return currentLoRaPower; }

static bool attemptLoRaStart() {
  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);

  // Hardware Reset pulse on LoRa module
  if (LORA_RST != -1) {
    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, LOW);
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    delay(10);
  }

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setSPI(SPI);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (LoRa.begin(LORA_BAND)) {
    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setTxPower(currentLoRaPower, PA_OUTPUT_PA_BOOST_PIN);
    LoRa.setCodingRate4(LORA_CR);
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.enableCrc();
    loraOnline = true;
    return true;
  }
  loraOnline = false;
  return false;
}

void initNetwork() {
  attemptLoRaStart();
}

void sendTelemetry(
    float pitch, float roll,
    int16_t accelX, int16_t accelY, int16_t accelZ,
    int16_t gyroX, int16_t gyroY, int16_t gyroZ,
    uint16_t rch1, uint16_t rch2, uint16_t rch3, uint16_t rch5,
    float batteryVoltage, float alt,
    bool rcSignalLost,
    bool isCalibMode
) {
  static uint32_t packetCount = 0;

  // If LoRa is not online, attempt non-blocking reconnect every 2 seconds
  if (!loraOnline) {
    unsigned long now = millis();
    if (now - lastLoRaReconnectAttempt >= 2000) {
      lastLoRaReconnectAttempt = now;
      attemptLoRaStart();
    }
    return; // Don't block flight controller while LoRa is offline
  }

  uint8_t flags = (rcSignalLost ? 1 : 0) | (isCalibMode ? 2 : 0);

  MantaTelemetryPacket pkt;
  encode_telemetry_packet(&pkt, pitch, roll,
                          accelX, accelY, accelZ,
                          gyroX, gyroY, gyroZ,
                          rch1, rch2, rch3, rch5,
                          batteryVoltage, alt,
                          flags);

  // beginPacket() returns 0 if radio is currently busy transmitting
  if (LoRa.beginPacket()) {
    LoRa.write((const uint8_t *)&pkt, sizeof(MantaTelemetryPacket));
    LoRa.endPacket(true); // Asynchronous / Non-blocking TX (never hangs the MCU)
    lastSuccessfulTxTime = millis();
    packetCount++;
  } else {
    // If radio remains busy for > 200ms (stalled transmission), auto-recover
    // NOTE: Do NOT reset lastSuccessfulTxTime here — only update it on real TX success (line above)
    if (lastSuccessfulTxTime > 0 && millis() - lastSuccessfulTxTime > 200) {
      attemptLoRaStart();
    }
  }
}
