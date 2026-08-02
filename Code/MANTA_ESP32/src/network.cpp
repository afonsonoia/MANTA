#include "network.h"
#include "config.h"
#include "esc.h"
#include <LoRa.h>
#include <SPI.h>

#define DIAG_LED 2

void initNetwork() {
  pinMode(DIAG_LED, OUTPUT);
  digitalWrite(DIAG_LED, LOW);

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

  int retries = 0;
  while (!LoRa.begin(LORA_BAND)) {
    Serial.println("[LORA] Error: Failed to initialize LoRa radio module! Check SPI wiring!");
    digitalWrite(DIAG_LED, HIGH);
    delay(150);
    digitalWrite(DIAG_LED, LOW);
    delay(350);
    retries++;
  }

  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.enableCrc();
  LoRa.receive();

  Serial.println("[LORA] Radio initialized successfully!");
}

void handleNetworkCommands() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String req = "";
    while (LoRa.available()) {
      req += (char)LoRa.read();
    }
    req.trim();
    if (req.length() > 0) {
      Serial.print("[LORA] Received command: ");
      Serial.println(req);

      if (req.startsWith("THROTTLE:")) {
        int val = req.substring(9).toInt();
        setThrottlePulse(val);
      }
    }
  }
}

void sendTelemetry(float rawADC, float batteryVoltage) {
  digitalWrite(DIAG_LED, HIGH);

  Serial.print("[LORA TX] Transmitting BAT_V:");
  Serial.print(batteryVoltage, 2);
  Serial.print("V | BAT_ADC:");
  Serial.println(rawADC, 1);

  LoRa.beginPacket();
  LoRa.print("BAT_V:");
  LoRa.print(batteryVoltage, 2);
  LoRa.print("|BAT_ADC:");
  LoRa.print(rawADC, 1);
  LoRa.endPacket();

  LoRa.receive(); // Re-enable receive mode after transmission
  digitalWrite(DIAG_LED, LOW);
}
