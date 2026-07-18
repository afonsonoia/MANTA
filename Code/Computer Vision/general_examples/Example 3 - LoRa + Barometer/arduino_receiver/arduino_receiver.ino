#include <SPI.h>
#include <LoRa.h>

// LoRa pins with voltage dividers
const int csPin = 10;          // LoRa NSS
const int resetPin = 9;        // LoRa RST
const int irqPin = 2;          // LoRa DIO0

void setup() {
  Serial.begin(9600); 
  while (!Serial);

  Serial.println("Starting LoRa Receiver...");

  LoRa.setPins(csPin, resetPin, irqPin);

  // Start radio at 433MHz
  if (!LoRa.begin(433E6)) {
    Serial.println("Error: Failed to start LoRa. Check wiring and resistors!");
    while (1);
  }

  Serial.println("Ready! Waiting for data from ESP32...");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print("Received: '");

    while (LoRa.available()) {
      String data = LoRa.readString();
      Serial.print(data);
    }

    Serial.print("' | RSSI: ");
    Serial.println(LoRa.packetRssi());
  }
}