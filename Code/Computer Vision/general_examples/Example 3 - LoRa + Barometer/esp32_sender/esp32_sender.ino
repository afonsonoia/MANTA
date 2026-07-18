#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

// ESP32 LoRa pins
#define SCK 18
#define MISO 19
#define MOSI 23
#define SS 5
#define RST 14
#define DIO0 2

Adafruit_BMP280 bmp; // I2C Sensor

void setup() {
  Serial.begin(115200);

  // 1. Start BMP280 Sensor (Pins 21 and 22)
  Wire.begin(21, 22);
  if (!bmp.begin(0x77)) {
    Serial.println("Error: BMP280 sensor not found!");
    while (1);
  }

  // 2. Start LoRa
  LoRa.setPins(SS, RST, DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("Error: Failed to start LoRa!");
    while (1);
  }

  Serial.println("ESP32 Sender configured!");
}

void loop() {
  float temp = bmp.readTemperature();
  float press = bmp.readPressure() / 100.0F;

  String msg = "T:" + String(temp) + "C P:" + String(press) + "hPa";

  Serial.println("Sending: " + msg);

  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();

  delay(5000); 
}