#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

// Pinos LoRa no ESP32
#define SCK 18
#define MISO 19
#define MOSI 23
#define SS 5
#define RST 14
#define DIO0 2

Adafruit_BMP280 bmp; // Sensor I2C

void setup() {
  Serial.begin(115200);

  // 1. Iniciar Sensor BMP280 (Pinos 21 e 22)
  Wire.begin(21, 22);
  if (!bmp.begin(0x77)) {
    Serial.println("Erro: Sensor BMP280 nao encontrado!");
    while (1);
  }

  // 2. Iniciar LoRa
  LoRa.setPins(SS, RST, DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("Erro: Falha ao iniciar LoRa!");
    while (1);
  }

  Serial.println("Emissor ESP32 configurado!");
}

void loop() {
  // Ler dados do sensor
  float temp = bmp.readTemperature();
  float press = bmp.readPressure() / 100.0F;

  // Criar mensagem
  String msg = "T:" + String(temp) + "C P:" + String(press) + "hPa";

  Serial.println("Enviando: " + msg);

  // Enviar via LoRa
  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();

  delay(5000); // Espera 5 segundos
}