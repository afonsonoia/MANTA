#include <SPI.h>
#include <LoRa.h>

// Pinos definidos para a tua montagem com divisores de tensão
const int csPin = 10;          // LoRa NSS
const int resetPin = 9;        // LoRa RST
const int irqPin = 2;          // LoRa DIO0

void setup() {
  Serial.begin(9600); // Velocidade padrão do Arduino Uno
  while (!Serial);

  Serial.println("Iniciando Recetor LoRa...");

  // Configura os pinos do LoRa
  LoRa.setPins(csPin, resetPin, irqPin);

  // Inicia o rádio na frequência 433MHz
  if (!LoRa.begin(433E6)) {
    Serial.println("Erro: Falha ao iniciar LoRa. Verifica as ligacoes e as resistencias!");
    while (1);
  }

  Serial.println("Pronto! Aguardando dados do ESP32...");
}

void loop() {
  // Tenta ler um pacote
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print("Recebido: '");

    // Lê o conteúdo da mensagem
    while (LoRa.available()) {
      String data = LoRa.readString();
      Serial.print(data);
    }

    // Mostra a força do sinal (RSSI)
    Serial.print("' | RSSI: ");
    Serial.println(LoRa.packetRssi());
  }
}