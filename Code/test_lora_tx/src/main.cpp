#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

constexpr int LORA_MOSI = 23;
constexpr int LORA_MISO = 19;
constexpr int LORA_SCK = 18;
constexpr int LORA_CS = 5;
constexpr int LORA_RST = 14;
constexpr int LORA_DIO0 = 4;
constexpr long LORA_BAND = 433E6;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n[LORA HARDWARE MONITOR COM5] Checking LoRa chip...");
}

void loop() {
    pinMode(LORA_CS, OUTPUT);
    digitalWrite(LORA_CS, HIGH);

    if (LORA_RST != -1) {
        pinMode(LORA_RST, OUTPUT);
        digitalWrite(LORA_RST, LOW);
        delay(10);
        digitalWrite(LORA_RST, HIGH);
        delay(10);
    }

    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    
    digitalWrite(LORA_CS, LOW);
    SPI.transfer(0x42 & 0x7F);
    uint8_t ver = SPI.transfer(0x00);
    digitalWrite(LORA_CS, HIGH);

    Serial.print("[COM5 HARDWARE MONITOR] SPI Reg 0x42: 0x");
    if (ver < 16) Serial.print("0");
    Serial.print(ver, HEX);

    if (ver == 0x12) {
        Serial.println(" -> SUCCESS! SX1276 DETECTED!");
        LoRa.setSPI(SPI);
        LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
        if (LoRa.begin(LORA_BAND)) {
            Serial.println("[COM5] LoRa Radio Started! Transmitting test telemetry...");
            LoRa.setTxPower(17);
            LoRa.setSyncWord(0x12);
            LoRa.beginPacket();
            LoRa.print("BAT_ADC:2450.0");
            LoRa.endPacket();
        }
    } else {
        Serial.println(" -> Waiting for LoRa hardware wiring on COM5...");
    }

    delay(2000);
}
