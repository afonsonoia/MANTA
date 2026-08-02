#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

struct ScanConfig {
    const char* name;
    int sck, miso, mosi, cs, rst, dio0;
};

ScanConfig configs[] = {
    // VSPI variants
    {"VSPI CS:5 RST:14 DIO:4", 18, 19, 23, 5, 14, 4},
    {"VSPI CS:5 RST:15 DIO:4", 18, 19, 23, 5, 15, 4},
    {"VSPI CS:5 RST:2 DIO:4", 18, 19, 23, 5, 2, 4},
    {"VSPI CS:5 RST:-1 DIO:4", 18, 19, 23, 5, -1, 4},
    {"VSPI CS:15 RST:14 DIO:4", 18, 19, 23, 15, 14, 4},
    {"VSPI CS:15 RST:-1 DIO:4", 18, 19, 23, 15, -1, 4},
    {"VSPI CS:5 RST:14 DIO:26", 18, 19, 23, 5, 14, 26},
    {"VSPI CS:5 RST:14 DIO:27", 18, 19, 23, 5, 14, 27},
    {"VSPI CS:5 RST:27 DIO:26", 18, 19, 23, 5, 27, 26},
    {"VSPI CS:5 RST:32 DIO:33", 18, 19, 23, 5, 32, 33},
    {"VSPI CS:4 RST:14 DIO:2", 18, 19, 23, 4, 14, 2},
    {"VSPI CS:2 RST:14 DIO:4", 18, 19, 23, 2, 14, 4},

    // HSPI variants
    {"HSPI CS:15 RST:14 DIO:4", 14, 12, 13, 15, 14, 4},
    {"HSPI CS:15 RST:2 DIO:4", 14, 12, 13, 15, 2, 4},
    {"HSPI CS:15 RST:-1 DIO:4", 14, 12, 13, 15, -1, 4},
    {"HSPI CS:15 RST:27 DIO:26", 14, 12, 13, 15, 27, 26},
    {"HSPI CS:5 RST:14 DIO:4", 14, 12, 13, 5, 14, 4},

    // TTGO / Heltec variants
    {"Heltec/TTGO SCK:5 MISO:19 MOSI:27 CS:18 RST:14 DIO:26", 5, 19, 27, 18, 14, 26},
    {"TTGO T-Beam SCK:5 MISO:19 MOSI:27 CS:18 RST:23 DIO:26", 5, 19, 27, 18, 23, 26}
};

const int numConfigs = sizeof(configs) / sizeof(configs[0]);

uint8_t checkVersion(int sck, int miso, int mosi, int cs, int rst) {
    LoRa.end();
    SPI.end();
    delay(50);

    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH);

    if (rst != -1) {
        pinMode(rst, OUTPUT);
        digitalWrite(rst, LOW);
        delay(10);
        digitalWrite(rst, HIGH);
        delay(10);
    }

    SPI.begin(sck, miso, mosi, cs);
    
    digitalWrite(cs, LOW);
    SPI.transfer(0x42 & 0x7F);
    uint8_t ver = SPI.transfer(0x00);
    digitalWrite(cs, HIGH);
    return ver;
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n[LORA PIN SCANNER FOR CURRENT BOARD ON COM4] Starting...");

    for (int i = 0; i < numConfigs; i++) {
        ScanConfig cfg = configs[i];
        uint8_t ver = checkVersion(cfg.sck, cfg.miso, cfg.mosi, cfg.cs, cfg.rst);
        
        Serial.print("[SCAN] ");
        Serial.print(cfg.name);
        Serial.print(" -> Reg 0x42 = 0x");
        if (ver < 16) Serial.print("0");
        Serial.print(ver, HEX);

        if (ver == 0x12) {
            Serial.println(" *** FOUND SX1276! ***");
            LoRa.setSPI(SPI);
            LoRa.setPins(cfg.cs, cfg.rst, cfg.dio0);
            if (LoRa.begin(433E6)) {
                Serial.println("SUCCESS! LoRa initialized successfully!");
                LoRa.setTxPower(17);
                LoRa.setSyncWord(0x12);
                return;
            }
        } else {
            Serial.println();
        }
        delay(200);
    }
    Serial.println("\n[SCAN COMPLETE] No LoRa module detected on any pin combination for this board.");
}

void loop() {
    static unsigned long lastTx = 0;
    if (millis() - lastTx > 2000) {
        lastTx = millis();
        Serial.println("[MANTA TEST TX] Sending heartbeat packet...");
        LoRa.beginPacket();
        LoRa.print("BAT_ADC:2500");
        LoRa.endPacket();
    }
}
