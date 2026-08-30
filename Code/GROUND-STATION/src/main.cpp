#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <LoRa.h>
#include "config.h"

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("\n==================================================");
    Serial.println("  MANTA GROUND STATION LORA RECEIVER (SIMPLEX RX) ");
    Serial.println("==================================================");

    // Disable unused Wi-Fi & Bluetooth
    WiFi.mode(WIFI_OFF);
    btStop();

    // Hardware Reset pulse on LoRa module if connected
    if (LORA_RST != -1) {
        pinMode(LORA_RST, OUTPUT);
        digitalWrite(LORA_RST, LOW);
        delay(10);
        digitalWrite(LORA_RST, HIGH);
        delay(10);
    }

    pinMode(LORA_CS, OUTPUT);
    digitalWrite(LORA_CS, HIGH);

    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    LoRa.setSPI(SPI);
    LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

    Serial.println("[LORA GS] Initializing LoRa Radio on 433 MHz...");
    while (!LoRa.begin(LORA_BAND)) {
        Serial.println("[LORA GS] Error: LoRa initialization failed! Retrying in 1s...");
        delay(1000);
    }

    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setTxPower(LORA_TX_POWER, PA_OUTPUT_PA_BOOST_PIN);
    LoRa.setCodingRate4(LORA_CR);
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.enableCrc();
    LoRa.receive();

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    // Initial short beep on power-up
    digitalWrite(BUZZER_PIN, HIGH);
    delay(50);
    digitalWrite(BUZZER_PIN, LOW);

    Serial.println("[LORA GS] LoRa Radio ready (433MHz, SF8, BW250k, CR4/5, SYNC 0x12)!");
    Serial.println("[LORA GS] Continuous Simplex RX active - listening for MANTA packets...\n");
}

static unsigned long lastPacketTime = 0;
static unsigned long lastRxCheck = 0;
static bool intermittentBeep = false;
static unsigned long lastBeepToggle = 0;
static bool beepState = false;

void loop() {
    // Intermittent alarm buzzer handler
    if (intermittentBeep) {
        unsigned long now = millis();
        if (now - lastBeepToggle >= 250) {
            lastBeepToggle = now;
            beepState = !beepState;
            digitalWrite(BUZZER_PIN, beepState ? HIGH : LOW);
        }
    }

    // 1. Check for incoming LoRa packets from MANTA
    int packetSize = LoRa.parsePacket();
    if (packetSize > 0) {
        lastPacketTime = millis();
        uint8_t packetBuffer[128];
        int bytesRead = 0;
        while (LoRa.available() && bytesRead < 128) {
            packetBuffer[bytesRead++] = (uint8_t)LoRa.read();
        }

        int rssi = LoRa.packetRssi();
        float snr = LoRa.packetSnr();

        // Immediately put radio back into continuous RX mode before serial streaming
        LoRa.receive();

        // Send raw binary payload to PC Serial non-blocking
        Serial.write(packetBuffer, bytesRead);
        Serial.printf(" RSSI:%d SNR:%.1f\n", rssi, snr);
    } else {
        // Watchdog: If no valid packet received in 1000ms, ensure radio stays actively in RX continuous mode
        unsigned long now = millis();
        if (now - lastPacketTime > 1000 && now - lastRxCheck > 500) {
            lastRxCheck = now;
            LoRa.receive();
        }
    }

    // 2. Check for buzzer commands from PC Serial
    static char serialCmdBuf[64];
    static int serialCmdIdx = 0;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (serialCmdIdx > 0) {
                serialCmdBuf[serialCmdIdx] = '\0';

                if (strncmp(serialCmdBuf, "BEEP:SHORT", 10) == 0) {
                    digitalWrite(BUZZER_PIN, HIGH);
                    delay(50);
                    digitalWrite(BUZZER_PIN, LOW);
                } else if (strncmp(serialCmdBuf, "BEEP:INTERMITTENT", 17) == 0) {
                    intermittentBeep = true;
                } else if (strncmp(serialCmdBuf, "BEEP:OFF", 8) == 0) {
                    intermittentBeep = false;
                    digitalWrite(BUZZER_PIN, LOW);
                }
                serialCmdIdx = 0;
            }
        } else if (serialCmdIdx < (int)sizeof(serialCmdBuf) - 1) {
            serialCmdBuf[serialCmdIdx++] = c;
        }
    }
}
