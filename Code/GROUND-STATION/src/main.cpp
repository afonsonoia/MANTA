#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "config.h"

#define PWM_CHANNEL 0
#define PWM_RESOLUTION 8
const int DUTY_CYCLE = 128;

enum BuzzerMode {
    BUZZER_MODE_OFF,
    BUZZER_MODE_INTERMITTENT,
    BUZZER_MODE_CONTINUOUS
};

BuzzerMode currentBuzzerMode = BUZZER_MODE_OFF;
unsigned long intermittentTimer = 0;
bool intermittentState = false;

void stopTone() {
    #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(BUZZER_PIN, 0);
    #else
    ledcWrite(PWM_CHANNEL, 0);
    #endif
}

void startTone() {
    #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWriteTone(BUZZER_PIN, BUZZER_FREQ);
    ledcWrite(BUZZER_PIN, DUTY_CYCLE);
    #else
    ledcWriteTone(PWM_CHANNEL, BUZZER_FREQ);
    ledcWrite(PWM_CHANNEL, DUTY_CYCLE);
    #endif
}

void updateBuzzer() {
    unsigned long now = millis();
    if (currentBuzzerMode == BUZZER_MODE_INTERMITTENT) {
        if (now - intermittentTimer >= 200) {
            intermittentTimer = now;
            intermittentState = !intermittentState;
            if (intermittentState) startTone(); else stopTone();
        }
    } else if (currentBuzzerMode == BUZZER_MODE_CONTINUOUS) {
        startTone();
    } else {
        stopTone();
    }
}

void setup() {
    Serial.begin(115200);

    #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcAttach(BUZZER_PIN, BUZZER_FREQ, PWM_RESOLUTION);
    #else
    ledcSetup(PWM_CHANNEL, BUZZER_FREQ, PWM_RESOLUTION);
    ledcAttachPin(BUZZER_PIN, PWM_CHANNEL);
    #endif
    stopTone();

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

    Serial.println("[LORA GS] Initializing LoRa Radio on 433 MHz...");
    while (!LoRa.begin(LORA_BAND)) {
        Serial.println("[LORA GS] Error: LoRa initialization failed! Retrying in 1s...");
        delay(1000);
    }

    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setCodingRate4(LORA_CR);
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.enableCrc();
    LoRa.receive();

    Serial.println("[LORA GS] LoRa Radio initialized and listening on 433 MHz!");
}

void loop() {
    updateBuzzer();

    // 1. Check for incoming LoRa packets from MANTA
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        String receivedData = "";
        while (LoRa.available()) {
            receivedData += (char)LoRa.read();
        }
        receivedData.trim();

        int rssi = LoRa.packetRssi();
        float snr = LoRa.packetSnr();

        Serial.print("[LORA RX 433MHz] ");
        Serial.print(receivedData);
        Serial.print(" | RSSI: ");
        Serial.print(rssi);
        Serial.print(" dBm | SNR: ");
        Serial.print(snr);
        Serial.println(" dB");
    }

    // 2. Check for Serial input from PC (lora_logger.py / GUI)
    if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() > 0) {
            if (cmd == "BEEP:INTERMITTENT" || cmd == "INTERMITTENT") {
                if (currentBuzzerMode != BUZZER_MODE_INTERMITTENT) {
                    currentBuzzerMode = BUZZER_MODE_INTERMITTENT;
                    intermittentTimer = millis();
                    intermittentState = false;
                    Serial.println("[LORA GS] Alarm Intermittent ON (50% 2kHz)");
                }
            } else if (cmd == "BEEP:CONTINUOUS" || cmd == "CONTINUOUS") {
                if (currentBuzzerMode != BUZZER_MODE_CONTINUOUS) {
                    currentBuzzerMode = BUZZER_MODE_CONTINUOUS;
                    startTone();
                    Serial.println("[LORA GS] Alarm Continuous ON");
                }
            } else if (cmd == "BEEP:OFF" || cmd == "OFF") {
                if (currentBuzzerMode != BUZZER_MODE_OFF) {
                    currentBuzzerMode = BUZZER_MODE_OFF;
                    stopTone();
                    Serial.println("[LORA GS] Alarm OFF");
                }
            } else if (cmd.startsWith("THROTTLE:")) {
                Serial.print("[LORA TX Command] ");
                Serial.println(cmd);

                LoRa.beginPacket();
                LoRa.print(cmd);
                LoRa.endPacket();
                LoRa.receive();
            }
        }
    }
}
