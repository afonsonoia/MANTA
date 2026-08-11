#include <Arduino.h>
#include <WiFi.h>
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
    // 1. Startup power stabilization delay (150ms) to eliminate USB inrush current spikes
    delay(150);

    // Disable unused Wi-Fi & Bluetooth radios to minimize power consumption
    WiFi.mode(WIFI_OFF);
    btStop();

    Serial.begin(115200);

    // Explicitly configure Buzzer pin LOW before PWM attach to avoid transient boot current spikes
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

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

    // Set initial LoRa transmit power (14 dBm) - reduced to prevent high USB power spikes
    LoRa.setTxPower(14);
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
    if (packetSize > 0) {
        uint8_t packetBuffer[128];
        int bytesRead = 0;
        while (LoRa.available() && bytesRead < 128) {
            packetBuffer[bytesRead++] = (uint8_t)LoRa.read();
        }

        int rssi = LoRa.packetRssi();
        float snr = LoRa.packetSnr();

        // Adaptive Tx Power adjustment for Ground Station based on RSSI
        static int currentGsPower = 14;
        int targetGsPower = currentGsPower;
        if (rssi < -95) {
            targetGsPower = 20; // Weak signal: boost to max power (20 dBm)
        } else if (rssi > -80) {
            targetGsPower = 14; // Strong signal: conserve power (14 dBm)
        }
        if (targetGsPower != currentGsPower) {
            currentGsPower = targetGsPower;
            LoRa.setTxPower(currentGsPower);
        }

        // Send raw binary payload to PC Serial without null-byte string truncation
        Serial.write(packetBuffer, bytesRead);
        Serial.printf(" RSSI:%d SNR:%.1f\n", rssi, snr);
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
            } else if (cmd == "BEEP:SHORT" || cmd == "SHORT") {
                startTone();
                delay(60);
                stopTone();
                Serial.println("[LORA GS] Beep Short 60ms");
            } else if (cmd == "BEEP:OFF" || cmd == "OFF") {
                if (currentBuzzerMode != BUZZER_MODE_OFF) {
                    currentBuzzerMode = BUZZER_MODE_OFF;
                    stopTone();
                    Serial.println("[LORA GS] Alarm OFF");
                }
            } else {
                // Forward all outbound telemetry & configuration commands (THROTTLE, CUTOFF, SET_RC_FILTER, CALIB, etc.) to drone
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
