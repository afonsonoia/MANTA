#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <LoRa.h>
#include "config.h"
#include "telemetry_codec.h"

// Ring buffer / packet storage for ISR to loop handoff
static volatile bool packetReady = false;
static volatile int packetLen = 0;
static uint8_t packetBuf[128];
static volatile int packetRssi = 0;
static volatile float packetSnr = 0.0f;

static unsigned long lastPacketTime = 0;
static unsigned long lastRxWatchdog = 0;
static unsigned long lastRadioReset = 0;
static bool intermittentBeep = false;
static unsigned long lastBeepToggle = 0;
static bool beepState = false;
static unsigned long beepUntil = 0;

// Flight mode tracking for audio feedback
static bool hasInitialMode = false;
static uint8_t lastFlightMode = 0;
static bool lastRollActive = false;

#define BUZZER_PWM_CHANNEL 0
#define BUZZER_PWM_RESOLUTION 8
constexpr int BUZZER_DUTY_CYCLE = 128; // 50% square wave for loud acoustic output

static inline void startBuzzerTone() {
#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWriteTone(BUZZER_PIN, BUZZER_FREQ);
    ledcWrite(BUZZER_PIN, BUZZER_DUTY_CYCLE);
#else
    ledcWriteTone(BUZZER_PWM_CHANNEL, BUZZER_FREQ);
    ledcWrite(BUZZER_PWM_CHANNEL, BUZZER_DUTY_CYCLE);
#endif
}

static inline void stopBuzzerTone() {
#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(BUZZER_PIN, 0);
#else
    ledcWrite(BUZZER_PWM_CHANNEL, 0);
#endif
}

// Trigger timed buzzer non-blocking (default MODE_CHANGE_BEEP_MS = 1000ms)
static void triggerBuzzer(unsigned long durationMs = MODE_CHANGE_BEEP_MS) {
    startBuzzerTone();
    beepUntil = millis() + durationMs;
    if (beepUntil == 0) beepUntil = 1;
}

// LoRa ISR callback on DIO0 rise
void IRAM_ATTR onLoRaReceive(int packetSize) {
    if (packetSize <= 0 || packetSize > (int)sizeof(packetBuf)) {
        return;
    }
    
    // Only copy if previous packet was already consumed by loop
    if (!packetReady) {
        int bytesRead = 0;
        while (LoRa.available() && bytesRead < packetSize) {
            packetBuf[bytesRead++] = (uint8_t)LoRa.read();
        }
        packetLen = bytesRead;
        packetRssi = LoRa.packetRssi();
        packetSnr = LoRa.packetSnr();
        packetReady = true;
    }
}

static bool initLoRaRadio() {
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

    if (!LoRa.begin(LORA_BAND)) {
        return false;
    }

    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setTxPower(LORA_TX_POWER, PA_OUTPUT_PA_BOOST_PIN);
    LoRa.setCodingRate4(LORA_CR);
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.enableCrc();
    LoRa.onReceive(onLoRaReceive);
    LoRa.receive();
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("\n==================================================");
    Serial.println("  MANTA GROUND STATION LORA RECEIVER (SIMPLEX RX) ");
    Serial.println("==================================================");

    // Disable unused Wi-Fi & Bluetooth
    WiFi.mode(WIFI_OFF);
    btStop();

    Serial.println("[LORA GS] Initializing LoRa Radio on 433 MHz...");
    while (!initLoRaRadio()) {
        Serial.println("[LORA GS] Error: LoRa initialization failed! Retrying in 1s...");
        delay(1000);
    }

#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcAttach(BUZZER_PIN, BUZZER_FREQ, BUZZER_PWM_RESOLUTION);
#else
    ledcSetup(BUZZER_PWM_CHANNEL, BUZZER_FREQ, BUZZER_PWM_RESOLUTION);
    ledcAttachPin(BUZZER_PIN, BUZZER_PWM_CHANNEL);
#endif
    stopBuzzerTone();

    // Initial power-up beep removed (silent until communication)
    // startBuzzerTone();
    // delay(100);
    // stopBuzzerTone();

    Serial.println("[LORA GS] LoRa Radio ready (433MHz, SF7, BW250k, CR4/5, SYNC 0x12)!");
    Serial.println("[LORA GS] Continuous Simplex RX active (Interrupt-driven) - listening...\n");
    lastPacketTime = millis();
}

void loop() {
    unsigned long now = millis();

    // 1. Handle incoming packet from ISR
    if (packetReady) {
        // Fast local copy
        uint8_t localBuf[128];
        int localLen = packetLen;
        int localRssi = packetRssi;
        float localSnr = packetSnr;
        memcpy(localBuf, packetBuf, localLen);
        packetReady = false;

        if (localLen >= 2 && localBuf[0] == 'M' && localBuf[1] == 'T') {
            lastPacketTime = now;
            // Send raw binary payload to PC Serial
            Serial.write(localBuf, localLen);
            Serial.printf(" RSSI:%d SNR:%.1f\n", localRssi, localSnr);

            // Flight mode change detection & audible feedback (1s short beep)
            if (localLen == (int)sizeof(MantaTelemetryPacket)) {
                const MantaTelemetryPacket *pkt = (const MantaTelemetryPacket *)localBuf;
                uint16_t expectedCrc = calculate_telemetry_crc16(localBuf, offsetof(MantaTelemetryPacket, crc16));
                if (pkt->crc16 == expectedCrc) {
                    bool rcLost = (pkt->flags & 0x01) != 0;
                    if (!rcLost) {
                        uint8_t mode = pkt->flight_mode;
                        bool rollActive = (pkt->flags & 0x02) != 0;
                        if (!hasInitialMode) {
                            lastFlightMode = mode;
                            lastRollActive = rollActive;
                            hasInitialMode = true;
                        } else if (mode != lastFlightMode || rollActive != lastRollActive) {
                            triggerBuzzer(MODE_CHANGE_BEEP_MS);
                            lastFlightMode = mode;
                            lastRollActive = rollActive;
                        }
                    }
                }
            } else if (localLen == 49) {
                uint16_t expectedCrc = calculate_telemetry_crc16(localBuf, 47);
                uint16_t pktCrc = (uint16_t)localBuf[47] | ((uint16_t)localBuf[48] << 8);
                if (pktCrc == expectedCrc) {
                    uint8_t flags = localBuf[46];
                    bool rcLost = (flags & 0x01) != 0;
                    if (!rcLost) {
                        uint16_t ch5 = (uint16_t)localBuf[29] | ((uint16_t)localBuf[30] << 8);
                        uint8_t mode = 1;
                        bool rollActive = (flags & 0x02) != 0;
                        if (ch5 >= 1345 && ch5 < 1610) mode = 2;
                        else if (ch5 >= 1610) mode = 3;

                        if (!hasInitialMode) {
                            lastFlightMode = mode;
                            lastRollActive = rollActive;
                            hasInitialMode = true;
                        } else if (mode != lastFlightMode || rollActive != lastRollActive) {
                            triggerBuzzer(MODE_CHANGE_BEEP_MS);
                            lastFlightMode = mode;
                            lastRollActive = rollActive;
                        }
                    }
                }
            }
        }
    }

    // 2. Hardware RF Watchdog: Auto-recover if no packet received for > 3000ms
    if (now - lastPacketTime > 3000 && now - lastRxWatchdog > 1000) {
        lastRxWatchdog = now;
        // Re-arm continuous RX mode safely
        LoRa.receive();
    }

    // If dead for > 10 seconds, perform full radio reinitialization (rate limited to once every 5000ms)
    if (now - lastPacketTime > 10000 && now - lastRadioReset > 5000) {
        lastRadioReset = now;
        initLoRaRadio();
    }

    // 3. Buzzer handler: Timed mode feedback beep (1s non-blocking) or intermittent alarm
    if (beepUntil > 0) {
        if ((long)(now - beepUntil) >= 0) {
            beepUntil = 0;
            if (!intermittentBeep) {
                stopBuzzerTone();
            }
        }
    } else if (intermittentBeep) {
        if (now - lastBeepToggle >= 250) {
            lastBeepToggle = now;
            beepState = !beepState;
            if (beepState) startBuzzerTone();
            else stopBuzzerTone();
        }
    }

    // 4. Check for local buzzer commands from PC Serial (Simplex RX: Ground Station local buzzer only)
    static char serialCmdBuf[64];
    static int serialCmdIdx = 0;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (serialCmdIdx > 0) {
                serialCmdBuf[serialCmdIdx] = '\0';

                if (strncmp(serialCmdBuf, "BEEP:SHORT", 10) == 0 || strncmp(serialCmdBuf, "SHORT", 5) == 0 ||
                    strncmp(serialCmdBuf, "BEEP:0.7S", 9) == 0 || strncmp(serialCmdBuf, "BEEP:1S", 7) == 0 || strncmp(serialCmdBuf, "BEEP:MODE", 9) == 0) {
                    triggerBuzzer(MODE_CHANGE_BEEP_MS);
                    Serial.println("[BUZZER] Beep 0.7s triggered");
                } else if (strncmp(serialCmdBuf, "BEEP:CONTINUOUS", 15) == 0 || strncmp(serialCmdBuf, "CONTINUOUS", 10) == 0) {
                    intermittentBeep = false;
                    beepUntil = 0;
                    startBuzzerTone();
                    Serial.println("[BUZZER] Continuous ON");
                } else if (strncmp(serialCmdBuf, "BEEP:INTERMITTENT", 17) == 0 || strncmp(serialCmdBuf, "INTERMITTENT", 12) == 0) {
                    beepUntil = 0;
                    intermittentBeep = true;
                    Serial.println("[BUZZER] Intermittent ON");
                } else if (strncmp(serialCmdBuf, "BEEP:OFF", 8) == 0 || strncmp(serialCmdBuf, "OFF", 3) == 0) {
                    intermittentBeep = false;
                    beepUntil = 0;
                    stopBuzzerTone();
                    Serial.println("[BUZZER] OFF");
                }
                serialCmdIdx = 0;
            }
        } else if (serialCmdIdx < (int)sizeof(serialCmdBuf) - 1) {
            serialCmdBuf[serialCmdIdx++] = c;
        }
    }

    // Small yield to let FreeRTOS and serial buffers breathe
    delay(1);
}

