#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// LoRa Pin Configuration (VSPI & Control - Identical to Drone)
constexpr int LORA_MOSI = 23;
constexpr int LORA_MISO = 19;
constexpr int LORA_SCK = 18;
constexpr int LORA_CS = 5;
constexpr int LORA_RST = -1;
constexpr int LORA_DIO0 = 4;

// LoRa Radio Parameters - Dedicated Simplex RX (Downlink Telemetry from MANTA)
constexpr long LORA_BAND = 433E6;        // Frequency: 433 MHz
constexpr int LORA_TX_POWER = 17;        // Nominal PA setting (Ground Station operates strictly in RX mode)
constexpr int LORA_SF = 7;               // Spreading Factor 7 (Matching MANTA)
constexpr long LORA_BW = 250E3;          // Bandwidth 250 kHz (Matching MANTA)
constexpr int LORA_CR = 5;               // Coding rate 4/5
constexpr uint8_t LORA_SYNC_WORD = 0x12; // Matching LoRa Sync Word

// Hardware Peripherals (Local Ground Station Buzzer Alarm on Pin D22)
constexpr int BUZZER_PIN = 22;
constexpr int BUZZER_FREQ = 2000;    // 2000 Hz
constexpr int BEEP_DURATION_MS = 50; // Quick 50ms beep per packet
constexpr unsigned long MODE_CHANGE_BEEP_MS = 700; // 0.7-second audible feedback beep on mode change

#endif // CONFIG_H
