#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Wi-Fi Access Point Configuration
constexpr char WIFI_SSID[] = "ESP32_Battery_Monitor";
constexpr uint16_t SERVER_PORT = 5005;

// Hardware Pin Configuration
constexpr int PIN_BATTERY = 36; // ESP32 Pin VP / GPIO36
constexpr int PIN_ESC = 25;     // ESC Control Pin D25

// LoRa Pin Configuration (VSPI & Control)
constexpr int LORA_MOSI = 23;
constexpr int LORA_MISO = 19;
constexpr int LORA_SCK = 18;
constexpr int LORA_CS = 5;
constexpr int LORA_RST = 14;
constexpr int LORA_DIO0 = 4;

// LoRa Short-Distance Communication Parameters
constexpr long LORA_BAND = 433E6; // Frequency: 433 MHz (433E6)
constexpr int LORA_TX_POWER = 17; // 17 dBm
constexpr int LORA_SF = 7;      // Spreading Factor 7 (fastest & lowest latency)
constexpr long LORA_BW = 125E3; // Bandwidth 125 kHz
constexpr int LORA_CR = 5;      // Coding rate 4/5
constexpr uint8_t LORA_SYNC_WORD = 0x12; // Matching LoRa Sync Word

// Battery Monitoring & Safety Parameters
constexpr float BATTERY_DIVIDER_RATIO =
    4.84f; // HiLetgo 0–25V Voltage Sensor (4.84:1)
constexpr float ADC_REFERENCE = 3.3f;
constexpr float ADC_MAX = 4095.0f;
constexpr float MIN_CUTOFF_VOLTAGE =
    12.50f; // Safety threshold (V) matching battery monitor

constexpr unsigned long LOGGING_INTERVAL_MS =
    10000; // 10s telemetry broadcast interval for live graphing
constexpr unsigned long SAMPLE_INTERVAL_MS =
    100; // Uniform sampling every 100ms across the 10s window (100 samples/log)
constexpr int ADC_OVERSAMPLE_PER_TICK =
    8; // 8 burst readings per sample tick for high-frequency noise filtering

// ESC Throttle Limits
constexpr int THROTTLE_MIN_PULSE = 1000; // us (Armed / Off)
constexpr int THROTTLE_MAX_PULSE = 2000; // us (Full throttle)

#endif // CONFIG_H
