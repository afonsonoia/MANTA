#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// LoRa Pin Configuration (VSPI & Control - Identical to Drone)
constexpr int LORA_MOSI = 23;
constexpr int LORA_MISO = 19;
constexpr int LORA_SCK = 18;
constexpr int LORA_CS = 5;
constexpr int LORA_RST = 14;
constexpr int LORA_DIO0 = 4;

constexpr long LORA_BAND = 433E6; // Frequency: 433 MHz (433E6)
constexpr int LORA_TX_POWER = 17;  // 17 dBm
constexpr int LORA_SF = 8;        // Spreading Factor 8 (Optimal balance: +3dB sensitivity gain, high range & 4Hz 55ms airtime)
constexpr long LORA_BW = 125E3;   // Bandwidth 125 kHz
constexpr int LORA_CR = 5;        // Coding rate 4/5
constexpr uint8_t LORA_SYNC_WORD = 0x12; // Matching LoRa Sync Word


// Hardware Peripherals
constexpr int BUZZER_PIN = 22;
constexpr int BUZZER_FREQ = 2000;    // 2000 Hz
constexpr int BEEP_DURATION_MS = 50; // Quick 50ms beep per packet

#endif // CONFIG_H
