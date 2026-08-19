#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Hardware Pin Configuration
constexpr int PIN_BATTERY = 36; // ESP32 Pin VP / GPIO36
constexpr int PIN_ESC = 25;     // ESC Control Pin D25

// RC Receiver Input Pins
constexpr int PIN_RC_CH1 = 39; // VIN / VN (GPIO39)
constexpr int PIN_RC_CH2 = 34; // D34 (GPIO34)
constexpr int PIN_RC_CH3 = 35; // D35 (GPIO35)
constexpr int PIN_RC_CH4 = 32; // D32 (GPIO32)
constexpr int PIN_RC_CH5 = 33; // D33 (GPIO33)

// Servo Output Pins (V-Tail Airframe)
constexpr int PIN_SERVO_BR = 13; // Back Right (GPIO13)
constexpr int PIN_SERVO_BL = 14; // Back Left (GPIO14)
constexpr int PIN_SERVO_FR = 27; // Front Right (GPIO27)
constexpr int PIN_SERVO_FL = 26; // Front Left (GPIO26)

// Control System Hysteresis & Thermal Protection Constants
constexpr uint8_t DEFAULT_RC_MARGIN_DEADBAND =
    18; // Hysteresis threshold in us (ignore PWM jitter < 18us)
constexpr uint16_t DEFAULT_SERVO_MIN_UPDATE_INTERVAL_MS =
    67; // 67ms interval between servo updates (15 Hz max)
constexpr unsigned long SERVO_KEEPALIVE_INTERVAL_MS =
    1000; // 1.0s (1000ms) keep-alive update heartbeat

// Servo Rotation Angle Limits & Conversion Factors (Default +/-30 degrees)
constexpr float US_PER_DEGREE =
    11.11f; // ~11.11us per degree (1000us total span / 90 deg)
constexpr uint8_t DEFAULT_SERVO_MAX_ANGLE_DEG =
    30; // Default +/- 30 degrees rotation limit (1167us - 1833us)

// MPU6050 I2C Pin Configuration
constexpr int PIN_SDA = 21; // MPU6050 SDA Pin D21
constexpr int PIN_SCL = 22; // MPU6050 SCL Pin D22

// LoRa Pin Configuration (VSPI & Control)
constexpr int LORA_MOSI = 23;
constexpr int LORA_MISO = 19;
constexpr int LORA_SCK = 18;
constexpr int LORA_CS = 5;
constexpr int LORA_RST =
    -1; // RST not connected: avoids conflict with PIN_SERVO_BL on GPIO 14!
constexpr int LORA_DIO0 = 4;

// LoRa High-Speed Low-Power Parameters (~1.5km range @ ~6.2ms airtime)
constexpr long LORA_BAND = 433E6; // Frequency: 433 MHz
constexpr int LORA_TX_POWER =
    17; // 17 dBm (50mW power-optimized output for 1.5km range)
constexpr int LORA_SF = 8; // Spreading Factor 8 (+2.5dB demodulation margin down to -10dB SNR)
constexpr long LORA_BW = 250E3; // Bandwidth 250 kHz (High frequency offset
                                // tolerance & fast ~26ms airtime)
constexpr int LORA_CR = 5;      // Coding rate 4/5
constexpr uint8_t LORA_SYNC_WORD = 0x12; // Matching LoRa Sync Word

// Battery Monitoring & Safety Parameters
constexpr float BATTERY_DIVIDER_RATIO =
    4.84f; // HiLetgo 0–25V Voltage Sensor (4.84:1)
constexpr float ADC_REFERENCE = 3.3f;
constexpr float ADC_MAX = 4095.0f;
constexpr float ABSOLUTE_MIN_CUTOFF_VOLTAGE =
    12.00f; // Absolute hard safety cutoff floor (12.0V)
constexpr float DEFAULT_CUTOFF_VOLTAGE =
    12.50f; // Default threshold until updated by Ground Station

// High Frequency Sampling & Moving Average Filter Parameters
constexpr unsigned long LOGGING_INTERVAL_MS =
    50; // 50ms = 20 Hz Telemetry Broadcast (High speed flight data for PID
        // tuning)

constexpr unsigned long SAMPLE_INTERVAL_MS =
    10; // High frequency IMU sampling every 10ms (100 Hz sampling)
constexpr int MA_WINDOW_SIZE =
    15; // Moving average filter window size (15 samples @ 250Hz)
constexpr int ADC_OVERSAMPLE_PER_TICK =
    8; // 8 burst readings per sample tick for ADC noise filtering

// ESC Throttle Limits & Linear Affine Scaling Parameters
constexpr int THROTTLE_MIN_PULSE = 1000; // us (Armed / Off)
constexpr int THROTTLE_MAX_PULSE = 2000; // us (Full throttle hardware limit)

// Throttle Transmitter Input Range & Scaled Output Range (Capped at 1800us)
constexpr int THROTTLE_INPUT_MIN_US = 1000; // Expected transmitter stick bottom
constexpr int THROTTLE_INPUT_MAX_US = 2000; // Expected transmitter stick top
constexpr int THROTTLE_OUTPUT_MIN_US =
    1000; // Capped ESC output bottom (1000us)
constexpr int THROTTLE_OUTPUT_MAX_US =
    1800; // Capped ESC output top (1800us max cap)

#endif // CONFIG_H
