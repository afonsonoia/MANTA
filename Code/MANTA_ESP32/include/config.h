#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Hardware Pin Configuration
constexpr int PIN_BATTERY = 36; // ESP32 Pin VP / GPIO36
constexpr int PIN_ESC = 25;     // ESC Control Pin D25

// RC Receiver Configuration (FlySky FS-iA6B via i-Bus Protocol)
// PCB Port CH1 (GPIO 39 / Pin VN) with integrated resistive voltage divider (5V -> 3.3V)
constexpr int PIN_IBUS_RX = 39; // Hardware UART1 RX connected to FS-iA6B i-BUS SERVO port on PCB CH1
constexpr long IBUS_BAUD = 115200; // Standard FlySky i-Bus baud rate (8N1)


// Servo Output Pins (V-Tail Airframe)
constexpr int PIN_SERVO_BR = 13; // Back Right (GPIO13)
constexpr int PIN_SERVO_BL = 14; // Back Left (GPIO14)
constexpr int PIN_SERVO_FR = 27; // Front Right (GPIO27)
constexpr int PIN_SERVO_FL = 26; // Front Left (GPIO26)

// Control System Hysteresis Constants
constexpr uint8_t DEFAULT_RC_MARGIN_DEADBAND =
    4;  // Median filter already eliminates ISR jitter — 4us deadband prevents servo buzz without masking real stick input

// Servo Rotation Angle Limits & Conversion Factors (Default +/-25 degrees)
constexpr float US_PER_DEGREE =
    11.11f; // ~11.11us per degree (1000us total span / 90 deg)
constexpr uint8_t DEFAULT_SERVO_MAX_ANGLE_DEG =
    25; // Default +/- 25 degrees rotation limit (~278us from neutral: 1222us - 1778us)

// Flaperon Expanded Servo Deflection Limit (FR & FL surfaces when flaperons are engaged)
// Expanded from 25.0 deg (278 us) to 32.0 deg (~356 us) to allow full 20.0 deg flaps + roll authority
// without premature headroom choking, while retaining >= 100 us safe margin from the 2000 us ceiling.
constexpr uint8_t DEFAULT_FLAPERON_MAX_ANGLE_DEG = 32;
constexpr int FLAPERON_MAX_PULSE_LIMIT_US =
    (int)(DEFAULT_FLAPERON_MAX_ANGLE_DEG * US_PER_DEGREE + 0.5f); // ~356 us

// Flaperons (Landing Flaps) Parameters: Deflect both roll surfaces 20 degrees DOWN
constexpr float FLAPERON_DEFLECTION_DEG = 20.0f; // 20.0 deg DOWN max deflection for approach and landing
constexpr int FLAPERON_OFFSET_US =
    (int)(FLAPERON_DEFLECTION_DEG * US_PER_DEGREE + 0.5f); // ~222 us
// Servos FR & FL are physically mirrored:
// Deflection is calculated directly from each servo's trimmed neutral (spline/horn fit):
// For FL: negative PWM deflects DOWN (-222 us from trimmed neutral)
// For FR: positive PWM deflects DOWN (+222 us from trimmed neutral)
constexpr int FLAPERON_US_FR = +FLAPERON_OFFSET_US; // +222 us -> deflects right rolleron DOWN
constexpr int FLAPERON_US_FL = -FLAPERON_OFFSET_US; // -222 us -> deflects left rolleron DOWN

// Flaperon Throttle Governor Parameters:
// Flaperons deploy smoothly between 1500us and 1200us throttle (linear uniform transition).
// Above 1500us they are fully retracted (0 offset). Below 1200us they reach full 20.0 deg deflection.
constexpr int FLAPERON_THROTTLE_MAX_US = 1500; // >= 1500 us: Flaperons fully retracted (0 offset)
constexpr int FLAPERON_THROTTLE_MIN_US = 1200; // <= 1200 us: Flaperons fully deployed (20.0 deg DOWN)

// RC Stick Exponential Response Factor (0.0 = Linear, 1.0 = Pure Cubic; 0.35 = 35% Expo for center stick precision)
constexpr float RC_EXPO_FACTOR = 0.35f;

// MPU6050 I2C Pin Configuration
constexpr int PIN_SDA = 21; // MPU6050 SDA Pin D21
constexpr int PIN_SCL = 22; // MPU6050 SCL Pin D22

// GPS UART Pin Configuration (Hardware UART2)
constexpr int GPS_RX_PIN = 16;  // GPIO16 (RX2) connected to GPS TX
constexpr int GPS_TX_PIN = 17;  // GPIO17 (TX2) connected to GPS RX
constexpr long GPS_BAUD = 9600; // Standard NMEA GPS Baud Rate

// LoRa Pin Configuration (VSPI & Control)
constexpr int LORA_MOSI = 23;
constexpr int LORA_MISO = 19;
constexpr int LORA_SCK = 18;
constexpr int LORA_CS = 5;
constexpr int LORA_RST =
    -1; // RST not connected: avoids conflict with PIN_SERVO_BL on GPIO 14!
constexpr int LORA_DIO0 = 4;

// LoRa Long-Range High-Penetration Parameters (~3-5km range, SF8, CR 4/6, 20dBm)
constexpr long LORA_BAND = 433E6; // Frequency: 433 MHz
constexpr int LORA_TX_POWER =
    20; // 20 dBm (100mW max PA_BOOST output power for maximum range)
constexpr int LORA_SF = 8; // Spreading Factor 8 (+3dB sensitivity gain, high obstacle penetration)
constexpr long LORA_BW = 250E3; // Bandwidth 250 kHz (High frequency offset tolerance)
constexpr int LORA_CR = 6;      // Coding rate 4/6 (Hamming FEC for obstacle & interference immunity)
constexpr uint8_t LORA_SYNC_WORD = 0x12; // Matching LoRa Sync Word

// Battery Monitoring & Safety Parameters (Supports 3S & 4S LiPo with Auto-Detection)
constexpr float ABSOLUTE_MIN_CUTOFF_VOLTAGE = 9.00f; // Absolute hard safety floor for 3S/4S
constexpr float CUTOFF_VOLTAGE_4S = 12.00f;          // 4S cutoff: 3.0V per cell under load
constexpr float CUTOFF_VOLTAGE_3S = 9.60f;           // 3S cutoff: 3.2V per cell under load
constexpr float DEFAULT_CUTOFF_VOLTAGE = 12.00f;     // Default 4S nominal threshold
constexpr unsigned long BATTERY_SAMPLE_INTERVAL_MS = 5000; // 5000ms = 0.2 Hz
constexpr uint8_t LOW_VOLT_CONFIRM_COUNT = 3;        // Require 3 consecutive low readings to reject transient sag

// Sampling & 5.0 Hz Telemetry Broadcast Parameters
constexpr unsigned long LOGGING_INTERVAL_MS =
    200; // 200ms = 5.0 Hz Telemetry Broadcast (Ensures ~63ms idle margin above ~137ms LoRa 74B Time-on-Air)

constexpr unsigned long SAMPLE_INTERVAL_MS =
    10; // High frequency IMU sampling every 10ms (100 Hz sampling)
constexpr unsigned long BARO_SAMPLE_INTERVAL_MS =
    100; // BMP280 Barometer sampling every 100ms (10 Hz sampling)
constexpr int ADC_OVERSAMPLE_PER_TICK =
    8; // 8 burst readings per sample tick for ADC noise filtering

// ESC Throttle Limits & Soft Power Floor
constexpr int THROTTLE_MIN_PULSE = 1000; // us (Armed / Off)
constexpr int THROTTLE_MAX_PULSE = 1900; // us (Capped full throttle hardware limit: 1900us)
constexpr int THROTTLE_LOW_VOLT_CEILING_PULSE = 1350; // us (Soft power ceiling: ~35-40% throttle to glide/land safely)

// Throttle Transmitter Input Range & Scaled Output Range (Capped at 1900us)
constexpr int THROTTLE_INPUT_MIN_US = 1000; // Expected transmitter stick bottom
constexpr int THROTTLE_INPUT_MAX_US = 2000; // Expected transmitter stick top
constexpr int THROTTLE_OUTPUT_MIN_US =
    1000; // Capped ESC output bottom (1000us)
constexpr int THROTTLE_OUTPUT_MAX_US =
    1900; // Capped ESC output top (1900us max cap)

// ── FLY-BY-WIRE & CLOSED-LOOP PID PARAMETERS ────────────────────────────────
// Pitch PI-D Gains (V-Tail: Servos BR & BL)
constexpr float PID_PITCH_KP = 5.00f;
constexpr float PID_PITCH_KI = 2.50f;
constexpr float PID_PITCH_KD = 0.450f;

// Roll PI-D Gains (Rollerons: Servos FR & FL)
constexpr float PID_ROLL_KP = 15.00f;
constexpr float PID_ROLL_KI = 5.00f;
constexpr float PID_ROLL_KD = 1.500f;

// Fly-By-Wire Attitude Angle Limits: Pitch capped at 25 deg for stall prevention, Roll at 45 deg
constexpr float FBW_MAX_PITCH_DEG = 25.0f;
constexpr float FBW_MAX_ROLL_DEG = 45.0f;

// FBW Stick Exponential Factor (0.08 = ~92% linear, soft center without deadened feeling)
constexpr float FBW_EXPO_FACTOR = 0.08f;

// Anti-windup maximum integral authority in PWM microseconds:
// Pitch integral ceiling: 100.0us (+/-9.0 deg trim authority, ~36% of anglePulseLimit = 278us)
// Roll integral ceiling: 150.0us (+/-13.5 deg trim authority, ~54% of anglePulseLimit = 278us, 50% increase)
// to reliably eliminate steady-state attitude droop while reserving dynamic P+D response.
constexpr float MAX_INTEGRAL_PULSE_PITCH_US = 100.0f;
constexpr float MAX_INTEGRAL_PULSE_ROLL_US = 150.0f;
constexpr float MAX_INTEGRAL_PULSE_US = MAX_INTEGRAL_PULSE_PITCH_US;

// Coordinated Turn Pitch Compensation Gain (compensates vertical lift drop in turns)
constexpr float TURN_PITCH_COMP_GAIN = 6.0f; // degrees factor: ~1.8 deg added at 45 deg bank

// ── IMU MOUNTING ATTITUDE TRIM OFFSETS (Degrees) ───────────────────────────
// Pitch mounting offset: Calibrated level mounting trim
constexpr float IMU_PITCH_MOUNTING_OFFSET_DEG = 9.2f;
// Roll mounting offset: Calibrated to provide true zero-roll straight flight reference
// (+2.5 deg correction over original 1.9 deg to eliminate left-roll bank / counter-clockwise turning bias)
constexpr float IMU_ROLL_MOUNTING_OFFSET_DEG = 4.4f;

// ── EXTREMUM SEEKING CONTROL (ESC) PARAMETERS (MODE 3) ──────────────────────
// Pitch Extremum Seeking: Dither frequency ~1.0 Hz, amplitude ~0.08 (8% variation)
constexpr float ESC_PITCH_OMEGA = 6.283185f;  // 2 * PI * 1.0 Hz (rad/s)
constexpr float ESC_PITCH_DITHER_AMP = 0.08f; // 8% dither amplitude
constexpr float ESC_PITCH_GAMMA = 0.05f;       // Adaptation learning rate
constexpr float ESC_PITCH_MIN_SCALE = 0.60f;  // Minimum gain scale factor (60% nominal)
constexpr float ESC_PITCH_MAX_SCALE = 1.60f;  // Maximum gain scale factor (160% nominal)

// Roll Extremum Seeking: Dither frequency ~1.5 Hz (orthogonal to pitch), amplitude ~0.08
constexpr float ESC_ROLL_OMEGA = 9.424778f;   // 2 * PI * 1.5 Hz (rad/s)
constexpr float ESC_ROLL_DITHER_AMP = 0.08f;  // 8% dither amplitude
constexpr float ESC_ROLL_GAMMA = 0.05f;        // Adaptation learning rate
constexpr float ESC_ROLL_MIN_SCALE = 0.60f;   // Minimum gain scale factor (60% nominal)
constexpr float ESC_ROLL_MAX_SCALE = 1.60f;   // Maximum gain scale factor (160% nominal)

// Extremum Seeking High-Pass Washout Filter Cutoff Frequency (rad/s)
constexpr float ESC_HPF_CUTOFF = 1.25f;        // ~0.2 Hz cutoff to isolate dither perturbation

// Weight on angular rates in cost function J = error^2 + W_rate * rate^2
constexpr float ESC_PITCH_RATE_WEIGHT = 0.02f;
constexpr float ESC_ROLL_RATE_WEIGHT = 0.02f;

// ── CH5 FLIGHT MODE TRANSMITTER CALIBRATION & EMI HYSTERESIS THRESHOLDS ──────
// Calibrated transmitter PWM values:
// SWC 1 + SWB OFF = 1166us (Mode 1 + Flaperons OFF)
// SWC 2 + SWB OFF = 1328us (Mode 2 + Flaperons OFF)
// SWC 3 + SWB OFF = 1411us (Mode 3 + Flaperons OFF)
// SWC 1 + SWB ON  = 1541us (Mode 1 + Flaperons ON: 20 deg DOWN)
// SWC 2 + SWB ON  = 1825us (Mode 2 + Flaperons ON: 20 deg DOWN)
// SWC 3 + SWB ON  = 1942us (Mode 2 Auto Flap-Safe + Flaperons ON: 20 deg DOWN)

// Decision thresholds with Schmitt-trigger hysteresis bands for EMI rejection:
// Transition 1: Mode 1 OFF <-> Mode 2 OFF (Midpoint ~1247us)
constexpr uint16_t CH5_THRES_M1_OFF_TO_M2_OFF = 1250; // Rising edge trigger (us)
constexpr uint16_t CH5_THRES_M2_OFF_TO_M1_OFF = 1244; // Falling edge trigger (us)

// Transition 2: Mode 2 OFF <-> Mode 3 OFF (Midpoint ~1370us)
constexpr uint16_t CH5_THRES_M2_OFF_TO_M3_OFF = 1373; // Rising edge trigger (us)
constexpr uint16_t CH5_THRES_M3_OFF_TO_M2_OFF = 1367; // Falling edge trigger (us)

// Transition 3: Mode 3 OFF <-> Mode 1 ON (Midpoint ~1476us)
constexpr uint16_t CH5_THRES_M3_OFF_TO_M1_ON = 1479;  // Rising edge trigger (us)
constexpr uint16_t CH5_THRES_M1_ON_TO_M3_OFF = 1473;  // Falling edge trigger (us)

// Transition 4: Mode 1 ON <-> Mode 2 ON (Midpoint ~1683us)
constexpr uint16_t CH5_THRES_M1_ON_TO_M2_ON = 1686;   // Rising edge trigger (us)
constexpr uint16_t CH5_THRES_M2_ON_TO_M1_ON = 1680;   // Falling edge trigger (us)

// Transition 5: Mode 2 ON <-> Mode 3 ON (Midpoint ~1884us)
constexpr uint16_t CH5_THRES_M2_ON_TO_M3_ON = 1887;   // Rising edge trigger (us)
constexpr uint16_t CH5_THRES_M3_ON_TO_M2_ON = 1881;   // Falling edge trigger (us)

// Backward compatibility alias definitions
constexpr uint16_t CH5_THRES_M1_OFF_TO_ON = CH5_THRES_M1_OFF_TO_M2_OFF;
constexpr uint16_t CH5_THRES_M1_ON_TO_OFF = CH5_THRES_M2_OFF_TO_M1_OFF;
constexpr uint16_t CH5_THRES_M1_TO_M2 = CH5_THRES_M2_OFF_TO_M3_OFF;
constexpr uint16_t CH5_THRES_M2_TO_M1 = CH5_THRES_M3_OFF_TO_M2_OFF;
constexpr uint16_t CH5_THRES_M2_OFF_TO_ON = CH5_THRES_M3_OFF_TO_M1_ON;
constexpr uint16_t CH5_THRES_M2_ON_TO_OFF = CH5_THRES_M1_ON_TO_M3_OFF;
constexpr uint16_t CH5_THRES_M2_TO_M3 = CH5_THRES_M1_ON_TO_M2_ON;
constexpr uint16_t CH5_THRES_M3_TO_M2 = CH5_THRES_M2_ON_TO_M1_ON;
constexpr uint16_t CH5_THRES_M3_OFF_TO_ON = CH5_THRES_M2_ON_TO_M3_ON;
constexpr uint16_t CH5_THRES_M3_ON_TO_OFF = CH5_THRES_M3_ON_TO_M2_ON;

// Temporal debounce confirmation ticks (2 consecutive 20ms cycles = 40ms rejection window)
constexpr uint8_t CH5_DEBOUNCE_CONFIRM_TICKS = 2;

// ── AUTONOMOUS EMERGENCY FAILSAFE PARAMETERS (SIGNAL LOSS RECOVERY) ────────
constexpr int FAILSAFE_ARM_THROTTLE_THRESHOLD_US = 1600; // Only activate in-flight failsafe if throttle > 1600us in past
constexpr int FAILSAFE_DISARM_THROTTLE_THRESHOLD_US = 1100; // Throttle < 1100us considered idle/landed
constexpr unsigned long FAILSAFE_LANDING_DISARM_TIMEOUT_MS = 20000; // 20s continuous idle throttle on ground disarms failsafe
constexpr unsigned long FAILSAFE_STAGE2_TIMEOUT_MS = 30000; // 30s maximum in Stage 2 before transitioning to Stage 3
constexpr unsigned long FAILSAFE_ETAPA2_TIMEOUT_MS = FAILSAFE_STAGE2_TIMEOUT_MS; // Backward compatibility alias

constexpr float FAILSAFE_CLIMB_ALT_M = 25.0f;            // Target altitude for climb (m)
constexpr float FAILSAFE_CLIMB_PITCH_DEG = 20.0f;        // Climb pitch attitude (degrees UP)
constexpr float FAILSAFE_CLIMB_ROLL_DEG = 0.0f;          // Wings level during climb (degrees)
constexpr int FAILSAFE_CLIMB_THROTTLE_US = 1800;         // Climb throttle (us)

constexpr float FAILSAFE_LOITER_ALT_TARGET_M = 25.0f;    // Target loiter altitude (m)
constexpr float FAILSAFE_LOITER_HIGH_ALT_M = 40.0f;      // High altitude boundary for aggressive descent (m)
constexpr float FAILSAFE_LOITER_PITCH_DEG = 5.0f;        // Loiter pitch attitude (degrees UP)
constexpr float FAILSAFE_LOITER_ROLL_DEG = -30.0f;       // Loiter roll attitude (30 degrees LEFT bank)
constexpr int FAILSAFE_LOITER_THROTTLE_BASE_US = 1500;   // Nominal base throttle for Stage 2 (us)
constexpr int FAILSAFE_LOITER_MIN_THROTTLE_US = 1250;    // Minimum throttle floor in Stage 2 to prevent stall in 30 deg bank

// Slew rate limits for smooth linear transitions (per 20ms cycle = 50Hz)
constexpr int FAILSAFE_THROTTLE_SLEW_US_PER_TICK = 1;    // Strictly 1 us per tick (~50 us/s) for ultra-smooth ramp rate
constexpr float FAILSAFE_PITCH_SLEW_DEG_PER_TICK = 0.5f; // ~25 deg/s pitch slew rate
constexpr float FAILSAFE_ROLL_SLEW_DEG_PER_TICK = 0.8f;  // ~40 deg/s roll slew rate

#endif // CONFIG_H
