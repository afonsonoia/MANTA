#include <WiFi.h>
#include <ESP32Servo.h>

// Access Point Configuration (Open Network - No Password)
const char* ssid = "ESP32_Battery_Monitor";
const uint16_t port = 5005;

// Hardware Configuration
const int pinBattery = 36; // ESP32 Pin VP / GPIO36
const int pinESC = 25;     // ESC Control Pin D25

const float BATTERY_DIVIDER_RATIO = 4.84f;   // HiLetgo 0–25V Voltage Sensor (4.84:1)
const float ADC_REFERENCE = 3.3f;
const float ADC_MAX = 4095.0f;
const unsigned long interval = 5000; // 5 second logging interval
const float MIN_CUTOFF_VOLTAGE = 12.50; // Low Voltage Cutoff safety threshold (12.5V)

bool lowVoltageCutoffTriggered = false;
unsigned long previousMillis = 0;
WiFiServer server(port);
WiFiClient client;
Servo myESC;
int currentThrottlePulse = 1000; // Default off/armed pulse (1000us)

void setup() {
  Serial.begin(115200);
  pinMode(pinBattery, INPUT);
  
  // Set 12-bit resolution and 11dB pin attenuation (~3.3V max)
  analogReadResolution(12);
  analogSetPinAttenuation(pinBattery, ADC_11db);

  // Initialize ESC on GPIO 25
  myESC.attach(pinESC, 1000, 2000);
  myESC.writeMicroseconds(currentThrottlePulse); // Arm ESC at 1000us

  Serial.println("\n[WiFi] Setting up Access Point...");
  WiFi.mode(WIFI_AP);
  if (WiFi.softAP(ssid)) { // Default AP IP: 192.168.4.1
    IPAddress apIP = WiFi.softAPIP();
    Serial.print("[WiFi] Access Point '");
    Serial.print(ssid);
    Serial.println("' created successfully!");
    Serial.print("[WiFi] AP IP Address: ");
    Serial.println(apIP);
  } else {
    Serial.println("[WiFi] Failed to create Access Point!");
  }

  server.begin();
  Serial.print("[TCP] Server listening on port ");
  Serial.println(port);
  Serial.println("[ESC] Initialized on GPIO 25 at 1000us (Off/Armed)");
  Serial.println("[SAFETY] Low Voltage Cutoff active at <= 12.50V");
}

void loop() {
  // Check for incoming TCP commands from client (logger.py)
  WiFiClient newClient = server.available();
  if (newClient) {
    client = newClient;
  }

  if (client && client.connected()) {
    while (client.available()) {
      String req = client.readStringUntil('\n');
      req.trim();
      if (req.startsWith("THROTTLE:")) {
        int val = req.substring(9).toInt();
        if (lowVoltageCutoffTriggered) {
          Serial.println("[SAFETY BLOCKED] Throttle command REJECTED because Battery Voltage <= 12.0V!");
          currentThrottlePulse = 1000;
          myESC.writeMicroseconds(1000);
        } else if (val >= 1000 && val <= 2000) {
          if (val != currentThrottlePulse) {
            currentThrottlePulse = val;
            myESC.writeMicroseconds(currentThrottlePulse);
            Serial.print("[ESC] Throttle command received -> Updated to: ");
            Serial.print(currentThrottlePulse);
            Serial.println(" us");
          }
        }
      }
    }
  }

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Multi-sample oversampling (64 readings) to filter out ESC / BLDC electrical noise
    long sum = 0;
    for (int i = 0; i < 64; i++) {
      sum += analogRead(pinBattery);
      delayMicroseconds(100);
    }
    float rawInput = (float)sum / 64.0f;
    float voltageADC = (rawInput / ADC_MAX) * ADC_REFERENCE;
    float currentBatteryVoltage = -0.0000009f * voltageADC * voltageADC + 0.0089f * voltageADC - 5.8868f;

    if (currentBatteryVoltage < 0.0f) currentBatteryVoltage = 0.0f;

    // Low Voltage Cutoff Protection Trigger
    if (currentBatteryVoltage <= MIN_CUTOFF_VOLTAGE && currentBatteryVoltage > 0.0f) {
      if (!lowVoltageCutoffTriggered) {
        lowVoltageCutoffTriggered = true;
        Serial.println("\n[CRITICAL SAFETY ALERT] Battery voltage <= 12.0V! LOW VOLTAGE CUTOFF TRIGGERED!");
      }
      if (currentThrottlePulse != 1000) {
        currentThrottlePulse = 1000;
        myESC.writeMicroseconds(1000);
        Serial.println("[CRITICAL SAFETY ALERT] Motor FORCE CUT OFF (1000us) due to Low Voltage!");
      }
    }

    Serial.print("Raw ADC: ");
    Serial.print(rawInput);
    Serial.print(" | ADC Voltage: ");
    Serial.print(voltageADC, 3);
    Serial.print(" V | Battery Voltage: ");
    Serial.print(currentBatteryVoltage, 2);
    Serial.print(" V | ESC: ");
    Serial.print(currentThrottlePulse);
    Serial.println(lowVoltageCutoffTriggered ? " us [LOCKED]" : " us");

    // Broadcast raw ADC reading (unconverted sensor value) to connected TCP client
    if (client && client.connected()) {
      client.println(rawInput);
    }
  }
}
