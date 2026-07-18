#include <ESP32Servo.h>

// Pin config
const int CH3 = 32;          // Receiver Throttle
const int pinBattery = 36;   // Voltage sensor 'S' pin (VP)
const int pinESC = 13;       // ESC control

// Control variables
Servo myESC;
int lastESCValue = 0;
const int deadband = 5;

// Calibration factor
const float CALIBRATION_FACTOR = 5.34; 

void setup() {
  Serial.begin(115200);
  pinMode(CH3, INPUT);
  pinMode(pinBattery, INPUT);
  myESC.attach(pinESC, 1000, 2000); 
  Serial.println("--- System Started ---");
}

void loop() {
  // Motor logic
  int rcReading = pulseIn(CH3, HIGH, 25000);
  if (rcReading > 900 && rcReading < 2100) {
    if (abs(rcReading - lastESCValue) > deadband) {
      myESC.writeMicroseconds(rcReading);
      lastESCValue = rcReading;
    }
  }

  // Battery logic
  int adcValue = analogRead(pinBattery);
  
  float pinVoltage = (adcValue * 3.3) / 4095.0;
  float realVoltage = pinVoltage * CALIBRATION_FACTOR;

  // Percentage (LiPo 3S: 10.5V to 12.6V)
  float percentage = (realVoltage - 10.5) / (12.6 - 10.5) * 100.0;
  
  if (percentage > 100) percentage = 100;
  if (percentage < 0)   percentage = 0;

  Serial.print("Throttle: ");
  Serial.print(lastESCValue);
  Serial.print(" | Voltage: ");
  Serial.print(realVoltage, 2); 
  Serial.print("V | Charge: ");
  Serial.print((int)percentage);
  Serial.println("%");

  delay(100); 
}