#include <ESP32Servo.h>

// Top row input pins
const int CH1 = 34; // D34 - rollerons
const int CH2 = 35; // D35 - elevator
const int CH3 = 32; // D32 - throttle
const int CH5 = 33; // D33 - swA

const int pinBatery = 36; // VP - Voltage Sensor
const int pinESC = 13;    // ESC control
const int pinLAileron = 25; // Left aileron
const int pinRAileron = 26; // Right aileron
const int pinElevator = 27; // Elevator

// Control variables
Servo ESC;
Servo LAileron;
Servo RAileron;
Servo Elevator;

const int channelsInputLen = 4;
const int channelsInputPins[] = {CH1, CH2, CH3, CH5};
int channelsInput[] = {1, 1, 1, 1}; // Values between 1000 and 2000

// Battery sensor
const float BATERY_CALIBRATION_FACTOR = 5.34;
int currentBateryPercentage = 999;
float currentBateryVoltage = 999;

// Deadband control variables
int lastESCinput = -1;
int lastAileronInput = -1;
int lastElevatorInput = -1;
const int deadband = 3;

void setAilerons(int valueRaw) {
  if (abs(valueRaw - lastAileronInput) > deadband) {
    Serial.print("Input value: "); Serial.print(valueRaw); Serial.print(" - ");
    
    int master_angle = ((float)valueRaw - 1000) / 1000 * 180;
    if(master_angle <= -10) { master_angle = 90; }

    float angleRraw = master_angle / 180.0;
    float angleLraw = (180 - master_angle) / 180.0;

    // Define limits
    const int maxR = 180;
    const int minR = 0;
    const int offsetR = 0;

    const int maxL = 180;
    const int minL = 0;
    const int offsetL = 0;

    int angleR = (maxR - minR) * angleRraw + minR + offsetR;
    int angleL = (maxL - minL) * angleLraw + minL + offsetL;

    Serial.print("AngleR: "); Serial.print(angleR); Serial.print(" AngleL: "); Serial.print(angleL); Serial.println();
    Serial.print("Master: "); Serial.print(master_angle); Serial.println();
    Serial.println();
    
    RAileron.write(angleR);
    LAileron.write(angleL);
    
    lastAileronInput = valueRaw;
  }
}

void setESC(int valueRaw) {
  // Security stop (Ensure channelsInput[3] is correct dynamically)
  if(channelsInput[3] <= 1050) { 
    ESC.writeMicroseconds(1000);
    lastESCinput = 1000;
    return;
  }

  if (abs(valueRaw - lastESCinput) > deadband) {
    if(valueRaw < 1050) {
      ESC.writeMicroseconds(1000);
    } else {
      ESC.writeMicroseconds(valueRaw);
    }
    lastESCinput = valueRaw;
  }
}

void setElevator(int valueRaw) {
  if (abs(valueRaw - lastElevatorInput) > deadband) {
    int angle = ((float)valueRaw - 1000) / 1000 * 180;
    if(angle < -5) {
      angle = 90;
    }
    Elevator.write(angle);
    lastElevatorInput = valueRaw;
  }
}

void getReceiverValues() {
  for(int i = 0; i < channelsInputLen; i++) {
    channelsInput[i] = pulseIn(channelsInputPins[i], HIGH, 25000);
  }
}

void getBateryValues() {
  int rawInput = analogRead(pinBatery);
  float voltageRaw = (rawInput * 3.3) / 4095.0;
  currentBateryVoltage = voltageRaw * BATERY_CALIBRATION_FACTOR;
  currentBateryPercentage = (currentBateryVoltage - 11.0) / (12.7 - 11.0) * 100.0;
  Serial.print(currentBateryVoltage); Serial.print("V - ");
  Serial.print(currentBateryPercentage); Serial.print("%\n");
}

void setup() {
  Serial.begin(115200);

  // Inputs
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH5, INPUT);

  pinMode(pinBatery, INPUT);

  // Outputs
  ESC.attach(pinESC, 1000, 2000);
  LAileron.attach(pinLAileron, 500, 2400); 
  RAileron.attach(pinRAileron, 500, 2400); 
  Elevator.attach(pinElevator, 500, 2400); 
}

void loop() {
  getReceiverValues();
  getBateryValues();
  setElevator(channelsInput[1]);
  setAilerons(channelsInput[0]);
  int ESCinput = pulseIn(CH3, HIGH, 25000);
  setESC(ESCinput);
}