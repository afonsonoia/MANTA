#include <ESP32Servo.h>

// Definição dos 6 pinos de entrada (Fila de Cima)
const int CH1 = 34; // D34        - rollerons
const int CH2 = 35; // D35        - elevador
const int CH3 = 32; // D32        - throttle
const int CH5 = 39; // Pino VN    - swA
const int CH6 = 36; // Pino VP    - swD

const int pinBatery = 33;  // Voltage Sensor
const int pinESC = 13;      // ESC control
const int pinLAileron = 27;      // Left aileron control
const int pinRAileron = 26;      // Right aileron control
const int pinElevator = 25;      // Elevator control

// control variables
Servo ESC;
Servo LAileron;
Servo RAileron;
Servo Elevator;

int lastESCval = 0;
const int deadBand = 5;

const int channelsInputLen = 5;
const int channelsInputPins[] = {CH1, CH2, CH3, CH5, CH6};
int channelsInput[] = {1, 1, 1, 1, 1}; // vals between 1000 and 2000

// batery sensor
const float BATERY_CALIBRATION_FACTOR = 5.34;
int currentBateryPercentage = 999;
float currentBateryVoltage = 999;


void setElevator(int valueRaw){
  Serial.print("Input value: "); Serial.print(valueRaw); Serial.print(" - ");
  
  int angle = ((float)valueRaw-1000)/1000*180;
  Serial.print("Angle: "); Serial.print(angle); Serial.println();
  Elevator.write(angle);
}

void getReceiverValues(){
  for(int i=0; i<channelsInputLen; i++){
    channelsInput[i] = pulseIn(channelsInputPins[i], HIGH, 25000);
    //Serial.print(channelsInput[i]); Serial.print(" - ");
  }
  //Serial.println();
}

void getBateryValues(){
  int rawInput = analogRead(pinBatery);
  float voltageRaw = (rawInput * 3.3) / 4095.0;
  currentBateryVoltage = voltageRaw * BATERY_CALIBRATION_FACTOR;
  currentBateryPercentage = (currentBateryVoltage - 11.0) / (12.6 - 11.0) * 100.0;
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
    pinMode(CH6, INPUT);

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
  

}
