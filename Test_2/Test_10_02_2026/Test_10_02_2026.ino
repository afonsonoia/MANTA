#include <ESP32Servo.h>

// Definição dos 6 pinos de entrada (Fila de Cima)
const int CH1 = 34; // D34        - rollerons
const int CH2 = 35; // D35        - elevador
const int CH3 = 32; // D32        - throttle
const int CH5 = 39; // Pino VN    - swA
const int CH6 = 36; // Pino VP    - swD

const int pinBatery = 33;  // Voltage Sensor
const int pinESC = 13;      // ESC control
const int pinLAileron = 25 ;      // Left aileron control
const int pinRAileron = 26;      // Right aileron control
const int pinElevator = 27;      // Elevator control

// control variables
Servo ESC;
Servo LAileron;
Servo RAileron;
Servo Elevator;

const int channelsInputLen = 5;
const int channelsInputPins[] = {CH1, CH2, CH3, CH5, CH6};
int channelsInput[] = {1, 1, 1, 1, 1}; // vals between 1000 and 2000

// batery sensor
const float BATERY_CALIBRATION_FACTOR = 5.34;
int currentBateryPercentage = 999;
float currentBateryVoltage = 999;

// --- VARIAVEIS DE CONTROLO DE DEADBAND (Alterado) ---
int lastESCinput = -1;
int lastAileronInput = -1; // Novo
int lastElevatorInput = -1; // Novo
const int deadband = 3; // Alterado de 3 para 5

void setAilerons(int valueRaw){
  // Apenas executa se a diferença for maior que a margem (deadband)
  if (abs(valueRaw - lastAileronInput) > deadband) {
    
      Serial.print("Input value: "); Serial.print(valueRaw); Serial.print(" - ");
      
      int master_angle = ((float)valueRaw-1000)/1000*180;
      if(master_angle <= -10){ master_angle = 90; }

      float angleRraw = master_angle/180;
      float angleLraw = (180-master_angle)/180;

      // define limits
      const int maxR = 180;
      const int minR = 0;
      const int offsetR = 0;

      const int maxL = 180;
      const int minL = 0;
      const int offsetL = 0;

      int angleR = (maxR-minR) * angleRraw + minR + offsetR;
      int angleL = (maxL-minL) * angleLraw + minL + offsetL;

      Serial.print("AngleR: "); Serial.print(angleR); Serial.print(" AngleL: "); Serial.print(angleL); Serial.println();
      Serial.print("Master: "); Serial.print(master_angle); Serial.println();
      Serial.println();
      
      RAileron.write(angleR);
      LAileron.write(angleL);
      
      lastAileronInput = valueRaw; // Atualiza o ultimo valor
  }
}

void setESC(int valueRaw){
  
  if(channelsInputPins[3] <= 1050){ // security stop (Nota: verifique se channelsInputPins[3] lê o pino correto dinamicamente ou se devia ler channelsInput[3])
    ESC.writeMicroseconds(1000);
    lastESCinput = 1000;
    return;
  }

  if (abs(valueRaw - lastESCinput) > deadband) {
    if(valueRaw < 1050){
      ESC.writeMicroseconds(1000);
    } else {
      ESC.writeMicroseconds(valueRaw);
    }
      lastESCinput = valueRaw;
  }
  
}

void setElevator(int valueRaw){
  // Apenas executa se a diferença for maior que a margem (deadband)
  if (abs(valueRaw - lastElevatorInput) > deadband) {
      //Serial.print("Input value: "); Serial.print(valueRaw); Serial.print(" - ");
      
      int angle = ((float)valueRaw-1000)/1000*180;
      if(angle < -5){
        angle = 90;
      }
      //Serial.print("Angle: "); Serial.print(angle); Serial.println();
      Elevator.write(angle);
      
      lastElevatorInput = valueRaw; // Atualiza o ultimo valor
  }
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
  setAilerons(channelsInput[0]);
  int ESCinput = pulseIn(CH3, HIGH, 25000);
  setESC(ESCinput);
  
}