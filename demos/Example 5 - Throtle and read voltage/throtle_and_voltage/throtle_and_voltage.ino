#include <ESP32Servo.h>

// --- Configuração de Pinos ---
const int CH3 = 32;          // Entrada do Recetor (Throttle)
const int pinoBateria = 33;  // Entrada do Sinal 'S' do Sensor
const int pinoESC = 13;      // Saída para o sinal do ESC

// --- Variáveis de Controlo ---
Servo meuESC;
int ultimoValorESC = 0;
const int deadband = 5;

// --- Parâmetros de Calibração e Bateria ---
// Ajuste este valor até que a leitura no Serial bata certo com o teu Multímetro
// Se o multímetro diz 12.6V e o ESP diz 11.8V, o fator é 12.6 / 11.8 = 5.34
const float FATOR_CALIBRACAO = 5.34; 

void setup() {
  Serial.begin(115200);
  pinMode(CH3, INPUT);
  pinMode(pinoBateria, INPUT);
  meuESC.attach(pinoESC, 1000, 2000); 
  Serial.println("--- Sistema Calibrado Iniciado ---");
}

void loop() {
  // 1. LÓGICA DO MOTOR (ESC)
  int leituraRC = pulseIn(CH3, HIGH, 25000);
  if (leituraRC > 900 && leituraRC < 2100) {
    if (abs(leituraRC - ultimoValorESC) > deadband) {
      meuESC.writeMicroseconds(leituraRC);
      ultimoValorESC = leituraRC;
    }
  }

  // 2. LÓGICA DA BATERIA (Multímetro Digital)
  int valorADC = analogRead(pinoBateria);
  
  // Cálculo da tensão real corrigida
  float voltagemPino = (valorADC * 3.3) / 4095.0;
  float voltagemReal = voltagemPino * FATOR_CALIBRACAO;

  // 3. CÁLCULO DA PERCENTAGEM (LiPo 3S: 10.5V a 12.6V)
  float percentagem = (voltagemReal - 10.5) / (12.6 - 10.5) * 100.0;
  
  // Limitar a percentagem entre 0 e 100
  if (percentagem > 100) percentagem = 100;
  if (percentagem < 0)   percentagem = 0;

  // 4. DISPLAY NO MONITOR SERIAL
  Serial.print("Throttle: ");
  Serial.print(ultimoValorESC);
  Serial.print(" | Voltagem: ");
  Serial.print(voltagemReal, 2); // Imprime com 2 casas decimais
  Serial.print("V | Carga: ");
  Serial.print((int)percentagem);
  Serial.println("%");

  delay(100); 
}