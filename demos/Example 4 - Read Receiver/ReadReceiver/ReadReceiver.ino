// Definição dos 6 pinos de entrada (Fila de Cima)
const int CH1 = 34; // Input Only - rollerons
const int CH2 = 35; // Input Only - elevador
const int CH3 = 32; //            - throttle
const int CH5 = 39; // Pino VN    - swA
const int CH6 = 36; // Pino VP    - swD

void setup() {
  Serial.begin(115200);
  
  // Configurar todos como entrada
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH5, INPUT);
  pinMode(CH6, INPUT);
  
  Serial.println("--- Sistema Iniciado: Lendo 6 Canais ---");
}

void loop() {
  // Leitura dos pulsos (em microssegundos)
  // O timeout de 25000us (25ms) evita que o código congele se um fio soltar
  long val1 = pulseIn(CH1, HIGH, 25000);
  long val2 = pulseIn(CH2, HIGH, 25000);
  long val3 = pulseIn(CH3, HIGH, 25000);
  long val5 = pulseIn(CH5, HIGH, 25000);
  long val6 = pulseIn(CH6, HIGH, 25000);

  // Formatação para facilitar a leitura no monitor serial
  Serial.print("C1:"); Serial.print(val1);
  Serial.print(" C2:"); Serial.print(val2);
  Serial.print(" C3:"); Serial.print(val3);

  Serial.print(" C5:"); Serial.print(val5);
  Serial.print(" C6:"); Serial.println(val6);

  delay(50); // Ajusta a velocidade de atualização
}