// Top row input pins
const int CH1 = 34; // Input Only - rollerons
const int CH2 = 35; // Input Only - elevator
const int CH3 = 32; // throttle
const int CH5 = 39; // VN pin - swA
const int CH6 = 36; // VP pin - swD

void setup() {
  Serial.begin(115200);
  
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH5, INPUT);
  pinMode(CH6, INPUT);
  
  Serial.println("--- System Started: Reading 6 Channels ---");
}

void loop() {
  // Read pulses (in microseconds). Timeout of 25ms to prevent freezing.
  long val1 = pulseIn(CH1, HIGH, 25000);
  long val2 = pulseIn(CH2, HIGH, 25000);
  long val3 = pulseIn(CH3, HIGH, 25000);
  long val5 = pulseIn(CH5, HIGH, 25000);
  long val6 = pulseIn(CH6, HIGH, 25000);

  Serial.print("C1:"); Serial.print(val1);
  Serial.print(" C2:"); Serial.print(val2);
  Serial.print(" C3:"); Serial.print(val3);

  Serial.print(" C5:"); Serial.print(val5);
  Serial.print(" C6:"); Serial.println(val6);

  delay(50);
}