const int pinBatery = 36; // ESP32 Pin VP / GPIO36
const float BATERY_CALIBRATION_FACTOR = 5.34;
const unsigned long interval = 5000;

unsigned long previousMillis = 0;

void setup() {
  Serial.begin(115200);
  pinMode(pinBatery, INPUT);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    int rawInput = analogRead(pinBatery);
    float voltageRaw = (rawInput * 3.3) / 4095.0;
    float currentBateryVoltage = voltageRaw * BATERY_CALIBRATION_FACTOR;

    Serial.println(currentBateryVoltage);
  }
}
