#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

// Calibration Config
const float KNOWN_ALTITUDE = 150.0; // Starting altitude in meters

// I2C Pins
#define I2C_SDA 23
#define I2C_SCL 22

// Network Config
const char* ssid = "MEO-0A46BF";
const char* password = "E31106422B";
const char* host = "192.168.1.81"; 
const uint16_t port = 5005;

WiFiClient client;
Adafruit_BMP280 bmp;
float calibrated_sea_level_pressure; 

void connectToServer() {
  if (WiFi.status() != WL_CONNECTED) return;
  Serial.print("Connecting to TCP server...");
  while (!client.connect(host, port)) {
    Serial.print(".");
    delay(2000);
  }
  Serial.println("\n[SYSTEM] Connected to Server!");
}

void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!bmp.begin(0x77)) {
    Serial.println("Error: BMP280 not found!");
    while (1); 
  }

  // Calibration process
  Serial.println("\n--- CALIBRATION ---");
  Serial.print("Reference altitude: ");
  Serial.print(KNOWN_ALTITUDE);
  Serial.println(" m");

  float current_pressure = bmp.readPressure() / 100.0F;
  calibrated_sea_level_pressure = bmp.seaLevelForAltitude(KNOWN_ALTITUDE, current_pressure);
  
  Serial.print("Local pressure: "); Serial.print(current_pressure); Serial.println(" hPa");
  Serial.print("Calculated sea level pressure: "); Serial.print(calibrated_sea_level_pressure); Serial.println(" hPa");
  Serial.println("------------------\n");

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[SYSTEM] WiFi Connected!");

  connectToServer();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (!client.connected()) connectToServer();

  if (client.connected()) {
    float temp = bmp.readTemperature();
    float press = bmp.readPressure() / 100.0F;
    float alt = bmp.readAltitude(calibrated_sea_level_pressure);

    String dados = "Temp: " + String(temp, 1) + "C | " +
                   "Press: " + String(press, 1) + "hPa | " +
                   "Alt: " + String(alt, 1) + "m";

    Serial.println("Sending: " + dados);
    client.println(dados);

    unsigned long t = millis();
    while (client.available() == 0 && millis() - t < 500);
    while (client.available()) {
      Serial.println("Server: " + client.readStringUntil('\n'));
    }
  }

  delay(5000); 
}