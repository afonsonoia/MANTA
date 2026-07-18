#include <WiFi.h>

const char* ssid = "MEO-0A46BF";
const char* password = "E31106422B";
const char* host = "192.168.1.81"; 
const uint16_t port = 5005;

WiFiClient client;

void connectToServer() {
  Serial.print("Connecting to TCP server...");
  while (!client.connect(host, port)) {
    Serial.print(".");
    delay(2000);
  }
  Serial.println("\nConnected to server!");
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");

  connectToServer();
}

void loop() {
  // CRITICAL: Reconnect if connection is lost
  if (!client.connected()) {
    Serial.println("Connection lost!");
    connectToServer();
  }

  // Send data
  Serial.println("Sending data...");
  client.println("Hello from ESP32 via TCP!"); 

  // Wait for response
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 1000) { 
      Serial.println("No response from server (Timeout)");
      break;
    }
  }

  while (client.available()) {
    String response = client.readStringUntil('\n');
    Serial.println("Server response: " + response);
  }

  delay(5000); 
}