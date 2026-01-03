#include <WiFi.h>

const char* ssid = "MEO-0A46BF";
const char* password = "E31106422B";
const char* host = "192.168.1.81"; 
const uint16_t port = 5005;

WiFiClient client;

void conectarAoServidor() {
  Serial.print("A tentar ligar ao servidor TCP...");
  // Tenta ligar. Se não conseguir, espera 2 segundos e tenta de novo
  while (!client.connect(host, port)) {
    Serial.print(".");
    delay(2000);
  }
  Serial.println("\nConectado ao servidor!");
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  Serial.print("Conectando ao WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado!");

  conectarAoServidor();
}

void loop() {
  // VERIFICAÇÃO CRÍTICA: Se a ligação caiu, tenta ligar de novo
  if (!client.connected()) {
    Serial.println("Ligação perdida!");
    conectarAoServidor();
  }

  // Envia uma mensagem com \n no fim
  Serial.println("A enviar dados...");
  client.println("Olá do ESP32 via TCP!"); // println já adiciona o \n automático

  // Lê a resposta do PC
  // Usamos um while pequeno para dar tempo ao PC de responder
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 1000) { // Espera no máximo 1 segundo
      Serial.println("Sem resposta do servidor (Timeout)");
      break;
    }
  }

  while (client.available()) {
    String resposta = client.readStringUntil('\n');
    Serial.println("Resposta do PC: " + resposta);
  }

  delay(5000); 
}