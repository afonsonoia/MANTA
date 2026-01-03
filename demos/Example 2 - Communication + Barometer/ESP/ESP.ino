#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

// ==========================================
// CONFIGURAÇÕES DE CALIBRAÇÃO
// ==========================================
const float ALTITUDE_CONHECIDA = 150.0; // Altera para a altitude inicial em metros
// ==========================================

// Definição dos pins (lado a lado)
#define I2C_SDA 23
#define I2C_SCL 22

// Configurações de Rede
const char* ssid = "MEO-0A46BF";
const char* password = "E31106422B";
const char* host = "192.168.1.81"; 
const uint16_t port = 5005;

WiFiClient client;
Adafruit_BMP280 bmp;
float pressao_nivel_mar_calibrada; // Variável global para guardar a calibração

void conectarAoServidor() {
  if (WiFi.status() != WL_CONNECTED) return;
  Serial.print("A conectar ao servidor TCP...");
  while (!client.connect(host, port)) {
    Serial.print(".");
    delay(2000);
  }
  Serial.println("\n[SISTEMA] Conectado ao Servidor!");
}

void setup() {
  Serial.begin(115200);

  // Inicializa I2C nos pins 23 e 22
  Wire.begin(I2C_SDA, I2C_SCL);

  // Inicializa Sensor
  if (!bmp.begin(0x77)) {
    Serial.println("Erro: Sensor BMP280 não encontrado!");
    while (1); 
  }

  // --- PROCESSO DE CALIBRAÇÃO NO SETUP ---
  Serial.println("\n--- CALIBRAÇÃO ---");
  Serial.print("Altitude de referência definida: ");
  Serial.print(ALTITUDE_CONHECIDA);
  Serial.println(" m");

  // Lê a pressão atual no local
  float pressao_atual = bmp.readPressure() / 100.0F;
  
  // Calcula a pressão ao nível do mar com base na altitude conhecida
  // Esta função diz: "Se estou a X metros e a pressão é Y, então ao nível do mar seria Z"
  pressao_nivel_mar_calibrada = bmp.seaLevelForAltitude(ALTITUDE_CONHECIDA, pressao_atual);
  
  Serial.print("Pressão local: "); Serial.print(pressao_atual); Serial.println(" hPa");
  Serial.print("Pressão ao nível do mar calculada: "); Serial.print(pressao_nivel_mar_calibrada); Serial.println(" hPa");
  Serial.println("------------------\n");

  WiFi.begin(ssid, password);
  Serial.print("Conectando ao WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[SISTEMA] WiFi Conectado!");

  conectarAoServidor();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (!client.connected()) conectarAoServidor();

  if (client.connected()) {
    float temp = bmp.readTemperature();
    float press = bmp.readPressure() / 100.0F;
    
    // Agora usamos a pressão de nível do mar que calculámos no setup
    float alt = bmp.readAltitude(pressao_nivel_mar_calibrada);

    String dados = "Temp: " + String(temp, 1) + "C | " +
                   "Press: " + String(press, 1) + "hPa | " +
                   "Alt: " + String(alt, 1) + "m";

    Serial.println("A enviar: " + dados);
    client.println(dados);

    unsigned long t = millis();
    while (client.available() == 0 && millis() - t < 500);
    while (client.available()) {
      Serial.println("Servidor: " + client.readStringUntil('\n'));
    }
  }

  delay(5000); 
}