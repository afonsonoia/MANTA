#include "network.h"
#include "battery.h"
#include "config.h"
#include "control.h"
#include "receiver.h"
#include "telemetry_codec.h"
#include <LoRa.h>
#include <SPI.h>

#define DIAG_LED 2

static uint8_t currentLoRaPower = LORA_TX_POWER;

void setLoRaTxPower(uint8_t powerDbm) {
  if (powerDbm < 2)
    powerDbm = 2;
  if (powerDbm > 20)
    powerDbm = 20;
  if (currentLoRaPower != powerDbm) {
    uint8_t oldVal = currentLoRaPower;
    currentLoRaPower = powerDbm;
    LoRa.setTxPower(currentLoRaPower, PA_OUTPUT_PA_BOOST_PIN);
    Serial.printf(
        "[CONFIG] Changed variable LORA_TX_POWER: [%d dBm] -> [%d dBm]\n",
        oldVal, currentLoRaPower);
  }
}

uint8_t getLoRaTxPower() { return currentLoRaPower; }

void initNetwork() {
  pinMode(DIAG_LED, OUTPUT);
  digitalWrite(DIAG_LED, LOW);

  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);

  // Hardware Reset pulse on LoRa module
  if (LORA_RST != -1) {
    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, LOW);
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    delay(10);
  }

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setSPI(SPI);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  while (!LoRa.begin(LORA_BAND)) {
    Serial.println("[LORA] Error: Failed to initialize LoRa radio module! "
                   "Check SPI wiring!");
    digitalWrite(DIAG_LED, HIGH);
    delay(150);
    digitalWrite(DIAG_LED, LOW);
    delay(350);
  }

  LoRa.setTxPower(currentLoRaPower, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.enableCrc();
  LoRa.receive();

  Serial.printf(
      "[LORA] Radio initialized successfully! Tx Power: %d dBm (PA_BOOST)\n",
      currentLoRaPower);
}

static void sendLoRaAck(const String &cmd) {
  String ackMsg = "ACK:" + cmd;
  Serial.printf("[MANTA ACK] Transmitting LoRa ACK: %s\n", ackMsg.c_str());
  if (LoRa.beginPacket()) {
    LoRa.print(ackMsg);
    LoRa.endPacket();
    LoRa.receive();
  }
}

static void processCommandString(String req) {
  req.trim();
  if (req.length() == 0)
    return;

  // Security Guard: Ignore remote configuration commands in flight mode (CH5 <=
  // 1900 & RC Active)
  uint16_t ch1 = 0, ch2 = 0, ch3 = 0, ch4 = 0, ch5 = 0;
  getReceiverChannels(ch1, ch2, ch3, ch4, ch5);
  if (!isRCSignalLost() && ch5 <= 1900) {
    return; // Block execution: strictly 1-way MANTA -> Ground Station in flight
            // mode
  }

  bool executed = false;
  if (req.startsWith("THROTTLE:")) {
    int val = req.substring(9).toInt();
    setThrottlePulse(val);
    executed = true;
  } else if (req.startsWith("CUTOFF:")) {
    float val = req.substring(7).toFloat();
    setCutoffThreshold(val);
    executed = true;
  } else if (req.startsWith("CALIB_TRIM") || req.startsWith("CALIB_NEUTRAL")) {
    calibrateNeutralCenters();
    executed = true;
  } else if (req.startsWith("SET_SERVO_ANGLE:")) {
    int angle = req.substring(16).toInt();
    setServoMaxAngle((uint8_t)angle);
    executed = true;
  } else if (req.startsWith("SET_SERVO_TRIM:")) {
    String params = req.substring(15);
    int comma1 = params.indexOf(',');
    int comma2 = params.indexOf(',', comma1 + 1);
    int comma3 = params.indexOf(',', comma2 + 1);
    if (comma1 > 0 && comma2 > comma1 && comma3 > comma2) {
      int br = params.substring(0, comma1).toInt();
      int bl = params.substring(comma1 + 1, comma2).toInt();
      int fr = params.substring(comma2 + 1, comma3).toInt();
      int fl = params.substring(comma3 + 1).toInt();
      setServoTrims((int16_t)br, (int16_t)bl, (int16_t)fr, (int16_t)fl);
      saveCalibrationToNVS();
      executed = true;
    }
  } else if (req.startsWith("SET_SERVO_INV:")) {
    String params = req.substring(14);
    int comma1 = params.indexOf(',');
    int comma2 = params.indexOf(',', comma1 + 1);
    int comma3 = params.indexOf(',', comma2 + 1);
    if (comma1 > 0 && comma2 > comma1 && comma3 > comma2) {
      bool br = params.substring(0, comma1).toInt() != 0;
      bool bl = params.substring(comma1 + 1, comma2).toInt() != 0;
      bool fr = params.substring(comma2 + 1, comma3).toInt() != 0;
      bool fl = params.substring(comma3 + 1).toInt() != 0;
      setServoInversion(br, bl, fr, fl);
      saveCalibrationToNVS();
      executed = true;
    }
  } else if (req.startsWith("SET_DEADBAND:")) {
    int val = req.substring(13).toInt();
    setRCMarginDeadband((uint8_t)val);
    saveCalibrationToNVS();
    executed = true;
  } else if (req.startsWith("SET_RC_FILTER:")) {
    String params = req.substring(14);
    int sep1 = params.indexOf(':');
    if (sep1 < 0)
      sep1 = params.indexOf(',');
    int sep2 = params.indexOf(':', sep1 + 1);
    if (sep2 < 0)
      sep2 = params.indexOf(',', sep1 + 1);
    if (sep1 > 0) {
      uint8_t fType = (uint8_t)params.substring(0, sep1).toInt();
      uint16_t wSize = 5;
      float alphaVal = 0.33f;
      if (sep2 > sep1) {
        wSize = (uint16_t)params.substring(sep1 + 1, sep2).toInt();
        alphaVal = params.substring(sep2 + 1).toFloat();
        if (alphaVal > 1.0f)
          alphaVal /= 100.0f;
      } else {
        wSize = (uint16_t)params.substring(sep1 + 1).toInt();
      }
      setRCFilterConfig(fType, wSize, alphaVal);
      executed = true;
    }
  } else if (req.startsWith("SET_LORA_POWER:")) {
    int val = req.substring(15).toInt();
    setLoRaTxPower((uint8_t)val);
    executed = true;
  } else if (req.startsWith("SET_SERVO_INTERVAL:")) {
    int val = req.substring(19).toInt();
    setServoUpdateInterval((uint16_t)val);
    executed = true;
  } else if (req.startsWith("CALIB_SAVE")) {
    saveCalibrationToNVS();
    executed = true;
  }

  if (executed) {
    sendLoRaAck(req);
  }
}

void handleNetworkCommands() {
  // 1. Process incoming commands over USB Serial
  while (Serial.available()) {
    String serialReq = Serial.readStringUntil('\n');
    processCommandString(serialReq);
  }

  // 2. Process incoming commands over LoRa RF
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String req = "";
    while (LoRa.available()) {
      req += (char)LoRa.read();
    }
    processCommandString(req);
  }
}

void sendTelemetry(float rawADC, float batteryVoltage, float pitch, float roll,
                   float yaw, float effectiveCutoff, double lat, double lon,
                   float alt, float temp, int sats, int fix, uint16_t rch1,
                   uint16_t rch2, uint16_t rch3, uint16_t rch4, uint16_t rch5,
                   bool rcSignalLost) {
  static uint32_t packetCount = 0;
  static bool ledState = false;
  packetCount++;

  // Toggle LED state (ON <-> OFF) every 2 telemetry packets
  if (packetCount % 2 == 0) {
    ledState = !ledState;
    digitalWrite(DIAG_LED, ledState ? HIGH : LOW);
  }

  uint8_t activeDeadband = getRCMarginDeadband();
  uint8_t flags = (rcSignalLost ? 1 : 0) | ((rch5 > 1900) ? 2 : 0);

  Serial.printf(
      "[LORA TX 4Hz] BAT_V:%.2fV | BAT_ADC:%.1f | Pitch:%.1f | Roll:%.1f | "
      "Yaw:%.1f | Cutoff:%.2f | DB:%dus | Lat:%.6f | Lon:%.6f | Alt:%.1f | "
      "Temp:%.1f | Sats:%d | RC:[%u,%u,%u,%u,%u] SIG:%d\n",
      batteryVoltage, rawADC, pitch, roll, yaw, effectiveCutoff, activeDeadband,
      lat, lon, alt, temp, sats, rch1, rch2, rch3, rch4, rch5,
      rcSignalLost ? 0 : 1);

  MantaTelemetryPacket pkt;
  encode_telemetry_packet(&pkt, batteryVoltage, rawADC, pitch, roll, yaw,
                          effectiveCutoff, activeDeadband, lat, lon, alt, temp,
                          sats, fix, rch1, rch2, rch3, rch4, rch5, flags);

  if (LoRa.beginPacket()) {
    LoRa.write((const uint8_t *)&pkt, sizeof(MantaTelemetryPacket));

    // Synchronous transmission to ensure packet finishes sending over the air
    // (~55ms)
    LoRa.endPacket();

    LoRa.receive(); // Re-enable receive mode
  }
}
