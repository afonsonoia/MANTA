#include "gps.h"
#include "config.h"
#include <HardwareSerial.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static HardwareSerial gpsSerial(2);

static portMUX_TYPE gpsMux = portMUX_INITIALIZER_UNLOCKED;

static volatile int32_t gpsLatE7 = 0;
static volatile int32_t gpsLonE7 = 0;
static volatile int16_t gpsAltX10 = 0;
static volatile uint8_t gpsSatellites = 0;
static volatile uint8_t gpsFixType = 0; // 0 = No Fix, 2 = 2D, 3 = 3D

static volatile double gpsLatitude = 0.0;
static volatile double gpsLongitude = 0.0;
static volatile float gpsAltitude = 0.0f;

static char nmeaBuf[128];
static uint8_t nmeaIdx = 0;

// GPS Hardware auto-detection state machine
static GPSHardwareState hwState = GPS_HW_PROBING_DEFAULT;
static unsigned long stateTimer = 0;
static unsigned long lastValidNmeaTime = 0;
static uint32_t validNmeaCount = 0;

static double convertNMEAToDecimal(const char *raw, char hemisphere) {
    if (!raw || raw[0] == '\0') return 0.0;
    double val = atof(raw);
    double degrees = floor(val / 100.0);
    double minutes = val - (degrees * 100.0);
    double dec = degrees + (minutes / 60.0);
    if (hemisphere == 'S' || hemisphere == 's' || hemisphere == 'W' || hemisphere == 'w') {
        dec = -dec;
    }
    return dec;
}

static bool extractField(const char *sentence, int fieldIndex, char *out, size_t maxLen) {
    int currentField = 0;
    const char *p = sentence;
    while (*p && currentField < fieldIndex) {
        if (*p == ',') {
            currentField++;
        }
        p++;
    }
    if (currentField != fieldIndex) {
        if (maxLen > 0) out[0] = '\0';
        return false;
    }
    size_t i = 0;
    while (*p && *p != ',' && *p != '*' && *p != '\r' && *p != '\n' && i < maxLen - 1) {
        out[i++] = *p++;
    }
    out[i] = '\0';
    return true;
}

static void parseNMEASentence(const char *sentence) {
    if (!sentence || sentence[0] != '$') return;

    // Checksum verification: XOR of characters between '$' and '*'
    const char *star = strchr(sentence, '*');
    if (star != nullptr) {
        uint8_t expectedChk = (uint8_t)strtol(star + 1, nullptr, 16);
        uint8_t calcChk = 0;
        for (const char *p = sentence + 1; p < star; p++) {
            calcChk ^= (uint8_t)(*p);
        }
        if (calcChk != expectedChk) {
            return; // Discard corrupted NMEA sentence with checksum error
        }
    }

    // Check for valid NMEA talker IDs ($GP for GPS, $GN for multi-GNSS, $BD, $GA)
    bool isValidNmea = (strncmp(sentence, "$GP", 3) == 0 ||
                        strncmp(sentence, "$GN", 3) == 0 ||
                        strncmp(sentence, "$BD", 3) == 0 ||
                        strncmp(sentence, "$GA", 3) == 0);

    if (isValidNmea) {
        lastValidNmeaTime = millis();
        validNmeaCount++;
        if (hwState == GPS_HW_PROBING_DEFAULT) {
            hwState = GPS_HW_LOCKED_NORMAL;
            Serial.printf("[GPS AUTO-DETECT] OK: NMEA stream validated on default pins (RX=GPIO%d, TX=GPIO%d).\n",
                          GPS_RX_PIN, GPS_TX_PIN);
        } else if (hwState == GPS_HW_PROBING_SWAPPED) {
            hwState = GPS_HW_LOCKED_SWAPPED;
            Serial.printf("[GPS AUTO-DETECT] SUCCESS: Swapped RX/TX wires detected and auto-corrected via GPIO matrix (RX=GPIO%d, TX=GPIO%d).\n",
                          GPS_TX_PIN, GPS_RX_PIN);
        }
    }

    // 1. Parse GGA Sentences ($GPGGA, $GNGGA)
    if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0) {
        char rawLat[20] = "", latHem[4] = "";
        char rawLon[20] = "", lonHem[4] = "";
        char fixStr[8] = "", satStr[8] = "", altStr[16] = "";

        extractField(sentence, 2, rawLat, sizeof(rawLat));
        extractField(sentence, 3, latHem, sizeof(latHem));
        extractField(sentence, 4, rawLon, sizeof(rawLon));
        extractField(sentence, 5, lonHem, sizeof(lonHem));
        extractField(sentence, 6, fixStr, sizeof(fixStr));
        extractField(sentence, 7, satStr, sizeof(satStr));
        extractField(sentence, 9, altStr, sizeof(altStr));

        uint8_t newSats = gpsSatellites;
        if (satStr[0] != '\0') {
            newSats = (uint8_t)atoi(satStr);
        }

        int fixVal = atoi(fixStr);
        uint8_t newFix = 0;
        double newLat = gpsLatitude;
        double newLon = gpsLongitude;
        int32_t newLatE7 = gpsLatE7;
        int32_t newLonE7 = gpsLonE7;
        float newAlt = gpsAltitude;
        int16_t newAltX10 = gpsAltX10;

        if (fixVal > 0) {
            newFix = (newSats >= 4) ? 3 : 2; // 3D Fix or 2D Fix
            if (rawLat[0] != '\0' && rawLon[0] != '\0') {
                newLat  = convertNMEAToDecimal(rawLat, latHem[0] ? latHem[0] : 'N');
                newLon = convertNMEAToDecimal(rawLon, lonHem[0] ? lonHem[0] : 'E');
                newLatE7 = (int32_t)round(newLat * 1e7);
                newLonE7 = (int32_t)round(newLon * 1e7);
            }
            if (altStr[0] != '\0') {
                newAlt = (float)atof(altStr);
                newAltX10 = (int16_t)round(constrain(newAlt, -1000.0f, 3200.0f) * 10.0f);
            }
        }

        portENTER_CRITICAL(&gpsMux);
        gpsSatellites = newSats;
        gpsFixType = newFix;
        gpsLatitude = newLat;
        gpsLongitude = newLon;
        gpsLatE7 = newLatE7;
        gpsLonE7 = newLonE7;
        gpsAltitude = newAlt;
        gpsAltX10 = newAltX10;
        portEXIT_CRITICAL(&gpsMux);
    }
    // 2. Parse RMC Sentences ($GPRMC, $GNRMC)
    else if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0) {
        char status[4] = "";
        char rawLat[20] = "", latHem[4] = "";
        char rawLon[20] = "", lonHem[4] = "";

        extractField(sentence, 2, status, sizeof(status));
        extractField(sentence, 3, rawLat, sizeof(rawLat));
        extractField(sentence, 4, latHem, sizeof(latHem));
        extractField(sentence, 5, rawLon, sizeof(rawLon));
        extractField(sentence, 6, lonHem, sizeof(lonHem));

        if (status[0] == 'A') {
            double newLat = gpsLatitude;
            double newLon = gpsLongitude;
            int32_t newLatE7 = gpsLatE7;
            int32_t newLonE7 = gpsLonE7;
            if (rawLat[0] != '\0' && rawLon[0] != '\0') {
                newLat  = convertNMEAToDecimal(rawLat, latHem[0] ? latHem[0] : 'N');
                newLon = convertNMEAToDecimal(rawLon, lonHem[0] ? lonHem[0] : 'E');
                newLatE7 = (int32_t)round(newLat * 1e7);
                newLonE7 = (int32_t)round(newLon * 1e7);
            }
            portENTER_CRITICAL(&gpsMux);
            if (gpsFixType == 0) {
                gpsFixType = (gpsSatellites >= 4) ? 3 : 2;
            }
            gpsLatitude = newLat;
            gpsLongitude = newLon;
            gpsLatE7 = newLatE7;
            gpsLonE7 = newLonE7;
            portEXIT_CRITICAL(&gpsMux);
        } else if (status[0] == 'V') {
            // Receiver Warning: Navigation data invalid
            portENTER_CRITICAL(&gpsMux);
            gpsFixType = 0;
            portEXIT_CRITICAL(&gpsMux);
        }
    }
}

void initGPS() {
    hwState = GPS_HW_PROBING_DEFAULT;
    stateTimer = millis();
    lastValidNmeaTime = 0;
    validNmeaCount = 0;
    nmeaIdx = 0;

    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.printf("[GPS] Initialized on RX=GPIO%d, TX=GPIO%d @ %ld baud (Auto-Swap Protect Active)\n",
                  GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);
}

void updateGPS() {
    // 1. Drain UART FIFO & parse incoming characters
    while (gpsSerial.available() > 0) {
        char c = (char)gpsSerial.read();
        if (c == '$') {
            // Instant resynchronization on sentence boundary: discard any prior noise
            nmeaBuf[0] = '$';
            nmeaIdx = 1;
        } else if (c == '\n' || c == '\r') {
            if (nmeaIdx > 0) {
                nmeaBuf[nmeaIdx] = '\0';
                parseNMEASentence(nmeaBuf);
                nmeaIdx = 0;
            }
        } else if (nmeaIdx > 0) {
            // Only collect payload characters after seeing start '$'
            if (nmeaIdx < sizeof(nmeaBuf) - 1) {
                nmeaBuf[nmeaIdx++] = c;
            } else {
                // Buffer overflow protection: reset index
                nmeaIdx = 0;
            }
        }
    }

    unsigned long now = millis();
    constexpr unsigned long PROBE_TIMEOUT_MS = 3000;
    constexpr unsigned long RETRY_INTERVAL_MS = 5000;
    constexpr unsigned long SILENCE_TIMEOUT_MS = 5000;

    // 2. Hardware auto-detection state machine
    // If locked, monitor for total link silence (> 5.0s).
    // Note: Healthy GPS modules send NMEA sentences continuously every second (even with 0 satellites).
    // Total serial silence only occurs if wires are swapped, disconnected, or powered down.
    if (hwState == GPS_HW_LOCKED_NORMAL || hwState == GPS_HW_LOCKED_SWAPPED) {
        if (now - lastValidNmeaTime >= SILENCE_TIMEOUT_MS) {
            Serial.println(F("[GPS ALERT] Total loss of NMEA data (>5s). Initiating dynamic pin probe..."));
            // Test the alternate pin configuration immediately
            if (hwState == GPS_HW_LOCKED_NORMAL) {
                gpsSerial.end();
                delay(10);
                gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_TX_PIN, GPS_RX_PIN);
                hwState = GPS_HW_PROBING_SWAPPED;
            } else {
                gpsSerial.end();
                delay(10);
                gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
                hwState = GPS_HW_PROBING_DEFAULT;
            }
            stateTimer = now;
            nmeaIdx = 0;
        }
        return;
    }

    if (hwState == GPS_HW_PROBING_DEFAULT) {
        if (now - stateTimer >= PROBE_TIMEOUT_MS) {
            Serial.printf("[GPS AUTO-DETECT] No NMEA data on default pins (RX=GPIO%d, TX=GPIO%d). Testing swapped RX/TX (RX=GPIO%d, TX=GPIO%d)...\n",
                          GPS_RX_PIN, GPS_TX_PIN, GPS_TX_PIN, GPS_RX_PIN);
            gpsSerial.end();
            delay(10);
            gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_TX_PIN, GPS_RX_PIN);
            hwState = GPS_HW_PROBING_SWAPPED;
            stateTimer = now;
            nmeaIdx = 0;
        }
    } else if (hwState == GPS_HW_PROBING_SWAPPED) {
        if (now - stateTimer >= PROBE_TIMEOUT_MS) {
            Serial.printf("[GPS ALERT] GPS module not responding on either pin configuration. Resetting to default pins (RX=GPIO%d, TX=GPIO%d). Check VCC/GND and wiring.\n",
                          GPS_RX_PIN, GPS_TX_PIN);
            gpsSerial.end();
            delay(10);
            gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
            hwState = GPS_HW_DISCONNECTED;
            stateTimer = now;
            nmeaIdx = 0;
        }
    } else if (hwState == GPS_HW_DISCONNECTED) {
        if (now - stateTimer >= RETRY_INTERVAL_MS) {
            Serial.println(F("[GPS AUTO-DETECT] Initiating new probe attempt on default pins..."));
            hwState = GPS_HW_PROBING_DEFAULT;
            stateTimer = now;
            nmeaIdx = 0;
        }
    }
}

void getGPSData(int32_t &latE7, int32_t &lonE7, int16_t &altX10, uint8_t &sats, uint8_t &fixType) {
    portENTER_CRITICAL(&gpsMux);
    latE7 = gpsLatE7;
    lonE7 = gpsLonE7;
    altX10 = gpsAltX10;
    sats = gpsSatellites;
    fixType = gpsFixType;
    portEXIT_CRITICAL(&gpsMux);
}

void getGPSDataDec(double &lat, double &lon, float &alt, int &sats, int &fixType) {
    portENTER_CRITICAL(&gpsMux);
    lat = gpsLatitude;
    lon = gpsLongitude;
    alt = gpsAltitude;
    sats = (int)gpsSatellites;
    fixType = (int)gpsFixType;
    portEXIT_CRITICAL(&gpsMux);
}

bool isGPSHardwareConnected() {
    return (hwState == GPS_HW_LOCKED_NORMAL || hwState == GPS_HW_LOCKED_SWAPPED) &&
           (millis() - lastValidNmeaTime < 3500);
}

bool isGPSPinsSwapped() {
    return (hwState == GPS_HW_LOCKED_SWAPPED);
}

GPSHardwareState getGPSHardwareState() {
    return hwState;
}

const char* getGPSHardwareStatusStr() {
    switch (hwState) {
        case GPS_HW_LOCKED_NORMAL:
            return "OK (Normal: RX16/TX17)";
        case GPS_HW_LOCKED_SWAPPED:
            return "OK (Auto-Swapped: RX17/TX16)";
        case GPS_HW_PROBING_DEFAULT:
            return "Probing Default (RX16/TX17)";
        case GPS_HW_PROBING_SWAPPED:
            return "Probing Swapped (RX17/TX16)";
        case GPS_HW_DISCONNECTED:
        default:
            return "Disconnected / No Signal";
    }
}
