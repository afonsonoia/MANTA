#include "gps.h"
#include "config.h"
#include <HardwareSerial.h>

static HardwareSerial gpsSerial(2);

static double gpsLatitude = 0.0;
static double gpsLongitude = 0.0;
static float gpsAltitude = 0.0f;
static int gpsSatellites = 0;
static int gpsFixType = 0; // 0 = No Fix, 3 = 3D Fix

static String nmeaBuffer = "";

// GPS pin constants come from config.h (GPIO3/GPIO1 = RX0/TX0)
// GPS cable must be physically disconnected when flashing via USB.

static double convertNMEAToDecimal(String raw, char hemisphere) {
    if (raw.length() == 0) return 0.0;
    double val = raw.toDouble();
    double degrees = floor(val / 100.0);
    double minutes = val - (degrees * 100.0);
    double dec = degrees + (minutes / 60.0);
    if (hemisphere == 'S' || hemisphere == 'W') {
        dec = -dec;
    }
    return dec;
}

static void parseNMEASentence(String sentence) {
    sentence.trim();
    if (!sentence.startsWith("$")) return;

    // Split sentence into comma-separated tokens
    int tokens[20];
    int tokenCount = 0;
    int idx = 0;
    
    while (idx < (int)sentence.length() && tokenCount < 20) {
        int nextIdx = sentence.indexOf(',', idx);
        if (nextIdx == -1) {
            tokens[tokenCount++] = idx;
            break;
        }
        tokens[tokenCount++] = idx;
        idx = nextIdx + 1;
    }

    auto getField = [&](int index) -> String {
        if (index >= tokenCount) return "";
        int start = tokens[index];
        int end = sentence.indexOf(',', start);
        if (end == -1) end = sentence.indexOf('*', start);
        if (end == -1) end = sentence.length();
        return sentence.substring(start, end);
    };

    // 1. Parse GGA Sentences ($GPGGA, $GNGGA)
    if (sentence.startsWith("$GPGGA") || sentence.startsWith("$GNGGA")) {
        String rawLat  = getField(2);
        String latHem  = getField(3);
        String rawLon  = getField(4);
        String lonHem  = getField(5);
        String fixStr  = getField(6);
        String satStr  = getField(7);
        String altStr  = getField(9);

        if (satStr.length() > 0) {
            int sats = satStr.toInt();
            gpsSatellites = sats;
        }

        int fixVal = fixStr.toInt();
        if (fixVal > 0) {
            gpsFixType = (fixVal >= 2) ? 3 : 2; // 3D Fix or 2D Fix
            if (rawLat.length() > 0 && rawLon.length() > 0) {
                gpsLatitude  = convertNMEAToDecimal(rawLat, latHem.length() > 0 ? latHem.charAt(0) : 'N');
                gpsLongitude = convertNMEAToDecimal(rawLon, lonHem.length() > 0 ? lonHem.charAt(0) : 'E');
            }
            if (altStr.length() > 0) gpsAltitude = altStr.toFloat();
        } else {
            if (gpsSatellites < 4) {
                gpsFixType = 0;
                gpsLatitude = 0.0;
                gpsLongitude = 0.0;
            }
        }
    }
    // 2. Parse RMC Sentences ($GPRMC, $GNRMC)
    else if (sentence.startsWith("$GPRMC") || sentence.startsWith("$GNRMC")) {
        String status = getField(2); // 'A' = Valid, 'V' = Receiver Warning
        String rawLat = getField(3);
        String latHem = getField(4);
        String rawLon = getField(5);
        String lonHem = getField(6);

        if (status == "A") {
            if (gpsFixType == 0) gpsFixType = 3;
            if (rawLat.length() > 0 && rawLon.length() > 0) {
                gpsLatitude  = convertNMEAToDecimal(rawLat, latHem.length() > 0 ? latHem.charAt(0) : 'N');
                gpsLongitude = convertNMEAToDecimal(rawLon, lonHem.length() > 0 ? lonHem.charAt(0) : 'E');
            }
        }
    }
}

void initGPS() {
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.printf("[GPS] Initialized on RX=GPIO%d, TX=GPIO%d @ %ld baud (disconnect GPS cable when flashing!)\n", GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);
}

void updateGPS() {
    while (gpsSerial.available() > 0) {
        char c = (char)gpsSerial.read();
        if (c == '\n') {
            parseNMEASentence(nmeaBuffer);
            nmeaBuffer = "";
        } else if (c != '\r') {
            nmeaBuffer += c;
            if (nmeaBuffer.length() > 120) {
                nmeaBuffer = "";
            }
        }
    }
}

void getGPSData(double &lat, double &lon, float &alt, int &sats, int &fixType) {
    lat = gpsLatitude;
    lon = gpsLongitude;
    alt = gpsAltitude;
    sats = gpsSatellites;
    fixType = gpsFixType;
}
