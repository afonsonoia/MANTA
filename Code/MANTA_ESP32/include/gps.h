#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include <stdint.h>

enum GPSHardwareState : uint8_t {
    GPS_HW_PROBING_DEFAULT = 0, // Listening on default pins (RX=16, TX=17)
    GPS_HW_PROBING_SWAPPED = 1, // Testing inverted pins (RX=17, TX=16)
    GPS_HW_LOCKED_NORMAL   = 2, // Confirmed valid NMEA on default pins
    GPS_HW_LOCKED_SWAPPED  = 3, // Confirmed valid NMEA on inverted pins (Auto-Swapped)
    GPS_HW_DISCONNECTED    = 4  // No NMEA data on either pin combination (cable unplugged / no power)
};

void initGPS();
void updateGPS();
void getGPSData(int32_t &latE7, int32_t &lonE7, int16_t &altX10, uint8_t &sats, uint8_t &fixType);
void getGPSDataDec(double &lat, double &lon, float &alt, int &sats, int &fixType);

bool isGPSHardwareConnected();
bool isGPSPinsSwapped();
GPSHardwareState getGPSHardwareState();
const char* getGPSHardwareStatusStr();

#endif // GPS_H
