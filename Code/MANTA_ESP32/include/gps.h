#ifndef GPS_H
#define GPS_H

#include <Arduino.h>

void initGPS();
void updateGPS();
void getGPSData(double &lat, double &lon, float &alt, int &sats, int &fixType);

#endif // GPS_H
