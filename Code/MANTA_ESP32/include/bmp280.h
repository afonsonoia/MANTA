#ifndef BMP280_SENSOR_H
#define BMP280_SENSOR_H

#include <Arduino.h>

void initBMP280();
void sampleBMP280();
void getBaroData(float &altitude, float &pressure, float &temperature);
bool isBMP280Available();
void setBaselinePressure();

#endif // BMP280_SENSOR_H
