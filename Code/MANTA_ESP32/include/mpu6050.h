#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>

void initMPU6050();
void sampleMPU6050Uniformly();
void getFilteredMPUData(float &pitch, float &roll);
void getFilteredMPUData(float &pitch, float &roll, float &yaw);
bool isMPU6050Available();

// Expose raw data for detailed diagnostics and PID estimation
void getRawMPUData(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz);

#endif // MPU6050_H
