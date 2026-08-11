#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>

void initMPU6050();
void sampleMPU6050Uniformly();
void getFilteredMPUData(float &pitch, float &roll, float &yaw);

#endif // MPU6050_H
