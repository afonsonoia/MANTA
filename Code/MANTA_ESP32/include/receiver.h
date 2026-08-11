#ifndef RECEIVER_H
#define RECEIVER_H

#include <Arduino.h>

enum RCFilterType {
    RC_FILTER_NONE = 0, // Raw / Spike Rejection only
    RC_FILTER_SMA  = 1, // Simple Moving Average
    RC_FILTER_EMA  = 2, // Exponential Moving Average
    RC_FILTER_WMA  = 3  // Weighted Moving Average
};

void initReceiver();
void getReceiverChannels(uint16_t &ch1, uint16_t &ch2, uint16_t &ch3, uint16_t &ch4, uint16_t &ch5);
void setRCMarginDeadband(uint8_t deadbandUs);
uint8_t getRCMarginDeadband();
bool isRCSignalLost();
bool isRCQuietPeriod();
uint16_t getNoiseFloorUs(); // CH5-estimated noise floor (us)

void setRCFilterConfig(uint8_t filterType, uint16_t windowSize, float alpha);
void getRCFilterConfig(uint8_t &filterType, uint16_t &windowSize, float &alpha);

#endif // RECEIVER_H
