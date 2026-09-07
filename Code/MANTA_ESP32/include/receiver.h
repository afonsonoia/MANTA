#ifndef RECEIVER_H
#define RECEIVER_H

#include <Arduino.h>

void initReceiver();
void getReceiverChannels(uint16_t &ch1, uint16_t &ch2, uint16_t &ch3, uint16_t &ch5);
void getReceiverChannels(uint16_t &ch1, uint16_t &ch2, uint16_t &ch3, uint16_t &ch4, uint16_t &ch5);
void setRCMarginDeadband(uint8_t deadbandUs);
uint8_t getRCMarginDeadband();
bool isRCSignalLost();

#endif // RECEIVER_H
