// runcamsplits.h
#ifndef RUNCAMSPLITS_H
#define RUNCAMSPLITS_H

#include <Arduino.h>

void setupRunCam();
void loopRunCam();
void startRecording();
void stopRecording();
uint8_t calcCrc(uint8_t *buf, uint8_t numBytes);
uint8_t crc8_calc(uint8_t crc, unsigned char a, uint8_t poly);

#endif /* RUNCAMSPLITS_H */