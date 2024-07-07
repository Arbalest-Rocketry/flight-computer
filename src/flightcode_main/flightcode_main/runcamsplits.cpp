#include "runcamsplits.h"

//#define BUFF_SIZE 20
//#define HWSERIAL Serial1

//uint8_t txBuf[BUFF_SIZE], crc;
//int recState = 0;

byte calculateCRC(byte *data, byte len) {
    byte crc = 0;
    for (byte i = 0; i < len; i++) {
        crc ^= data[i];
        for (byte j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0xD5;  // Polynomial for DVB-S2 CRC-8
            else
                crc <<= 1;
        }
    }
    return crc;
}

void startRec() {
    byte startCommand[] = {0xCC, 0x03};
    byte startCRC = calculateCRC(startCommand, sizeof(startCommand));
    Serial1.write(startCommand, sizeof(startCommand));
    Serial1.write(startCRC);
    digitalWrite(13, HIGH);
    Serial.println("Recording started.");
}

void stopRec() {
    byte stopCommand[] = {0xCC, 0x04};
    byte stopCRC = calculateCRC(stopCommand, sizeof(stopCommand));
    Serial1.write(stopCommand, sizeof(stopCommand));
    Serial1.write(stopCRC);
    digitalWrite(13, LOW);
    Serial.println("Recording stopped.");
}

void turnOffCam() {
    byte turnOffCommand[] = {0xCC, 0x01};
    byte turnOffCRC = calculateCRC(turnOffCommand, sizeof(turnOffCommand));
    Serial1.write(turnOffCommand, sizeof(turnOffCommand));
    Serial1.write(turnOffCRC);
    digitalWrite(13, HIGH);
    Serial.println("Camera turned off.");
}

// secondary logic
/*
void setupRunCam() {
    Serial.begin(9600);
    HWSERIAL.begin(115200);
    delay(3000);

    txBuf[0] = 0xCC;
    txBuf[1] = 0x01;
    txBuf[2] = 0x01;
    txBuf[3] = calcCrc(txBuf, 3);
    Serial.println(txBuf[3]);

    delay(10000);
    startRecording();
}

void loopRunCam() {
    // Loop logic if necessary
}

void startRecording() {
    if (recState == 0) {
        Serial.println("Starting Recording");
        recState = 1;
        HWSERIAL.write(txBuf, 4);
    }
}

void stopRecording() {
    if (recState == 1) {
        Serial.println("Stopping Recording");
        recState = 0;
        HWSERIAL.write(txBuf, 4);
    }
}

uint8_t calcCrc(uint8_t *buf, uint8_t numBytes) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < numBytes; i++)
        crc = crc8_calc(crc, buf[i], 0xD5);
    return crc;
}

uint8_t crc8_calc(uint8_t crc, unsigned char a, uint8_t poly) {
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80)
            crc = (crc << 1) ^ poly;
        else
            crc <<= 1;
    }
    return crc;
}
*/
