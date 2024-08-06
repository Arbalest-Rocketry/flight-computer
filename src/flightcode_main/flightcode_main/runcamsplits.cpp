//========================================================================================================//
//                                                                                                        //
//      This RunCam implementation is designed for serial communication.                                  //
//      Note: This is not used in the final flight code.                                                  //
//                                                                                                        //
//      In the final flight code, we opt for the loop video settings and MOSFET implementation            //
//      to control the RunCam for more reliable and automated operations during flight. ~ Leroy           //
//                                                                                                        //
//========================================================================================================//

#include "runcamsplits.h"

#define BUFF_SIZE 20
#define HWSERIAL Serial1

uint8_t txBuf[BUFF_SIZE], crc;
int recState = 0;


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