#include <Arduino.h>

// CRC8 calculation function
uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a) {
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

void sendCommandToRunCam(uint8_t command[], size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum = crc8_dvb_s2(checksum, command[i]);
    }
    command[length] = checksum;
    Serial1.write(command, length + 1);
    
    Serial.print("Sent to RunCam: ");
    for (size_t i = 0; i < length + 1; i++) {
        Serial.print(command[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

void setup() {
    Serial.begin(115200);  // USB serial
    Serial1.begin(115200); // Serial1 for RunCam communication

    while (!Serial) {
        ; // Wait for Serial to be ready
    }

    Serial.println("Teensy Ready");
}

void loop() {
    if (Serial.available() > 0) {
        uint8_t command[10];
        size_t length = Serial.readBytes(command, 10);
        Serial.print("Received from Python: ");
        for (size_t i = 0; i < length; i++) {
            Serial.print(command[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
        sendCommandToRunCam(command, length);
    }

    if (Serial1.available() > 0) {
        Serial.print("Received from RunCam: ");
        while (Serial1.available() > 0) {
            uint8_t byte = Serial1.read();
            Serial.print(byte, HEX);
            Serial.print(" ");
            Serial.write(byte);  // Forward to USB serial
        }
        Serial.println();
    }
}