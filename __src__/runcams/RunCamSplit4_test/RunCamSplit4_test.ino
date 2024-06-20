#include <Arduino.h>

// CRC-8 calculation using polynomial 0xD5
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

void setup() {
  Serial1.begin(115200); // Set the baud rate to match the camera's requirement
  Serial.begin(9600);    // Start serial communication with computer for debugging

  pinMode(13, OUTPUT);   // Set pin 13 as output for the LED

  // Command to start recording
  byte startCommand[] = {0xCC, 0x03}; // Command byte for starting recording
  byte startCRC = calculateCRC(startCommand, sizeof(startCommand));
  Serial1.write(startCommand, sizeof(startCommand));
  Serial1.write(startCRC);
  digitalWrite(13, HIGH); // Turn on LED to indicate recording has started
  Serial.println("Recording started.");

  delay(10000); // Record for 10 seconds

  // Command to stop recording
  byte stopCommand[] = {0xCC, 0x04}; // Command byte for stopping recording
  byte stopCRC = calculateCRC(stopCommand, sizeof(stopCommand));
  Serial1.write(stopCommand, sizeof(stopCommand));
  Serial1.write(stopCRC);
  digitalWrite(13, LOW); // Turn off LED after recording stops
  Serial.println("Recording stopped.");
}

void loop() {} // do nothing