/*
#include "gps_class.h"

Gps gpsModule;

void setup() {
  Serial.begin(9600);
  gpsModule.begin_gps();
}

void loop() {
  gpsModule.read_position();
  delay(1000); // Read position every second
}
*/
#include <Wire.h>

void setup() {
  Wire.begin();
  Wire.setClock(100000); // Set I2C clock speed to 100kHz
  Serial.begin(9600);
  while (!Serial); // Wait for Serial Monitor to open
  Serial.println("Starting I2C communication test...");

  Wire.beginTransmission(0x42); // Attempt to communicate with GPS module at address 0x42
  byte error = Wire.endTransmission();
  if (error == 0) {
    Serial.println("GPS module detected at address 0x42");
  } else {
    Serial.print("Error detected: ");
    Serial.println(error);
  }
}

void loop() {
  // Nothing to do in the loop
}