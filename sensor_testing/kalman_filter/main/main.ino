#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include "kalman_filter.h"

Adafruit_BNO055 bno = Adafruit_BNO055();
Adafruit_BMP280 bmp;

Kalman kalman(&bmp, &bno);

void setup() {
  Serial.begin(9600);

  if (!bno.begin()) {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  kalman.begin();
}

void loop() {
  kalman.update();

  double filteredAltitude = kalman.getFilteredAltitude();
  double filteredVelocity = kalman.getFilteredVelocity();
  double rawAltitude = kalman.getRawAltitude();
  double rawAcceleration = kalman.getRawAcceleration();

  Serial.print("Raw Altitude: ");
  Serial.print(rawAltitude);
  Serial.print(", Raw Acceleration: ");
  Serial.print(rawAcceleration);
  Serial.print(", Filtered Altitude: ");
  Serial.print(filteredAltitude);
  Serial.print(", Filtered Velocity: ");
  Serial.println(filteredVelocity);

  delay(1000);
}