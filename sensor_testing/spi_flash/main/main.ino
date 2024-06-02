#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "SensorInterface.h"
#include "SensorDataLogger.h"

const int chipSelectPin = 10;
SensorInterface sensors;
SensorDataLogger dataLogger(chipSelectPin);

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }

  sensors.begin();
  dataLogger.begin();

  Serial.println("Initialization complete.");
}

void loop() {
  sensors_event_t event;
  sensors.readBNO055(&event);
  float temperature = sensors.readBMP280Temperature();
  float pressure = sensors.readBMP280Pressure();

  float quat[4];
  sensors.readBNO055Quat(quat);

  dataLogger.logData(quat, temperature, pressure);

  delay(1000);

  dataLogger.readAndPrintData();

  delay(5000);
}
