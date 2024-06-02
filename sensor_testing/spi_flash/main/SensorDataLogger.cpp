#include "SensorDataLogger.h"

SensorDataLogger::SensorDataLogger(int chipSelectPin) : _chipSelectPin(chipSelectPin) {}

void SensorDataLogger::begin() {
  if (!SD.begin(_chipSelectPin)) {
    Serial.println("SD card initialization failed!");
    return;
  }
}

void SensorDataLogger::logData(float *quat, float bmpTemperature, float bmpPressure) {
  _dataFile = SD.open("sensor_data.txt", FILE_WRITE);
  if (_dataFile) {
    _dataFile.print("BNO055 Quaternion: ");
    _dataFile.print(quat[0]);
    _dataFile.print(",");
    _dataFile.print(quat[1]);
    _dataFile.print(",");
    _dataFile.print(quat[2]);
    _dataFile.print(",");
    _dataFile.print(quat[3]);
    _dataFile.print(" BMP280 Temperature: ");
    _dataFile.print(bmpTemperature);
    _dataFile.print(" °C, Pressure: ");
    _dataFile.print(bmpPressure);
    _dataFile.println(" hPa");
    _dataFile.close();
    Serial.println("Data logged successfully.");
  } else {
    Serial.println("Error opening data file.");
  }
}

void SensorDataLogger::readAndPrintData() {
  _dataFile = SD.open("sensor_data.txt", FILE_READ);
  if (_dataFile) {
    Serial.println("Contents of sensor_data.txt:");
    while (_dataFile.available()) {
      Serial.write(_dataFile.read());
    }
    _dataFile.close();
  } else {
    Serial.println("Error opening data file for reading.");
  }
}
