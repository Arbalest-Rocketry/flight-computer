#ifndef SENSOR_DATA_LOGGER_H
#define SENSOR_DATA_LOGGER_H

#include <SD.h>

class SensorDataLogger {
  public:
    SensorDataLogger(int chipSelectPin);
    void begin();
    void logData(float *quat, float bmpTemperature, float bmpPressure);
    void readAndPrintData();

  private:
    int _chipSelectPin;
    File _dataFile;
};

#endif
