#ifndef SENSOR_INTERFACE_H
#define SENSOR_INTERFACE_H

#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>

class SensorInterface {
  public:
    SensorInterface();
    void begin();
    void readBNO055(sensors_event_t *event);
    void readBNO055Quat(float *quat);
    float readBMP280Temperature();
    float readBMP280Pressure();

  private:
    Adafruit_BNO055 _bno;
    Adafruit_BMP280 _bmp;
};

#endif
