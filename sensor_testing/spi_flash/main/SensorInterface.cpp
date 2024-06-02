#include "SensorInterface.h"

SensorInterface::SensorInterface() {}

void SensorInterface::begin() {
  if (!_bno.begin()) {
    Serial.println("Failed to initialize BNO055 sensor!");
    while (1);
  }

  if (!_bmp.begin()) {
    Serial.println("Failed to initialize BMP280 sensor!");
    while (1);
  }
}

void SensorInterface::readBNO055(sensors_event_t *event) {
  _bno.getEvent(event);
}

void SensorInterface::readBNO055Quat(float *quat) {
  imu::Quaternion quat_orientation = _bno.getQuat();
  quat[0] = quat_orientation.w();
  quat[1] = quat_orientation.x();
  quat[2] = quat_orientation.y();
  quat[3] = quat_orientation.z();
}

float SensorInterface::readBMP280Temperature() {
  return _bmp.readTemperature();
}

float SensorInterface::readBMP280Pressure() {
  return _bmp.readPressure() / 100.0;
}
