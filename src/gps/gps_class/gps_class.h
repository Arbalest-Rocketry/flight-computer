#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_I2C_GPS_Arduino_Library.h>
#include <TinyGPS++.h>

class Gps
{
private:
  I2CGPS i2c_gps;
  TinyGPSPlus gps;
  struct __attribute__((packed)) STRUCT
  {
    float lat;
    float lng;
    float diagmsg;
    float gpsalt;
  } gpsStruct;

public:
  Gps();
  void init();
  void begin_gps();
  void test_connection();
  void read_position();
  void read_data();
};