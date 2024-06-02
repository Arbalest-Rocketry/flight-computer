#ifndef APOGEE_DETECTOR_H
#define APOGEE_DETECTOR_H

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define WINDOW_SIZE 20
#define DECREASE_THRESHOLD 5 // leroy remember! -> may set it to >=30 cos of irl conditions

class RollingWindow {
public:
    RollingWindow();
    void init(double *pBackingArray, size_t capacity);
    void addDataPoint(double dataPoint);
    double getLatestDataPoint();
    double getEarliestDataPoint();

private:
    double *backingArray;
    size_t capacity;
    size_t size;
    size_t front;
    double sumOfElements;
    size_t modRollingWindow(size_t index, size_t modulo);
};

class ApogeeDetector {
public:
    ApogeeDetector();
    void init(double *backingArray, size_t capacity);
    void update(double currentAltitude);
    bool isApogeeReached();
    double getLastAltitude();

private:
    RollingWindow altitudeWindow;
    double lastAltitude;
    bool apogeeReached;
    int decreaseCount;
};

#endif /* APOGEE_DETECTOR_H */