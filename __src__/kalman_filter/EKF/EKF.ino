#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
#include "EKF.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55);
EKF ekf;

void setup() {
    Serial.begin(115200);
    if (!bno.begin()) {
        Serial.print("No BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }
    delay(1000);
    bno.setExtCrystalUse(true);
}

void loop() {
    sensors_event_t event;
    bno.getEvent(&event);

    // Get sensor data
    Vector3d accel(event.acceleration.x, event.acceleration.y, event.acceleration.z);
    Vector3d mag(event.magnetic.x, event.magnetic.y, event.magnetic.z);
    Vector3d gyro(event.gyro.x, event.gyro.y, event.gyro.z);

    double dt = 0.01; // Time step, adjust as needed

    ekf.predict(gyro, dt);
    ekf.update(accel, mag);

    Eigen::Vector3d eulerAngles = ekf.getEulerAngles(ekf.getXHat().head<4>());

    Serial.print("Roll: ");
    Serial.print(eulerAngles(0) * 180 / M_PI);
    Serial.print(", Pitch: ");
    Serial.print(eulerAngles(1) * 180 / M_PI);
    Serial.print(", Yaw: ");
    Serial.println(eulerAngles(2) * 180 / M_PI);

    delay(10); // Delay to match your time step
}