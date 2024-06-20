#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include "EKF.h"

// Create sensor objects
Adafruit_BMP280 bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Create EKF filter object
EKF ekf;

// Variables to hold sensor data
double barometerAltitude;
double accelZ = 0.0;
double rawVelocity = 0.0;

unsigned long previousMillis = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Initialize BMP280
    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP280 sensor, check wiring!");
        while (1);
    }

    // Initialize BNO055
    if (!bno.begin()) {
        Serial.println("Could not find a valid BNO055 sensor, check wiring!");
        while (1);
    }
    bno.setExtCrystalUse(true);

    // Calibrate the BNO055 sensor
    calibrateSensor();

    // Print labels for Serial Plotter
    Serial.println("Time (s)\tRaw Altitude (m)\tFiltered Altitude (m)\tRaw AccelZ (m/s^2)\tFiltered AccelZ (m/s^2)\tRaw Velocity (m/s)\tFiltered Velocity (m/s)");

    // Initialize EKF filter
    barometerAltitude = bmp.readAltitude(1013.25); // Assuming sea level pressure is 1013.25 hPa
    ekf.begin(barometerAltitude, accelZ); // Initialize EKF with initial altitude and acceleration
}

void loop() {
    unsigned long currentMillis = millis();
    double dt = (currentMillis - previousMillis) / 1000.0;
    previousMillis = currentMillis;

    // Read barometer altitude
    barometerAltitude = bmp.readAltitude(1013.25);

    // Read raw acceleration from BNO055
    sensors_event_t event;
    bno.getEvent(&event, Adafruit_BNO055::VECTOR_LINEARACCEL);
    accelZ = event.acceleration.z;

    // Calculate raw velocity by integrating raw acceleration
    rawVelocity += accelZ * dt;

    // Update EKF filter
    ekf.update(barometerAltitude, accelZ);

    // Print data for Arduino Serial Plotter
    Serial.print(barometerAltitude); // Print raw altitude
    Serial.print("\t");
    Serial.print(ekf.current_state[0]); // Print filtered altitude
    Serial.print("\t");
    Serial.print(accelZ); // Print raw acceleration
    Serial.print("\t");
    Serial.print(ekf.current_state[2]); // Print filtered acceleration
    Serial.print("\t");
    Serial.print(rawVelocity); // Print raw velocity
    Serial.print("\t");
    Serial.println(ekf.current_state[1]); // Print filtered velocity

    delay(1000); // Delay for readability
}

void calibrateSensor() {
    // Wait until the BNO055 sensor is calibrated
    uint8_t system, gyro, accel, mag = 0;
    Serial.println("Calibrating BNO055...");
    while (system != 3 || gyro != 3 || accel != 3 || mag != 3) {
        bno.getCalibration(&system, &gyro, &accel, &mag);
        Serial.print("Calibration: ");
        Serial.print("Sys=");
        Serial.print(system, DEC);
        Serial.print(" Gyro=");
        Serial.print(gyro, DEC);
        Serial.print(" Accel=");
        Serial.print(accel, DEC);
        Serial.print(" Mag=");
        Serial.println(mag, DEC);
        delay(500);

        if (mag != 3) {
            Serial.println("Move the sensor in a figure-eight pattern to calibrate the magnetometer.");
        }
        if (accel != 3) {
            Serial.println("Place the sensor in six different positions to calibrate the accelerometer: +X, -X, +Y, -Y, +Z, -Z.");
        }
        if (gyro != 3) {
            Serial.println("Keep the sensor stationary to calibrate the gyroscope.");
        }
    }
    Serial.println("BNO055 Calibration Complete.");
}