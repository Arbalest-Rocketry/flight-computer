#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "EKF.h"

// Create sensor object
Adafruit_BMP280 bmp;

// Create EKF filter object
EKF ekf;

// Variables to hold sensor data
double barometerAltitude;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Initialize BMP280
    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP280 sensor, check wiring!");
        while (1);
    }

    // Print labels for Serial Plotter
    Serial.println("Time (s)\tRaw Altitude (m)\tFiltered Altitude (m)");

    // Initialize EKF filter
    barometerAltitude = bmp.readAltitude(1013.25); // Assuming sea level pressure is 1013.25 hPa
    ekf.begin(barometerAltitude, 0.0); // Initialize EKF with initial altitude
}

void loop() {
    // Read barometer altitude
    barometerAltitude = bmp.readAltitude(1013.25);

    // Update EKF filter
    ekf.update(barometerAltitude, 0.0);

    // Print data for Arduino Serial Plotter
    Serial.print(barometerAltitude);
    Serial.print("\t");
    Serial.println(ekf.current_state[0]); // Print filtered altitude

    delay(1000); // Delay for readability
}