#include "apogee.h"

ApogeeDetector detector;
double altitude_backing_array[WINDOW_SIZE];

void setup() {
    Serial.begin(115200);
    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP280 sensor, check wiring!");
        while (1);
    }
    
    init_apogee_detector(&detector, altitude_backing_array, WINDOW_SIZE);
}

void loop() {
    double current_altitude = bmp.readAltitude(1013.25); // Assuming sea level pressure
    update_apogee_detector(&detector, current_altitude);
    
    if (is_apogee_reached(&detector) && !apogeeReached) {
        apogeeReached = true;
        // Additional logic for apogee event
    }
    delay(100); // Adjust delay as needed
}