#include "rocket_stages.h"
#include <Arduino.h>

// Define the pin numbers
const int pyro1Pin = 20;
const int pyro2Pin = 21;
const int pyroDroguePin = 22;
const int pyroMainPin = 23;
bool isLowPowerModeEntered = false;

// Define the BNO055 object
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

bool detectLaunch(Adafruit_BMP280 &bmp) {
    static double groundAltitude = bmp.readAltitude(1013.25);
    double currentAltitude = bmp.readAltitude(1013.25);
    double altitudeDifference = currentAltitude - groundAltitude;

    // Assuming launch is detected if the altitude difference is greater than 5 meters
    if (altitudeDifference > 1.0) { // for test at home ( I can't lift the breadboard more than 30cm :D )
        Serial.println("Launch detected");
        return true;
    }
    return false;
}

void deployFirstStagePyros() {
    Serial.println("Deploying first stage pyros");
    digitalWrite(pyro1Pin, HIGH);
    delay(1000); // Ensure the pyro is activated
    digitalWrite(pyro1Pin, LOW);
}

bool detectFirstStageBurnout(Adafruit_BMP280 &bmp) {
    static double lastAltitude = 0;
    static unsigned long lastTime = millis();
    unsigned long currentTime = millis();
    double currentAltitude = bmp.readAltitude(1013.25);

    // Check if altitude remains constant (±0.1 meter) for more than 2 seconds
    if (abs(currentAltitude - lastAltitude) < 0.1) {
        if (currentTime - lastTime > 2000) {
            Serial.println("First stage burnout detected");
            return true;
        }
    } else {
        lastTime = currentTime;
    }
    lastAltitude = currentAltitude;
    return false;
}

void separateStages() {
    Serial.println("Separating stages");
    digitalWrite(pyro1Pin, HIGH);
    delay(1000); // Ensure the separation pyro is activated
    digitalWrite(pyro1Pin, LOW);
}

void lightUpperStageMotor() {
    Serial.println("Lighting upper stage motor");
    digitalWrite(pyro2Pin, HIGH);
    delay(1000); // Ensure the upper stage motor is ignited
    digitalWrite(pyro2Pin, LOW);
}

void deploySecondStageDroguePyros(ApogeeDetector &detector, Adafruit_BMP280 &bmp, bool &apogeeReached) {
    if (is_apogee_reached(&detector) && !apogeeReached) {
        Serial.println("Apogee Reached!");
        Serial.println("Deploying second stage pyros (drogue chute)");
        delay(30000);
        digitalWrite(pyroDroguePin, HIGH);
        delay(1000); // Ensure the pyro is activated
        digitalWrite(pyroDroguePin, LOW);
        apogeeReached = true;
    } else {
        Serial.println("Apogee not reached yet.");
    }
}

void deployMainParachutePyros(bool &apogeeReached, bool &mainChuteDeployed, Adafruit_BMP280 &bmp) {
    if (apogeeReached && !mainChuteDeployed) {
        double currentAltitude = bmp.readAltitude(1013.25);
        if (currentAltitude <= 500) { // Assuming altitude is in meters
            Serial.println("500 meters above ground level on descent reached");
            Serial.println("Deploying main parachute pyros");
            digitalWrite(pyroMainPin, HIGH);
            delay(1000);
            digitalWrite(pyroMainPin, LOW);
            mainChuteDeployed = true;
        }
    }
}

bool detectLanding(Adafruit_BMP280 &bmp) {
    static double lastAltitude = 0;
    double currentAltitude = bmp.readAltitude(1013.25);
    static unsigned long landedTime = millis();

    // Check if altitude remains constant (±0.1 meter) for more than 5 seconds
    if (abs(currentAltitude - lastAltitude) < 0.1) {
        if (millis() - landedTime > 5000) {
            Serial.println("Landing detected");
            return true;
        }
    } else {
        landedTime = millis();
    }
    lastAltitude = currentAltitude;
    return false;
}

void enterLowPowerMode(void (*logData)(), void (*transmitData)()) {
    Serial.println("Entering low power mode");

    // Set BNO, GPS to low power mode
    bno.setExtCrystalUse(false); // Example of setting BNO055 to low power mode

    while (true) {
        // Call provided functions to transmit and log data
        logData();
        transmitData();
        delay(30000); // Transmit data every 30 seconds
    }
}