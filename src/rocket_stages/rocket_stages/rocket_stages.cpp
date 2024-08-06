///////////////////////////////////////////////////////////////////////////
//
//  Description:  
//      Rocket State Machine for Goose 4
//
//  Comments:
//
//  Leroy Musa
//  Arbalest Rocketry
//  7/09/2024
///////////////////////////////////////////////////////////////////////////  

#include "rocket_stages.h"
#include <Arduino.h>

// Define the pin numbers
const int 
pyrS1droguechute = 20,pyrS1mainchute = 21,pyrS12sep = 22,pyroIgniteS2 = 23,pyrS2droguechute = 24,pyrS2mainchute = 25;

// Define the BNO055 object
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
#define BNO055_POWER_MODE_LOWPOWER 0x01

bool detectLaunch () {
    if (accel.y() > 13) {
        Serial.println("Launch detected based on acceleration.");
        return true;
    }
    return false;
}

void deployPyro(int pin, const char* message) {
    Serial.println(message);
    digitalWrite(pin, HIGH); 
    delay(5000); //just making sure 
    digitalWrite(pin, LOW);
}

void deployS1drogue() {deployPyro(pyrS1droguechute, "Deploying first stage drogue pyros");}
void deployS1main() {deployPyro(pyrS1mainchute, "Deploying first stage main pyros");}
void separatestages() {deployPyro(pyrS12sep, "Separating stages...");}
void igniteupperstagemotors(){deployPyro(pyroIgniteS2, "Igniting upper stage motors ...");}
void deployS2drogue() {deployPyro(pyrS2droguechute, "Deploying second stage drogue pyros");}
void deployS2main() {deployPyro(pyrS2mainchute, "Deploying second stage main pyros");}


bool detectBurnout () {
    static int stage = 1;
    float burnoutThreshold = (stage == 1) ? 2.0 : 1.5; 
    if (accel.y() <= burnoutThreshold) { 
      //y-axis points up from our setup: https://github.com/Arbalest-Rocketry/flight-computer/blob/master/images/electronics_mount_cad_2.png
      //https://github.com/Arbalest-Rocketry/flight-computer/blob/feature/deploy/chutes/images/PCB_front.png
        Serial.print("Burnout detected at stage ");
        Serial.println(stage);
        stage++; 
        return true;
    }
    return false;
}

bool detectApogee() {return is_apogee_reached(&detector);}

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
    isLowPowerModeEntered = true;
    Serial.println("Entering low power mode");

    // Set BNO055 to low power mode
    bno.setMode(OPERATION_MODE_CONFIG);
    delay(25);
    // Use the Adafruit_BNO055 method to set power mode
    bno.enterSuspendMode();  // Assuming this sets the device to a low power state
    delay(25);
    bno.setMode(OPERATION_MODE_NDOF);
    delay(25);
    bno.setExtCrystalUse(false); // Example of setting BNO055 to low power mode
    Serial.println("BNO055 set to low power mode");

    // Set BMP280 to sleep mode
    bmp.setSampling(Adafruit_BMP280::MODE_SLEEP);
    Serial.println("BMP280 set to low power mode");

    // Set LoRa (RFM9x) to sleep mode
    rf95.sleep();
    Serial.println("LoRa module set to low power mode");

    // Ensure no open files on SD card to save power
    Serial.println("Ensure SD card is not accessed to save power");

    while (true) {
        // Call provided functions to transmit and log data
        logData();
        transmitData();
        delay(30000); // Transmit data every 30 seconds
    }
}