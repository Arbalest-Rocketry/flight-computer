#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include <RH_RF95.h>
#include <ArduinoJson.h>
#include <math.h>
#include <iostream>
#include <cmath>
#include <cstring>
#include <TimeLib.h> 
#include "EKF.h" 
#include "apogee.h"
#include "rocket_stages.h" 
#include "songs.h"
#include "runcamsplits.h"
#include "quaternion.h"

// State names corresponding to the RocketState enum
const char* stateNames[] = {
    "PRE_LAUNCH",
    "LAUNCH_DETECTED",
    "FIRST_STAGE_BURNOUT",
    "STAGE_SEPARATION",
    "UPPER_STAGE_IGNITION",
    "APOGEE",
    "MAIN_CHUTE_DEPLOYMENT",
    "LANDING_DETECTED",
    "LOW_POWER_MODE"
};

// Define the pin numbers
const int 
pyrS1droguechute = 20,pyrS1mainchute = 21,pyrS12sep = 22,pyroIgniteS2 = 23,pyrS2droguechute = 24,pyrS2mainchute = 25;

const unsigned long boosterBurpTime = 1000;
bool boosterBurpDetected, boosterBurnoutCheck = false;
#define BNO055_POWER_MODE_LOWPOWER 0x01
#define BNO055_AXIS_MAP_CONFIG_ADDR 0x41
#define BNO055_AXIS_MAP_SIGN_ADDR   0x42

// --- DETECT LAUNCH --- //
bool detectLaunch () {
    if (ekf.Ay_filtered() > 7.0) {
        Serial.println("Launch detected based on acceleration.");
        return true;
    }
    return false;
}

// -- BURNOUT DETECTION -- //

/* The burnout detection logic is carefully designed to figure out when a rocket stage 
   has burned through its fuel and isn’t generating enough thrust anymore, so we can smoothly 
   move on to the next stage.

 * First Stage: The first stage is all about getting the rocket off the ground and pushing 
   it through the thickest part of the atmosphere. It has a lot of power, so when the engines 
   cut off, you’ll see a big drop in acceleration. That’s why we use a higher threshold of 2.0 
   to detect when the first stage has burned out—it's a pretty clear sign that the engines have 
   done their job and it’s time to switch gears.

 * Second Stage: The second stage is designed for the upper atmosphere or space, where it doesn’t 
   need as much power. The engines burn a bit longer but not as intensely, so the drop in 
   acceleration when this stage burns out is more subtle. That’s why we use a lower threshold of 1.5 
   to catch that change and confirm the second stage has done its part.

 * Booster Burp: After we detect burnout, there’s a little safety check we do called a "booster burp" 
   check. This helps us make sure there aren’t any little bursts of thrust left that could mess up 
   the transition to the next stage. We keep an eye on the acceleration for a short period (called 
   boosterBurpTime). If we see a spike above 1.0 during this time, we know the engines aren’t quite 
   done yet, so we reset and wait. This way, we only move on when we’re really sure the burnout is 
   complete.

 * Stage Awareness: The whole system is smart enough to know which stage it’s in, so it adjusts its 
   burnout detection based on whether it’s dealing with the first or second stage. This makes sure 
   we’re using the right criteria for the right part of the flight, so the transitions happen 
   exactly when they’re supposed to.
 */

bool detectBurnout() {
    static int stage = 1;
    static bool boosterBurpDetected = false;
    static unsigned long boosterBurnoutTime = 0;

    // Define the burnout threshold based on the current stage
    float burnoutThreshold = (stage == 1) ? 2.0 : 1.5;
    if (ekf.Ay_filtered() <= burnoutThreshold) {
      //Last-minute mod: y-axis points down from our setup; it's now inverted!
        if (!boosterBurpDetected) {
            Serial.print("Burnout detected at stage ");
            Serial.println(stage);
            stage++;
            boosterBurnoutTime = millis();
            boosterBurpDetected = true;

            return true;
        } else if (millis() - boosterBurnoutTime <= boosterBurpTime) {
            if (ekf.Ay_filtered() > 1.0) {
                Serial.println("Booster burp detected. Resetting stage.");
                boosterBurpDetected = false;
                stage--; 
                return false;  
            }
        } else {
            boosterBurpDetected = false;
            return true;
        }
    }
    return false;
}

// -- AXIS REMAPPING -- //

void writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(0x28);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
    Wire.endTransmission();
}

void axisRemapping() {
    // Set to CONFIG mode to update axis remap
    bno.setMode(OPERATION_MODE_CONFIG);
    delay(25);

    // Use the default remap configuration (no axis remapping)
    writeRegister(BNO055_AXIS_MAP_CONFIG_ADDR, 0x24);  // Default configuration
    delay(10);

    // Invert the X, Y, and Z axes to account for the 180-degree rotation about the X-axis
    writeRegister(BNO055_AXIS_MAP_SIGN_ADDR, 0x03);  // Invert X, Y, and Z axes
    delay(10);

    // Set back to the desired operation mode, e.g., NDOF mode
    bno.setMode(OPERATION_MODE_NDOF);
    delay(20);
}

void sdwrite() {
    static String currentFileName = ""; // Static to retain across function calls
    if (currentFileName == "") {
        currentFileName = generateNewFileName("flightlog");
        Serial.println("New file created: " + currentFileName);
    }
    Serial.println("Attempting to log data to SD card...");
    logData(currentFileName.c_str());
}

void logData(const char* filename) {
    if (ensureFileExists(filename)) {
        File dataFile = SD.open(filename, FILE_WRITE);
        if (dataFile) {
            Serial.println("File opened successfully.");
            float temperature = bmp.readTemperature();
            float altitude = bmp.readAltitude(1013.25);
            float filteredAltitude = ekf.getFilteredAltitude();
            float filteredAy = ekf.Ay_filtered();

            // Write the data
            writeDataToSD(dataFile, temperature, altitude, filteredAltitude, accel.y(), filteredAy, stateNames[currentState]);

            dataFile.close();
            Serial.println("Data logged successfully.");
        } else {
            Serial.println("Error opening file for writing.");
        }
    }
}

bool ensureFileExists(const char* filename) {
    if (!SD.exists(filename)) {
        Serial.print("File ");
        Serial.print(filename);
        Serial.println(" does not exist. Creating a new file.");
        File dataFile = SD.open(filename, FILE_WRITE);
        if (dataFile) {
            dataFile.println("Timestamp,Temp,Alt,FAlt,AccelY,FAccelY,State");
            dataFile.close();
            return true;
        } else {
            Serial.println("Error creating file.");
            return false;
        }
    }
    return true;
}

void writeDataToSD(File &dataFile, float temperature, float altitude, float filteredAltitude, float accelY, float filteredAy, const char* state) {
    // Ensure the dataFile is open before writing
    if (dataFile) {
        unsigned long timestamp = millis();
        dataFile.print(timestamp);
        dataFile.print(",");
        dataFile.print(temperature, 2);
        dataFile.print(",");
        dataFile.print(altitude, 2);
        dataFile.print(",");
        dataFile.print(filteredAltitude, 2);
        dataFile.print(",");
        dataFile.print(accelY, 2);
        dataFile.print(",");
        dataFile.print(filteredAy, 2);
        dataFile.print(",");
        dataFile.println(state);
    } else {
        Serial.println("Error writing data to file.");
    }
}

String generateNewFileName(const char* baseName) {
    String fileName;
    for (uint8_t i = 1; i < 1000; i++) {
        fileName = String(baseName) + String(i) + ".txt";
        if (!SD.exists(fileName.c_str())) {
            break;
        }
    }
    return fileName;
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

// -- TRANSMIT DATA -- //
/*
void transmitData () {
    Serial.println("Transmitting data ...");
    DynamicJsonDocument doc(256);
    
    eulerToQuaternion(euler.x(), euler.y(), euler.z(), &q);
    float temperature = bmp.readTemperature();
    double altitude = bmp.readAltitude(1013.25);
    float pressure = bmp.readPressure();
    float filt_alt = ekf.getFilteredAltitude();

    char tempStr[8], altStr[8], pressStr[8], qwstr[8], qxstr[8], qystr[8], qzstr[8],filtAltStr[8];
    dtostrf(temperature, 5, 2, tempStr);
    dtostrf(altitude, 5, 2, altStr);
    dtostrf(pressure, 5, 2, pressStr);
    dtostrf(filt_alt, 5, 2, filtAltStr);
    dtostrf(q.w, 5, 2, qwstr);
    dtostrf(q.x, 5, 2, qxstr);
    dtostrf(q.y, 5, 2, qystr);
    dtostrf(q.z, 5, 2, qzstr);

    doc["temperature"] = tempStr;
    doc["pressure"] = pressStr;
    doc["altitude"] = altStr;
    doc["qw"] = qwstr;
    doc["qx"] = qxstr;
    doc["qy"] = qystr;
    doc["qz"] = qzstr;

    char jsonBuffer[256];
    serializeJson(doc, jsonBuffer);
    rf95.send((uint8_t *)jsonBuffer, strlen(jsonBuffer));
    rf95.waitPacketSent();
}
*/

void transmitData () {
    Serial.println("Transmitting data ...");
    DynamicJsonDocument doc(256);

    eulerToQuaternion(euler.x(), euler.y(), euler.z(), &q);
    float temperature = bmp.readTemperature();
    double altitude = bmp.readAltitude(1013.25);
    float pressure = bmp.readPressure();
    float filt_alt = ekf.getFilteredAltitude();

    doc["temperature"] = temperature;
    doc["pressure"] = pressure;
    doc["altitude"] = altitude;
    doc["filt_alt"] = filt_alt;
    doc["qw"] = q.w;
    doc["qx"] = q.x;
    doc["qy"] = q.y;
    doc["qz"] = q.z;

    char jsonBuffer[256];
    size_t len = serializeJson(doc, jsonBuffer);

    if (rf95.send((uint8_t *)jsonBuffer, len)) {
        rf95.waitPacketSent();
        Serial.println("Data transmitted successfully.");
    } else {
        Serial.println("Data transmission failed.");
    }
}

// mosfet methods for runcams
void methodOn() {
  Serial.println("ON");
  analogWrite(4, 300); // PIN 4!
  analogWrite(32, 300); // PIN 32! 
}

void methodOff() {
  analogWrite(4, 0);
  analogWrite(32, 0); 
  Serial.println("OFF");
}

/*
Roll (euler.x()): Tilting left/right (like a ship rocking sideways).
Pitch (euler.y()): Tilting forward/backward (like nodding).
Yaw (euler.z()): Spinning around the vertical axis (like a spinning top).
*/

void cutoffpower() {
    digitalWrite(teensyled, LOW);
    pinMode(pyrS1droguechute, OUTPUT);
    pinMode(pyrS1mainchute, OUTPUT);
    pinMode(pyrS12sep, OUTPUT);
    pinMode(pyroIgniteS2, OUTPUT);
    pinMode(pyrS2droguechute, OUTPUT);
    pinMode(pyrS2mainchute, OUTPUT);
    delay(100); 
    rf95.sleep();  
    digitalWrite(ledblu, LOW);
    digitalWrite(ledgrn, LOW);
    digitalWrite(ledred, LOW);
}

// -- ABORT! -- //
void tiltLock() {
    const float yTiltLimit = 75.0;
    const float xTiltLimit = 85.0;
    if (abs(euler.y()) > yTiltLimit || abs(euler.x()) > xTiltLimit) {
        Serial.println("Abort detected due to orientation limits.");
        cutoffpower();
    }
}

// -- DETECT LANDING -- //
bool detectLanding(Adafruit_BMP280 &bmp) {
    static double lastAltitude = 0;
    double currentAltitude = bmp.readAltitude(1013.25);
    static unsigned long landedTime = millis();
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

// -- LOW POWER MODE -- //
void lowpowermode (void (*sdwrite)(), void (*transmitData)()) {
    isLowPowerModeEntered = true;
    Serial.println("Entering low power mode");
    bno.setMode(OPERATION_MODE_CONFIG);
    delay(25);
    bno.enterSuspendMode();
    delay(25);
    bno.setMode(OPERATION_MODE_NDOF);
    delay(25);
    bno.setExtCrystalUse(false);
    Serial.println("BNO055 set to low power mode");
    bmp.setSampling(Adafruit_BMP280::MODE_SLEEP);
    Serial.println("BMP280 set to low power mode");
    rf95.sleep();
    Serial.println("LoRa module set to low power mode");
    methodOff();
    Serial.println("RunCams set to low power mode");
    Serial.println("Ensure SD card is not accessed to save power");
    while (true) {
        sdwrite();
        transmitData();
        delay(30000); // Transmit data every 30 seconds
    }
}