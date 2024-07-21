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

// --- DETECT LAUNCH --- //

bool detectLaunch () {
    if (accel.y() > 3) {
        Serial.println("Launch detected based on acceleration.");
        return true;
    }
    return false;
}

// -- BURNOUT -- //

/* First Stage: The first stage of a rocket generally 
 * provides the primary thrust necessary for liftoff and 
 * to overcome the earth's gravitational pull. It is usually
 * the most powerful and has a high thrust-to-weight ratio. 
 * The cutoff or "burnout" might be more abrupt, with a 
 * noticeable and rapid decrease in acceleration, hence a 
 * higher threshold like 2.0. This value indicates a clear 
 * drop but still within a range where the engines are
 * pushing significantly.

 * Second Stage: Upper stages are typically optimized 
 * for operation in thinner atmospheres or vacuum conditions 
 * and might have lower thrust engines that burn longer but 
 * with less intensity compared to the first stage. The change 
 * in acceleration at burnout might be less pronounced or more 
 * gradual, justifying a lower threshold like 1.5. This value 
 * might reflect a more subtle decrease in acceleration, 
 * appropriate for the operational characteristics 
 * of these stages.
*/
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

void sdwrite() {
    Serial.println("Attempting to log data to SD card...");

    // Ensure the file exists; create it if it doesn't
    if (!SD.exists("flightlog001.txt")) {
        Serial.println("File flightlog001.txt does not exist. Creating a new file.");
        File dataFile = SD.open("flightlog001.txt", FILE_WRITE);
        if (dataFile) {
            dataFile.println("Temperature,Altitude,Filtered Altitude,Accel Y,Filtered Ay,State"); // Add headers
            dataFile.close();
        } else {
            Serial.println("Error creating flightlog001.txt");
        }
    }

    // Open the file for appending
    File dataFile = SD.open("flightlog001.txt", FILE_WRITE);
    if (dataFile) {
        Serial.println("File opened successfully.");

        float temperature = bmp.readTemperature();
        float altitude = bmp.readAltitude(1013.25);
        float filteredAltitude = ekf.getFilteredAltitude();
        float filteredAy = ekf.Ay_filtered();

        // Debug prints for data being written
        Serial.print("Logging data - Temperature: ");
        Serial.print(temperature, 2);
        Serial.print(", Altitude: ");
        Serial.print(altitude, 2);
        Serial.print(", Filtered Altitude: ");
        Serial.print(filteredAltitude, 2);
        Serial.print(", Accel Y: ");
        Serial.print(accel.y(), 2);
        Serial.print(", Filtered Ay: ");
        Serial.print(filteredAy, 2);
        Serial.print(", State: ");
        Serial.println(stateNames[currentState]);

        // Write the data
        dataFile.print(temperature, 2);
        dataFile.print(",");
        dataFile.print(altitude, 2);
        dataFile.print(",");
        dataFile.print(filteredAltitude, 2);
        dataFile.print(",");
        dataFile.print(accel.y(), 2);
        dataFile.print(",");
        dataFile.print(filteredAy, 2);
        dataFile.print(",");
        dataFile.println(stateNames[currentState]);

        dataFile.close();
        Serial.println("Data logged to external SD card.");
    } else {
        Serial.println("Error opening flightlog001.txt");
    }
}

void teensysdwrite() {
    Serial.println("Attempting to log data to Teensy SD card...");

    // Ensure the file exists; create it if it doesn't
    if (!SD.exists("teensylog001.txt")) {
        Serial.println("File teensylog001.txt does not exist. Creating a new file.");
        File dataFile = SD.open("teensylog001.txt", FILE_WRITE);
        if (dataFile) {
            dataFile.println("Temperature,Altitude,Filtered Altitude,Accel Y,Filtered Ay,State"); // Add headers
            dataFile.close();
        } else {
            Serial.println("Error creating teensylog001.txt");
        }
    }

    // Open the file for appending
    File dataFile = SD.open("teensylog001.txt", FILE_WRITE);
    if (dataFile) {
        Serial.println("Teensy SD file opened successfully.");

        float temperature = bmp.readTemperature();
        float altitude = bmp.readAltitude(1013.25);
        float filteredAltitude = ekf.getFilteredAltitude();
        float filteredAy = ekf.Ay_filtered();

        // Debug prints for data being written
        Serial.print("Logging data - Temperature: ");
        Serial.print(temperature, 2);
        Serial.print(", Altitude: ");
        Serial.print(altitude, 2);
        Serial.print(", Filtered Altitude: ");
        Serial.print(filteredAltitude, 2);
        Serial.print(", Accel Y: ");
        Serial.print(accel.y(), 2);
        Serial.print(", Filtered Ay: ");
        Serial.print(filteredAy, 2);
        Serial.print(", State: ");
        Serial.println(stateNames[currentState]);

        // Write the data
        dataFile.print(temperature, 2);
        dataFile.print(",");
        dataFile.print(altitude, 2);
        dataFile.print(",");
        dataFile.print(filteredAltitude, 2);
        dataFile.print(",");
        dataFile.print(accel.y(), 2);
        dataFile.print(",");
        dataFile.print(filteredAy, 2);
        dataFile.print(",");
        dataFile.println(stateNames[currentState]);

        dataFile.close();
        Serial.println("Data logged to Teensy SD card.");
    } else {
        Serial.println("Error opening teensylog001.txt");
    }
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
void abortSystem () {
    if (abs(euler.y()) > 35 || abs(euler.x()) > 45) {
        Serial.println("Abort detected due to orientation limits.");
        cutoffpower();
    }
}

// -- DETECT LANDING -- //
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

// -- LOW POWER MODE -- //
void lowpowermode (void (*sdwrite)(), void (*transmitData)()) {
    Serial.println("Entering low power mode");

    // Set BNO, GPS to low power mode
    bno.setExtCrystalUse(false); // Example of setting BNO055 to low power mode

    while (true) {
        sdwrite();
        transmitData();
        delay(30000); // Transmit data every 30 seconds
    }
}