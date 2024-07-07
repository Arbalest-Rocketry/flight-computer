// Libraries
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

int state;

//LoRa settings
#define RFM95_CS 1
#define RFM95_RST 34
#define RFM95_INT 8
#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);  // Declare the rf95 object
Quaternion q;

bool detectLaunch () {
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    if (accel.x() > 13) {
        Serial.println("Launch detected based on acceleration.");
        return true;
    }
    return false;
}

// --- BURNOUT --- //

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
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    float burnoutThreshold = (stage == 1) ? 2.0 : 1.5; 
    if (accel.x() <= burnoutThreshold) {
        Serial.print("Burnout detected at stage ");
        Serial.println(stage);
        stage++; 
        return true;
    }
    return false;
}

//for adafruit sd
void sdwrite () {
    File dataFile = SD.open("flightlog001.txt", FILE_WRITE);
    if (dataFile) {
        float temperature = bmp.readTemperature();
        float altitude = bmp.readAltitude(1013.25);
        imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

        dataFile.print("Temperature,");
        dataFile.print(temperature);
        dataFile.print(",Altitude,");
        dataFile.print(altitude);
        dataFile.print(",Accel X,");
        dataFile.print(accel.x());
        dataFile.print(",System State,");
        dataFile.println(state);

        dataFile.close();
        Serial.println("Data logged.");
    } else {
        Serial.println("Error opening datalog.txt");
    }
}


//for teensy sd
void teensysdwrite (const String& msg) {
    static bool initialized = false;  // To ensure SD.begin() is called only once
    if (!initialized) {
        if (!SD.begin(BUILTIN_SDCARD)) {
            Serial.println("Built-in SD card initialization failed!");
            return;
        }
        initialized = true;
    }

    File logFile = SD.open("flightLog.txt", FILE_WRITE);
    if (logFile) {
        String timeStamp = String(hour()) + ":" + zeropad(minute()) + ":" + zeropad(second());
        String logEntry = timeStamp + " | " + String(millis()) + " ms | " + msg;

        logFile.println(logEntry);
        logFile.close();
        Serial.println("Logged to built-in SD card: " + logEntry);
    } else {
        Serial.println("Failed to open log file on built-in SD card.");
    }
}

String zeropad(int num) { return (num < 10 ? "0" : "") + String(num); }

void transmitData () {
    Serial.println("Transmitting data ...");
    DynamicJsonDocument doc(256);
    sensors_event_t event;
    bno.getEvent(&event);

    float yaw = event.orientation.x;
    float pitch = event.orientation.y;
    float roll = event.orientation.z;
    float qr, qi, qj, qk;
    eulerToQuaternion(event.orientation.x, event.orientation.y, event.orientation.z, &q);
    float temperature = bmp.readTemperature();
    double altitude = bmp.readAltitude(1013.25);
    float pressure = bmp.readPressure();

    char tempStr[8], altStr[8], pressStr[8], qrStr[8], qiStr[8], qjStr[8], qkStr[8];
    dtostrf(temperature, 5, 2, tempStr);
    dtostrf(altitude, 5, 2, altStr);
    dtostrf(pressure, 5, 2, pressStr);
    dtostrf(qr, 5, 2, qrStr);
    dtostrf(qi, 5, 2, qiStr);
    dtostrf(qj, 5, 2, qjStr);
    dtostrf(qk, 5, 2, qkStr);

    doc["temperature"] = tempStr;
    doc["pressure"] = pressStr;
    doc["altitude"] = altStr;
    doc["qr"] = qrStr;
    doc["qi"] = qiStr;
    doc["qj"] = qjStr;
    doc["qk"] = qkStr;

    char jsonBuffer[256];
    serializeJson(doc, jsonBuffer);
    rf95.send((uint8_t *)jsonBuffer, strlen(jsonBuffer));
    rf95.waitPacketSent();
}

/*
void abortSystem () {
    imu::Quaternion quat = bno.getQuat();  // Get quaternion for current orientation
    double roll, pitch, yaw;
    bno.getOrientation(Adafruit_BNO055::VECTOR_EULER, &roll, &pitch, &yaw);

    if (abs(pitch) > 25 || abs(roll) > 25) {
        Serial.println("Abort detected due to orientation limits.");
        state = 5;  // Transition to an abort state
        // Handle abort logic (e.g., activate recovery systems)
    }
}
*/

bool detectLanding (const Adafruit_BMP280& bmp) {
    static float lastAltitude = bmp.readAltitude(1013.25);
    float currentAltitude = bmp.readAltitude(1013.25);

    if (fabs(currentAltitude - lastAltitude) < 0.1) {  // Very small change indicates landing
        Serial.println("Landing detected based on altitude stability.");
        return true;
    }
    lastAltitude = currentAltitude;
    return false;
}

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
