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

// --- DETECT LAUNCH --- //

bool detectLaunch () {
    if (accel.y() > 13) {
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

//for adafruit sd
void sdwrite () {
    File dataFile = SD.open("flightlog001.txt", FILE_WRITE);
    if (dataFile) {
        float temperature = bmp.readTemperature();
        float altitude = bmp.readAltitude(1013.25);
        float filteredaltitude = ekf.getFilteredAltitude();
        float filteredAy = ekf.Ay_filtered();
        dataFile.print("Temperature,");
        dataFile.print(temperature);
        dataFile.print(",Raw BMP Altitude,");
        dataFile.print(altitude);
        dataFile.print(",EKF BMP Altitude,");
        dataFile.print(filteredaltitude);
        dataFile.print(",Accel Y,");
        dataFile.print(accel.y());
        dataFile.print(",EKF Accel Y,");
        dataFile.print(filteredAy);        
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
void deployChute(){};//TODO
void lightUpperStageMotor(){};//TODO

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
    digitalWrite(pyro1, LOW);
    digitalWrite(pyro2, LOW);
    digitalWrite(pyro_drogue, LOW);
    digitalWrite(pyro_main, LOW);
    teensysdwrite("System shutdown initiated.");
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
bool detectLanding (Adafruit_BMP280& bmp) {
    static float lastAltitude = bmp.readAltitude(1013.25);
    float currentAltitude = bmp.readAltitude(1013.25);

    if (fabs(currentAltitude - lastAltitude) < 0.1) {  // Very small change indicates landing
        Serial.println("Landing detected based on altitude stability.");
        return true;
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