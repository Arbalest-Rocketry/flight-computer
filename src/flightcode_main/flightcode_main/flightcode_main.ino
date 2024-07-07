/* Arbalest Rocketry
    Version 1.00
    July, 7th 2024 
    Author: Leroy Musa  */

// State Machine
enum state {

    PRE_FLIGHT,
    LAUNCH_DETECTION,
    FIRST_STAGE_BURNOUT,
    PREPARE_SECOND_STAGE,
    SECOND_STAGE_BURNOUT,
    APOGEE_DETECTION,
    MAIN_CHUTE_DEPLOYMENT,
    LANDING_CONFIRMATION,
    POST_FLIGHT

};

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

//Kalman filter object
EKF ekf;
double altitude_backing_array[WINDOW_SIZE]; // Array to store altitude data for the rolling window
ApogeeDetector detector; // Apogee detector object
//LEDs and Buzzer
const int ledblu = 7, ledgrn = 4, ledred = 0, teensyled = 13;
//SD CARD(S) CS
const int chipSelect = BUILTIN_SDCARD;
const int sdCardPin = 10;
unsigned long launchTime = 0;
Quaternion q;

//LoRa settings
#define RFM95_CS 1
#define RFM95_RST 34
#define RFM95_INT 8
#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);  // Declare the rf95 object

//===================================================//    
//==============         FLAGS        ===============//
//===================================================//
bool mainChuteDeployed = false;
bool launchDetected = false;
bool firstStageBurnoutDetected = false;
bool isLanded = false;
bool melodyPlayed1 = false, melodyPlayed2=false; // Flag to check if the melody has been played

void setup() {
    Serial.begin(9600);
    Wire.begin();
    Serial1.begin(115200); 
    Serial2.begin(115200); 
    Serial3.begin(115200); 
    
    pinMode(ledblu, OUTPUT);
    pinMode(ledgrn, OUTPUT);
    pinMode(ledred, OUTPUT);
    pinMode(teensyled, OUTPUT);    
    pinMode(pyro1, OUTPUT);
    pinMode(pyro2, OUTPUT);
    pinMode(pyro_drogue, OUTPUT);
    pinMode(pyro_main, OUTPUT);
    pinMode(buzzer, OUTPUT);

    adafruitSD ();
    teensySD ();
    startup ();
}

void loop() {
    Quaternion q;
    sensors_event_t event;
    bno.getEvent(&event);
    
    eulerToQuaternion(event.orientation.x, event.orientation.y, event.orientation.z, &q);
    //normalizeQuaternion(&q); // Optional: Normalize to maintain numerical stability
    double current_altitude = bmp.readAltitude(1013.25);
    double current_accelZ = event.acceleration.z; 
    ekf.update(current_altitude, current_accelZ);  
    update_apogee_detector(&detector, current_altitude);

    String msg="";
//===================================================================================//    
//===================         FSM (FINITE STATE MACHINE)     ========================//
//===================================================================================//

// Handle different states
switch (state) {
    case PRE_FLIGHT: 
            if (millis() > 600000) { 
                startRec ();
                state = LAUNCH_DETECTION;
                msg = "Pre-launch checks passed. Ready for launch.";
                teensysdwrite(msg);
            }
            break;
    case LAUNCH_DETECTION: 
            if (detectLaunch ()) {
                launchTime = millis();
                state = FIRST_STAGE_BURNOUT;
                msg = "Launch detected, transitioning to state 2 for burnout detection.";
                teensysdwrite(msg);
            }
            break;
    case FIRST_STAGE_BURNOUT: 
            if (detectBurnout ()) {
                state = PREPARE_SECOND_STAGE;
                msg = "First stage burnout detected, preparing second stage.";
                teensysdwrite(msg);
            }
            break;
    case PREPARE_SECOND_STAGE:
            if (millis() - launchTime > 10000) { 
                lightUpperStageMotor ();
                state = SECOND_STAGE_BURNOUT;
                msg="Second stage motor lit, monitoring for burnout.";
                teensysdwrite(msg);
            }
            break;
    case SECOND_STAGE_BURNOUT: 
            if (detectBurnout ()) {
                state = APOGEE_DETECTION;
                msg = "Second stage burnout detected, transitioning to APOGEE_DETECTION.";
                teensysdwrite(msg);
            }
            break;
    case APOGEE_DETECTION: 
            if (is_apogee_reached (&detector)) {
                deployChute ();
                state = MAIN_CHUTE_DEPLOYMENT;
                msg="Apogee reached, main chute deployed.";
                teensysdwrite(msg);
            }
            break;
    case MAIN_CHUTE_DEPLOYMENT: 
            if (detectLanding (bmp)) {
                state = LANDING_CONFIRMATION;
                msg="Landing detected, transitioning to recovery state.";
                teensysdwrite(msg);
            }
            break;
    case LANDING_CONFIRMATION: 
            lowpowermode(sdwrite, transmitData);
            msg="System is in recovery state.";
            teensysdwrite(msg);
            state = POST_FLIGHT;
            break;
    case POST_FLIGHT:
            break;
    }

    delay(100); // I am delaying to prevent excess polling

    // Log and transmit data
    sdwrite();
    transmitData();
}

    void adafruitSD() {
    if (!SD.begin(sdCardPin)) {
        Serial.println("SD card initialization failed!");
        return;
    }
    Serial.println("SD card initialized!");
    }
    
    void teensySD() {
    if (!SD.begin(chipSelect)) {
        Serial.println("Built-in SD card initialization failed!");
        return;
    }
    }
void startup () {
    digitalWrite(ledblu, HIGH);
    delay(500);
    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP280 sensor, check wiring!");
        while (1);
    } Serial.println("BMP280 initialized!");
    
    if (!bno.begin()) {
        Serial.println("No BNO055 detected, check wiring!");
        while (1);
    } Serial.println("BNO055 initialized!");

    // Calibrate the BMP280 sensor
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_125); /* Standby time. */

    // Initialize LoRa radio
    if (!rf95.init()) {
        Serial.println("LoRa radio init failed");
        while (1);
    } Serial.println("LoRa radio initialized!");

    rf95.setFrequency(RF95_FREQ);
    rf95.setTxPower(23, false);

    // Initialize apogee detector
    init_apogee_detector(&detector, altitude_backing_array, WINDOW_SIZE);
    
    // Initialize EKF
    ekf.begin(bmp.readAltitude(1013.25), 0);  // Initial altitude and acceleration (set to 0)
    digitalWrite(ledblu, LOW);

  if (!melodyPlayed1) {  whatisthatmelody1(); melodyPlayed1 = true;}
 }
