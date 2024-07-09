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

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include <SoftwareSerial.h>
#include <RH_RF95.h>
#include "apogee.h"
#include "rocket_stages.h"

// Constants and pin definitions
const int sdCardPin = 10;

#define RFM95_CS 1
#define RFM95_RST 34
#define RFM95_INT 8
#define RF95_FREQ 915.0

double altitude_backing_array[WINDOW_SIZE]; // Array to store altitude data for the rolling window
ApogeeDetector detector; // Apogee detector object
imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
Adafruit_BMP280 bmp;
RH_RF95 rf95(RFM95_CS, RFM95_INT);  // Declare the rf95 object

enum RocketState {
    PRE_LAUNCH,
    LAUNCH_DETECTED,
    FIRST_STAGE_BURNOUT,
    STAGE_SEPARATION,
    UPPER_STAGE_IGNITION,
    APOGEE,
    MAIN_CHUTE_DEPLOYMENT,
    LANDING_DETECTED,
    LOW_POWER_MODE
};

RocketState currentState = PRE_LAUNCH;
unsigned long stateEntryTime = 0;
bool apogeeReached,mainChuteDeployed,isLowPowerModeEntered = false;

void logData() {
    Serial.println("Logging data...");
}

void transmitData() {
    Serial.println("Transmitting data...");
}

void changeState(RocketState newState) {
    currentState = newState;
    stateEntryTime = millis();
}

void setup() {
    // Serial and pin initialization
    Serial.begin(9600);
    while (!Serial) delay(10); 

    pinMode(pyrS1droguechute, OUTPUT);
    pinMode(pyrS1mainchute, OUTPUT);
    pinMode(pyrS12sep, OUTPUT);
    pinMode(pyroIgniteS2, OUTPUT);
    pinMode(pyrS2droguechute, OUTPUT);
    pinMode(pyrS2mainchute, OUTPUT);

    // Initialize sensors
    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP280 sensor, check wiring!");
        while (1);
    } 
    Serial.println("BMP280 initialized!");

    if (!bno.begin()) {
        Serial.println("No BNO055 detected, check wiring!");
        while (1);
    } 
    Serial.println("BNO055 initialized!");

    // Calibrate the BMP280 sensor
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_125); /* Standby time. */

    // Initialize SD card
    if (!SD.begin(sdCardPin)) {
        Serial.println("SD card initialization failed!");
        return;
    }
    Serial.println("SD card initialized!");

    // Initialize LoRa radio
    if (!rf95.init()) {
        Serial.println("LoRa radio init failed");
        while (1);
    } 
    Serial.println("LoRa radio initialized!");
    rf95.setFrequency(RF95_FREQ);
    rf95.setTxPower(23, false);
    init_apogee_detector(&detector, altitude_backing_array, WINDOW_SIZE);
}

void loop() {
    unsigned long currentTime = millis();
    switch (currentState) {
        case PRE_LAUNCH:
            if (detectLaunch()) {
                changeState(LAUNCH_DETECTED);
            }
            break;
        case LAUNCH_DETECTED:
            if (detectBurnout()) {
                deployS1drogue();
                delay(10000);
                deployS1main();
                changeState(FIRST_STAGE_BURNOUT);
            }
            break;
        case FIRST_STAGE_BURNOUT:
            if (currentTime - stateEntryTime > 10000) {
                separatestages();
                changeState(STAGE_SEPARATION);
            }
            break;
        case STAGE_SEPARATION:
            if (currentTime - stateEntryTime > 10000) {
                igniteupperstagemotors();
                changeState(UPPER_STAGE_IGNITION);
            }
            break;
        case UPPER_STAGE_IGNITION:
            if (detectApogee()) {
                deployS2drogue();
                changeState(APOGEE);
            }
            break;
        case APOGEE:
            if (bmp.readAltitude(1013.25) <= 500 && !mainChuteDeployed) {
                deployS2main();
                changeState(MAIN_CHUTE_DEPLOYMENT);
                mainChuteDeployed = true;
            }
            break;
        case MAIN_CHUTE_DEPLOYMENT:
            if (detectLanding(bmp)) {
                changeState(LANDING_DETECTED);
            }
            break;
        case LANDING_DETECTED:
            enterLowPowerMode(logData, transmitData);
            changeState(LOW_POWER_MODE);
            break;
        case LOW_POWER_MODE:
            //TODO
            break;
    }
}