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
    rf95.setSpreadingFactor(10); // public: void setSpreadingFactor(uint8_t sf)
    rf95.setSignalBandwidth(62.5E3); // public: void setSignalBandwidth(long sbw)
    rf95.setCodingRate4(8); // public: void setCodingRate4(uint8_t denominator)
    init_apogee_detector(&detector, altitude_backing_array, WINDOW_SIZE);
}

/********************************/
/*                              */
/*     State Machine            */
/*                              */
/********************************/
void loop() {
    unsigned long currentTime = millis();
    switch (currentState) {
        case PRE_LAUNCH:
            // Continuously check for launch detection
            if (detectLaunch()) {
                changeState(LAUNCH_DETECTED);
            }
            break;

        case LAUNCH_DETECTED:
            // Wait until burnout is detected
            while (!detectBurnout()) {
                // Keep checking until burnout is detected
            }
            // Deploy the first stage drogue chute
            deployS1drogue();
            delay(10000);
            // Check if the altitude is sufficient for main chute deployment
            if (bmp.readAltitude(1013.25) >= 457.2) { // 1500 feet in meters
                deployS1main();
            } else {
                Serial.println("Altitude too low for first stage main chute deployment.");
            }
            // Move to the next state after deploying chutes
            changeState(FIRST_STAGE_BURNOUT);
            break;

        case FIRST_STAGE_BURNOUT:
            // Wait for a set period before separating stages
            while (currentTime - stateEntryTime <= 10000) {
                // Continuously update current time
                currentTime = millis();
            }
            // Separate the rocket stages
            separatestages();
            // Move to the next state
            changeState(STAGE_SEPARATION);
            break;

        case STAGE_SEPARATION:
            // Wait for a set period before igniting upper stage motors
            while (currentTime - stateEntryTime <= 10000) {
                // Continuously update current time
                currentTime = millis();
            }
            // Ignite the upper stage motors
            igniteupperstagemotors();
            // Move to the next state
            changeState(UPPER_STAGE_IGNITION);
            break;

        case UPPER_STAGE_IGNITION:
            // Wait until apogee is detected
            while (!detectApogee()) {
                // Keep checking until apogee is detected
            }
            // Deploy the second stage drogue chute
            deployS2drogue();
            // Move to the next state
            changeState(APOGEE);
            break;

        case APOGEE:
            // Wait until the main chute is deployed
            while (!mainChuteDeployed) {
                double currentAltitude = bmp.readAltitude(1013.25);
                // Check if the rocket is within the deployment window
                if (currentAltitude <= 500) { // 500 meters for the second stage deployment window
                    if (currentAltitude >= 457.2) { // 1500 feet in meters
                        deployS2main();
                        mainChuteDeployed = true;
                        changeState(MAIN_CHUTE_DEPLOYMENT);
                    } else {
                        Serial.println("Altitude too low for second stage main chute deployment.");
                    }
                }
            }
            break;

        case MAIN_CHUTE_DEPLOYMENT:
            // Wait until landing is detected
            while (!detectLanding(bmp)) {
                // Keep checking until landing is detected
            }
            // Move to the next state
            changeState(LANDING_DETECTED);
            break;

        case LANDING_DETECTED:
            // Enter low power mode after landing is detected
            enterLowPowerMode(logData, transmitData);
            changeState(LOW_POWER_MODE);
            break;

        case LOW_POWER_MODE:
            //TODO
            break;
    }
}