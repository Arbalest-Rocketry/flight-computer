/* Arbalest Rocketry
    Version 1.00
    July, 7th 2024 
    Author: Leroy Musa  */

// State Machine
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

// Libraries
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include <RH_RF95.h>
#include <ArduinoJson.h>
#include <TimeLib.h> 
#include "EKF.h" 
#include "apogee.h"
#include "rocket_stages.h" 
#include "songs.h"
#include "runcamsplits.h"
#include "quaternion.h"

EKF ekf; //Kalman filter object
double altitude_backing_array[WINDOW_SIZE]; //Array to store altitude data for the rolling window
ApogeeDetector detector; //Apogee detector object
RocketState currentState = PRE_LAUNCH;
unsigned long stateEntryTime = 0;
bool apogeeReached,mainChuteDeployed,isLowPowerModeEntered = false;

//LEDs & pyros
const int ledblu = 7, ledgrn = 4, ledred = 0, teensyled = 13;

//SD CARD(S) CS
const int chipSelect = BUILTIN_SDCARD;
const int sdCardPin = 10;
unsigned long launchTime = 0;
Quaternion q;
imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
Adafruit_BNO055 bno;
Adafruit_BMP280 bmp;

//LoRa settings
#define RFM95_CS 1
#define RFM95_RST 34
#define RFM95_INT 8
#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);  // Declare the rf95 object

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
    pinMode(pyrS1droguechute, OUTPUT);
    pinMode(pyrS1mainchute, OUTPUT);
    pinMode(pyrS12sep, OUTPUT);
    pinMode(pyroIgniteS2, OUTPUT);
    pinMode(pyrS2droguechute, OUTPUT);
    pinMode(pyrS2mainchute, OUTPUT);
    pinMode(buzzer, OUTPUT);

    adafruitSD ();
    teensySD ();
    startup ();
}

void loop() {
    Quaternion q;
    eulerToQuaternion(euler.x(), euler.y(), euler.z(), &q);
    normalizeQuaternion(&q);
    double current_altitude = bmp.readAltitude(1013.25);
    double current_accelY = accel.y(); 
    ekf.update(current_altitude, current_accelY);  
    update_apogee_detector(&detector, current_altitude);

    String msg="";

//============================================================//    
//=========         FSM (FINITE STATE MACHINE)       =========//
//============================================================//
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
            lowpowermode(sdwrite, transmitData);
            changeState(LOW_POWER_MODE);
            break;

        case LOW_POWER_MODE:
            //TODO
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

    void changeState(RocketState newState) {
    currentState = newState;
    stateEntryTime = millis();
  }

void startup() {
    /////////////////////////////////////////////////////////////////////////////
    digitalWrite(ledblu, HIGH);
    delay(500);

    // Initialize BMP280 Pressure Sensor
    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP280 sensor, check wiring!");
        while (1) {
            digitalWrite(ledred, HIGH);
            delay(500);
            digitalWrite(ledred, LOW);
            delay(500);
            // Note<> Flashing red LED indicates sensor initialization failure
        }
    }
    Serial.println("BMP280 initialized!");
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_125); /* Standby time. */

    // Initialize BNO055 Orientation Sensor
    if (!bno.begin()) {
        Serial.println("No BNO055 detected, check wiring!");
        while (1) {
            digitalWrite(ledred, HIGH);
            delay(300);
            digitalWrite(ledred, LOW);
            delay(300);
            // Flashing red LED at a different rate indicates another sensor initialization failure
        }
    }
    Serial.println("BNO055 initialized!");

    digitalWrite(ledblu, LOW);
    digitalWrite(ledgrn, HIGH);

    // Initialize LoRa Radio
    if (!rf95.init()) {
        Serial.println("LoRa radio init failed");
        while (1) {
            digitalWrite(ledred, HIGH);
            delay(100);
            digitalWrite(ledred, LOW);
            delay(100);
            // Faster flashing red LED indicates communication module failure
        }
    }
    Serial.println("LoRa radio initialized!");
    rf95.setFrequency(RF95_FREQ);
    rf95.setTxPower(23, false);
    rf95.setSpreadingFactor(10); // public: void setSpreadingFactor(uint8_t sf)
    rf95.setSignalBandwidth(62.5E3); // public: void setSignalBandwidth(long sbw)
    rf95.setCodingRate4(8); // public: void setCodingRate4(uint8_t denominator)
    digitalWrite(ledgrn, LOW);
    digitalWrite(ledred, HIGH);
    init_apogee_detector(&detector, altitude_backing_array, WINDOW_SIZE);
    ekf.begin(bmp.readAltitude(1013.25), 0);
    digitalWrite(ledred, LOW);
    ////////////////////////////////////////////////////////////////////////////////////
}