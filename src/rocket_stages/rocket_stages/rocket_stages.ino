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

double altitude_backing_array[WINDOW_SIZE]; // Array to store altitude data for the rolling window
ApogeeDetector detector; // Apogee detector object
const int sdCardPin = 10;

// LoRa settings
#define RFM95_CS 1
#define RFM95_RST 34
#define RFM95_INT 8
#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);  // Declare the rf95 object

// ------------------------- FLAGS --------- //
bool mainChuteDeployed = false;
bool launchDetected = false;
bool firstStageBurnoutDetected = false;
bool isLanded = false;
// ----------------------------------------- //

// I am putting dummy text beacuse these methods work in isolation
void logData() {
    Serial.println("Heyyy, loook at meeeee! I am logging data!!");
}

void transmitData() {
    Serial.println("Heyyy, loook at meeeee! I am transmitting data!!");
}

void setup() {
    // Serial and pin initialization
    Serial.begin(9600);
    while (!Serial) delay(10); // Wait for serial port to connect
    pinMode(pyro1Pin, OUTPUT);
    pinMode(pyro2Pin, OUTPUT);
    pinMode(pyroDroguePin, OUTPUT);
    pinMode(pyroMainPin, OUTPUT);
    
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

    // Initialize SD card
    if (!SD.begin(sdCardPin)) {
        Serial.println("SD card initialization failed!");
        return;
    }
    Serial.println("SD card initialized!");

    // Initialize LoRa radio for fun
    if (!rf95.init()) {
        Serial.println("LoRa radio init failed");
        while (1);
    } Serial.println("LoRa radio initialized!");

    rf95.setFrequency(RF95_FREQ);
    rf95.setTxPower(23, false);
   
    // Initialize apogee detector
    init_apogee_detector(&detector, altitude_backing_array, WINDOW_SIZE);
}

void loop() {
  
    double current_altitude = bmp.readAltitude(1013.25); // Assuming sea level pressure
    update_apogee_detector(&detector, current_altitude);
    
    if (is_apogee_reached(&detector) && !apogeeReached) {
        apogeeReached = true;
        // Additional logic for apogee event
    }
    delay(100); // Adjust delay as needed
   
if (!launchDetected) {
        launchDetected = detectLaunch(bmp);
    } else if (!firstStageBurnoutDetected) {
        firstStageBurnoutDetected = detectFirstStageBurnout(bmp);
    } else if (firstStageBurnoutDetected && !apogeeReached) {
        deployFirstStagePyros();
    } else if (firstStageBurnoutDetected && apogeeReached && !mainChuteDeployed) {
        separateStages();
        lightUpperStageMotor();
        deploySecondStageDroguePyros(detector, bmp, apogeeReached);
    } else if (apogeeReached && mainChuteDeployed) {
        if (!isLanded) {
            isLanded = detectLanding(bmp);
        } else {
            enterLowPowerMode(logData, transmitData);
        }
    }
    delay(100); // I am delaying to prevent excess polling

   if(!isLowPowerModeEntered){
    logData();
    transmitData();
    //handleCameraErrors(); // Check for camera errors
    delay(100);
   }
}