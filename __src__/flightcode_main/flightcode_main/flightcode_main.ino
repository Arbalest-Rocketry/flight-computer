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
#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
#include "EKF.h" 
#include "apogee.h"
#include "rocket_stages.h" 

EKF ekf;
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

// Function to convert Euler angles to quaternions
void eulerToQuaternion(float yaw, float pitch, float roll, float* qr, float* qi, float* qj, float* qk) {
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    *qr = cy * cp * cr + sy * sp * sr;
    *qi = cy * cp * sr - sy * sp * cr;
    *qj = sy * cp * sr + cy * sp * cr;
    *qk = sy * cp * cr - cy * sp * sr;
}

// CRC-8 calculation using polynomial 0xD5
byte calculateCRC(byte *data, byte len) {
    byte crc = 0;
    for (byte i = 0; i < len; i++) {
        crc ^= data[i];
        for (byte j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0xD5;  // Polynomial for DVB-S2 CRC-8
            else
                crc <<= 1;
        }
    }
    return crc;
}

/* --------------------------------------------------------------------------------------------------------------
void turnOnCam() {
  byte turnOnCommand[] = {0xCC, 0x01}; // Command to simulate power button press to turn on the camera
  byte turnOnCRC = calculateCRC(turnOnCommand, sizeof(turnOnCommand));
  Serial1.write(turnOnCommand, sizeof(turnOnCommand));
  Serial1.write(turnOnCRC);
  Serial.println("Camera turned on.");
}
 Turn on wouldn't be needed since RunCam turns on as flight computer does, so only turning off will suffice.
-------------------------------------------------------------------------------------------------------------- */

void startRec() {
  byte startCommand[] = {0xCC, 0x03}; // Command byte for starting recording
  byte startCRC = calculateCRC(startCommand, sizeof(startCommand));
  Serial1.write(startCommand, sizeof(startCommand));
  Serial1.write(startCRC);
  digitalWrite(13, HIGH); // Turn on LED to indicate recording has started
  Serial.println("Recording started.");
}

void stopRec() {
  byte stopCommand[] = {0xCC, 0x04}; // Command byte for stopping recording
  byte stopCRC = calculateCRC(stopCommand, sizeof(stopCommand));
  Serial1.write(stopCommand, sizeof(stopCommand));
  Serial1.write(stopCRC);
  digitalWrite(13, LOW); // Turn off LED after recording stops
  Serial.println("Recording stopped.");
}

// TODO: Implement this in such a way it simulates a "long press" as that is how the cam is truned on/off
void turnOffCam() {
  byte turnOffCommand[] = {0xCC, 0x01}; // Command to simulate power button press (like a click - this would not suffice!) to turn off the camera
  byte turnOffCRC = calculateCRC(turnOffCommand, sizeof(turnOffCommand));
  Serial1.write(turnOffCommand, sizeof(turnOffCommand));
  Serial1.write(turnOffCRC);
  digitalWrite(13, HIGH); // Turn on LED after cam turns off
  Serial.println("Camera turned off.");
}


void logData() {
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (dataFile) {
        sensors_event_t event;
        bno.getEvent(&event);
        float yaw = event.orientation.x;
        float pitch = event.orientation.y;
        float roll = event.orientation.z;
        float qr, qi, qj, qk;
        eulerToQuaternion(yaw, pitch, roll, &qr, &qi, &qj, &qk);
        float temperature = bmp.readTemperature();
        double altitude = bmp.readAltitude(1013.25);
        float pressure = bmp.readPressure();

        dataFile.print(millis());
        dataFile.print(",");
        dataFile.print(temperature);
        dataFile.print(",");
        dataFile.print(pressure);
        dataFile.print(",");
        dataFile.print(altitude, 2); // Print altitude with 2 decimal places
        dataFile.print(",");
        dataFile.print(qr, 2);
        dataFile.print(",");
        dataFile.print(qi, 2);
        dataFile.print(",");
        dataFile.print(qj, 2);
        dataFile.print(",");
        dataFile.print(qk, 2);
        dataFile.println();
        dataFile.close();
    } else {
        Serial.println("Error opening datalog.txt");
    }
}

void transmitData() {
    Serial.println("Transmitting data ...");
    DynamicJsonDocument doc(256);
    sensors_event_t event;
    bno.getEvent(&event);

    float yaw = event.orientation.x;
    float pitch = event.orientation.y;
    float roll = event.orientation.z;
    float qr, qi, qj, qk;
    eulerToQuaternion(yaw, pitch, roll, &qr, &qi, &qj, &qk);
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

void setup() {
    // Serial and pin initialization
    Serial1.begin(115200); 
    Serial2.begin(115200); 
    Serial3.begin(115200); 
    Serial.begin(9600);
    while (!Serial) delay(10); // Wait for serial port to connect
    pinMode(pyro1Pin, OUTPUT);
    pinMode(pyro2Pin, OUTPUT);
    pinMode(pyroDroguePin, OUTPUT);
    pinMode(pyroMainPin, OUTPUT);
    pinMode(13, OUTPUT);   // Set pin 13 as output for the LED
    
    // Check if camera communication ports are initialized
    /* if (!Serial1) {
        Serial.println("Error: One or more camera communication ports not initialized.");
        while (1);
    } */

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

    // Initialize LoRa radio
    if (!rf95.init()) {
        Serial.println("LoRa radio init failed");
        while (1);
    } Serial.println("LoRa radio initialized!");

    rf95.setFrequency(RF95_FREQ);
    rf95.setTxPower(23, false);
    
    startRec();

    // Wait for flight computer to turn on (10 minutes)
    // delay(600000); // 600000 milliseconds = 10 minutes
    // TODO: Find a way to delay cam from turning on for 10 minutes.
    delay(100000);
    // startRec();

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
    /*
    sensors_event_t event;
    bno.getEvent(&event);

    // Get sensor data
    Vector3d accel(event.acceleration.x, event.acceleration.y, event.acceleration.z);
    Vector3d mag(event.magnetic.x, event.magnetic.y, event.magnetic.z);
    Vector3d gyro(event.gyro.x, event.gyro.y, event.gyro.z);

    double dt = 0.01; // Time step, adjust as needed

    ekf.predict(gyro, dt);
    ekf.update(accel, mag);

    Eigen::Vector3d eulerAngles = ekf.getEulerAngles(ekf.getXHat().head<4>());

    Serial.print("Roll: ");
    Serial.print(eulerAngles(0) * 180 / M_PI);
    Serial.print(", Pitch: ");
    Serial.print(eulerAngles(1) * 180 / M_PI);
    Serial.print(", Yaw: ");
    Serial.println(eulerAngles(2) * 180 / M_PI);
    */

 /*/////////////////////////////////////////////////*/  
          /* TODO: Add Rocket Stages Logic */
 /*/////////////////////////////////////////////////*/ 
 
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
    
    /*
    // Read orientation from BNO055 sensor
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float yaw = euler.x();
    float pitch = euler.y();
    float roll = euler.z();
    
    // Convert Euler angles to quaternion
    float qr, qi, qj, qk;
    eulerToQuaternion(yaw, pitch, roll, &qr, &qi, &qj, &qk);
    
    // Normalize the quaternion
    float quat[4] = {qr, qi, qj, qk};
    normalizeQuat(quat);
    
    // Display quaternion
    Serial.print("Quaternion: ");
    Serial.print(quat[0], 4);
    Serial.print(", ");
    Serial.print(quat[1], 4);
    Serial.print(", ");
    Serial.print(quat[2], 4);
    Serial.print(", ");
    Serial.println(quat[3], 4);
    */

    // Check for low power mode
    if(!isLowPowerModeEntered){
        logData();
        transmitData();
        //handleCameraErrors(); // Check for camera errors
        delay(100);
    }
}