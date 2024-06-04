#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include <SoftwareSerial.h>
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
const int RunCam1_TX = 1; 
const int RunCam1_RX = 0; 
const int RunCam2_TX = 8; 
const int RunCam2_RX = 7; 
const int RunCam3_TX = 14; 
const int RunCam3_RX = 15; 

// LoRa settings
#define RFM95_CS 1
#define RFM95_RST 34
#define RFM95_INT 8
#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);  // Declare the rf95 object

// Initialize SoftwareSerial for RunCam communication
SoftwareSerial runCamSerial1(RunCam1_RX, RunCam1_TX); 
SoftwareSerial runCamSerial2(RunCam2_RX, RunCam2_TX);
SoftwareSerial runCamSerial3(RunCam3_RX, RunCam3_TX);

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

// CRC-16-CCITT calculation function
uint16_t crc16_ccitt(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

// CRC-8 DVB-S2 calculation function
uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
    crc ^= a;
    for (uint8_t ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
        else crc = crc << 1;
    }
    return crc;
}

// Define expected responses
const uint8_t expectedResponse1 = 0x11; // (0x01 << 4) + 0x01 = 0x10 + 0x01 = 0x11 (Confirmation on Power On)
const uint8_t expectedResponse2 = 0x21; // (0x02 << 4) + 0x01 = 0x20 + 0x01 = 0x21 (Confirmation on Power Off)
const uint8_t expectedResponse3 = 0x31; // (0x03 << 4) + 0x01 = 0x30 + 0x01 = 0x31 (Confirmation on Start Recording)

void handleCameraErrors() {
    // Check if the camera communication ports are not initialized
    if (!runCamSerial1 || !runCamSerial2 || !runCamSerial3) {
        Serial.println("Error: One or more camera communication ports not initialized.");
    }

    // Check for timeout while waiting for camera response
    if (!runCamSerial1.available() || !runCamSerial2.available() || !runCamSerial3.available()) {
        Serial.println("Error: Timeout waiting for camera response.");
    }

    // Check for unexpected responses from cameras
    if (runCamSerial1.peek() != expectedResponse1 || runCamSerial2.peek() != expectedResponse2 || runCamSerial3.peek() != expectedResponse3) {
        Serial.println("Error: Unexpected response from one or more cameras.");
    }

    // Check CRC errors in received data
    if (detectCRCError(runCamSerial1) || detectCRCError(runCamSerial2) || detectCRCError(runCamSerial3)) {
        Serial.println("Error: CRC error in camera data.");
    }
}

// Helper function to detect CRC errors in received data
bool detectCRCError(SoftwareSerial& serial) {
    // Assume CRC byte is at the end of the data
    int dataLength = serial.available();
    if (dataLength < 2) {
        // Not enough data to check CRC
        return false;
    }
    uint8_t data[dataLength];
    for (int i = 0; i < dataLength; i++) {
        data[i] = serial.read();
    }
    // Calculate CRC of received data
    uint16_t receivedCRC = (data[dataLength - 2] << 8) | data[dataLength - 1];
    uint16_t calculatedCRC = crc16_ccitt(data, dataLength - 2); // Exclude CRC bytes from calculation
    return receivedCRC != calculatedCRC;
}

void sendCommand(uint8_t actionId) {
    uint8_t commandPacket[5]; // Packet structure: Header + Command ID + Action ID + CRC
    commandPacket[0] = 0xCC; // Header
    commandPacket[1] = 0x01; // RCDEVICE_PROTOCOL_COMMAND_CAMERA_CONTROL
    commandPacket[2] = actionId; // Action ID
    commandPacket[3] = crc8_dvb_s2(0, commandPacket[1]); // Calculate CRC for Command ID
    commandPacket[4] = crc8_dvb_s2(commandPacket[3], commandPacket[2]); // Calculate CRC for Action ID

    // Send the command packet over UART
    runCamSerial1.write(commandPacket, sizeof(commandPacket)); 
    runCamSerial2.write(commandPacket, sizeof(commandPacket)); 
    runCamSerial3.write(commandPacket, sizeof(commandPacket)); 
}

void sendTurnOnCommand() {
    sendCommand(0x01); // RCDEVICE_PROTOCOL_CAMERA_TURN_ON
}

void sendStartRecordingCommand() {
    sendCommand(0x03); // RCDEVICE_PROTOCOL_CAMERA_START_RECORDING
}

void sendStopRecordingCommand() {
    sendCommand(0x04); // RCDEVICE_PROTOCOL_CAMERA_STOP_RECORDING
}

void sendTurnOffCommand() {
    sendCommand(0x02); // RCDEVICE_PROTOCOL_CAMERA_TURN_OFF
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
    Serial.begin(9600);
    while (!Serial) delay(10); // Wait for serial port to connect
    pinMode(pyro1Pin, OUTPUT);
    pinMode(pyro2Pin, OUTPUT);
    pinMode(pyroDroguePin, OUTPUT);
    pinMode(pyroMainPin, OUTPUT);
    
    runCamSerial1.begin(9600);
    runCamSerial2.begin(9600);
    runCamSerial3.begin(9600);
    
    // Check if camera communication ports are initialized
    if (!runCamSerial1 || !runCamSerial2 || !runCamSerial3) {
        Serial.println("Error: One or more camera communication ports not initialized.");
        while (1);
    }

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
    
    // Wait for flight computer to turn on (10 minutes)
    // delay(600000); // 600000 milliseconds = 10 minutes
    delay(6000); // use 6 seconds just for testing at home 
    sendTurnOnCommand();
    delay(2000);
    sendStartRecordingCommand();

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