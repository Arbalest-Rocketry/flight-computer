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

// Define pin numbers
const int pyro1Pin = 20;
const int pyro2Pin = 21;
const int pyroDroguePin = 22; 
const int pyroMainPin = 23;    
const int sdCardPin = 10;
const int runCam1TX = 1; 
const int runCam1RX = 0; 
const int runCam2TX = 8; 
const int runCam2RX = 7; 
const int runCam3TX = 14; 
const int runCam3RX = 15; 

// LoRa settings
#define RFM95_CS 1
#define RFM95_RST 34
#define RFM95_INT 8
#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);  // Declare the rf95 object

// Initialize SoftwareSerial for RunCam communication
SoftwareSerial runCamSerial1(runCam1RX, runCam1TX); 
SoftwareSerial runCamSerial2(runCam2RX, runCam2TX);
SoftwareSerial runCamSerial3(runCam3RX, runCam3TX);

#define WINDOW_SIZE 20
#define DECREASE_THRESHOLD 2 // Number of consecutive decreases to confirm apogee

// ------------------------- FLAGS --------- //
bool apogeeReached = false;
bool mainChuteDeployed = false;
bool launchDetected = false;
bool firstStageBurnoutDetected = false;
bool isLanded = false;
// ----------------------------------------- //

unsigned long startTime;  // To store the start time for camera activation

typedef struct {
    double *backing_array;
    size_t capacity;
    size_t size;
    size_t front;
    double sum_of_elements;
} RollingWindow;

typedef struct {
    RollingWindow altitude_window;
    double last_altitude;
    int apogee_reached;
    int decrease_count; // Track consecutive decreases
} ApogeeDetector;

Adafruit_BMP280 bmp; // bmp object
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // BNO055 object

double altitude_backing_array[WINDOW_SIZE];
ApogeeDetector detector;

// Rolling Window Functions
void init_rolling_window(RollingWindow *rw, double *pBackingArray, size_t capacity) {
    rw->backing_array = pBackingArray;
    rw->capacity = capacity;
    rw->size = 0;
    rw->front = 0;
    rw->sum_of_elements = 0.0;
}

size_t mod_rolling_window(size_t index, size_t modulo) {
    return (index % modulo + modulo) % modulo;
}

void add_data_point_rolling_window(RollingWindow *rw, double dataPoint) {
    if (rw->size >= rw->capacity) {
        rw->sum_of_elements -= rw->backing_array[mod_rolling_window(rw->front, rw->capacity)];
        rw->front = mod_rolling_window(rw->front + 1, rw->capacity);
        rw->size--;
    }
    size_t back_insertion_idx = mod_rolling_window(rw->front + rw->size, rw->capacity);
    rw->backing_array[back_insertion_idx] = dataPoint;
    rw->sum_of_elements += dataPoint;
    rw->size++;
}

double get_latest_datapoint_rolling_window(RollingWindow *rw) {
    return rw->backing_array[mod_rolling_window(rw->front + rw->size - 1, rw->capacity)];
}

double get_earliest_datapoint_rolling_window(RollingWindow *rw) {
    return rw->backing_array[mod_rolling_window(rw->front, rw->capacity)];
}

// Apogee Detector Functions
void init_apogee_detector(ApogeeDetector *detector, double *backing_array, size_t capacity) {
    init_rolling_window(&detector->altitude_window, backing_array, capacity);
    detector->last_altitude = -1.0;
    detector->apogee_reached = 0;
    detector->decrease_count = 0;
}

void update_apogee_detector(ApogeeDetector *detector, double current_altitude) {
    add_data_point_rolling_window(&detector->altitude_window, current_altitude);

    if (detector->apogee_reached) {
        return; // Exit the function if apogee has already been reached
    }

    if (detector->last_altitude >= 0 && current_altitude < detector->last_altitude) {
        detector->decrease_count++;
    } else {
        detector->decrease_count = 0; // Reset counter if altitude increases or stays the same
    }

    if (detector->decrease_count >= DECREASE_THRESHOLD) {
        detector->apogee_reached = 1;
    }
    detector->last_altitude = current_altitude;
}

int is_apogee_reached(ApogeeDetector *detector) {
    return detector->apogee_reached;
}

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

// Function to convert radians to degrees
float rad2deg(float rad) {
    return rad / M_PI * 180;
}

// Function to convert degrees to radians
float deg2rad(float deg) {
    return deg / 180 * M_PI;
}

// Function to normalize a quaternion
void normalizeQuat(float* q) {
    float mag = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (mag == 0) return; // Avoid division by zero
    q[0] /= mag;
    q[1] /= mag;
    q[2] /= mag;
    q[3] /= mag;
}


// Quaternion to rotation matrix
void getRotMat(float* q, float rotMat[3][3]) {
    rotMat[0][0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    rotMat[0][1] = 2 * (q[1] * q[2] - q[0] * q[3]);
    rotMat[0][2] = 2 * (q[1] * q[3] + q[0] * q[2]);
    rotMat[1][0] = 2 * (q[1] * q[2] + q[0] * q[3]);
    rotMat[1][1] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
    rotMat[1][2] = 2 * (q[2] * q[3] - q[0] * q[1]);
    rotMat[2][0] = 2 * (q[1] * q[3] - q[0] * q[2]);
    rotMat[2][1] = 2 * (q[2] * q[3] + q[0] * q[1]);
    rotMat[2][2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
}

// Get Euler angles from quaternion
void getEulerAngles(float* q, float* yaw, float* pitch, float* roll) {
    float rotMat[3][3];
    getRotMat(q, rotMat);
    float test = -rotMat[2][0];
    if (test > 0.99999) {
        *yaw = 0;
        *pitch = M_PI / 2;
        *roll = atan2(rotMat[0][1], rotMat[0][2]);
    } else if (test < -0.99999) {
        *yaw = 0;
        *pitch = -M_PI / 2;
        *roll = atan2(-rotMat[0][1], -rotMat[0][2]);
    } else {
        *yaw = atan2(rotMat[1][0], rotMat[0][0]);
        *pitch = asin(-rotMat[2][0]);
        *roll = atan2(rotMat[2][1], rotMat[2][2]);
    }
    *yaw = rad2deg(*yaw);
    *pitch = rad2deg(*pitch);
    *roll = rad2deg(*roll);
}

class System {
public:
    float xHat[7];
    float yHatBar[3];
    float p[7][7];
    float Q[7][7];
    float R[6][6];
    float K[7][6];
    float A[7][7];
    float B[7][6];
    float C[6][7];
    float xHatBar[7];
    float xHatPrev[7];
    float pBar[7][7];
    float accelReference[3];
    float magReference[3];
    float mag_Ainv[3][3];
    float mag_b[3];

    System() {
        float quaternion[4] = {1, 0, 0, 0};
        float bias[3] = {0, 0, 0};
        memcpy(xHat, quaternion, sizeof(quaternion));
        memcpy(xHat + 4, bias, sizeof(bias));
        memset(yHatBar, 0, sizeof(yHatBar));
        for (int i = 0; i < 7; i++) {
            for (int j = 0; j < 7; j++) {
                p[i][j] = (i == j) ? 0.01f : 0.0f;
                Q[i][j] = (i == j) ? 0.001f : 0.0f;
            }
        }
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                R[i][j] = (i == j) ? 0.1f : 0.0f;
            }
        }
        accelReference[0] = 0;
        accelReference[1] = 0;
        accelReference[2] = -1;
        magReference[0] = 0;
        magReference[1] = -1;
        magReference[2] = 0;

        float mag_Ainv_tmp[3][3] = {
            {2.06423128e-03, -1.04778851e-04, -1.09416190e-06},
            {-1.04778851e-04, 1.91693168e-03, 1.79409312e-05},
            {-1.09416190e-06, 1.79409312e-05, 1.99819154e-03}
        };
        memcpy(mag_Ainv, mag_Ainv_tmp, sizeof(mag_Ainv));
        float mag_b_tmp[3] = {80.51340236f, 37.08931099f, 105.6731885f};
        memcpy(mag_b, mag_b_tmp, sizeof(mag_b));
    }

    void normalizeQuat(float* q) {
        float mag = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        q[0] /= mag;
        q[1] /= mag;
        q[2] /= mag;
        q[3] /= mag;
    }

    void getRotMat(float* q, float rotMat[3][3]) {
        float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
        rotMat[0][0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
        rotMat[0][1] = 2 * (q1*q2 - q0*q3);
        rotMat[0][2] = 2 * (q1*q3 + q0*q2);
        rotMat[1][0] = 2 * (q1*q2 + q0*q3);
        rotMat[1][1] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
        rotMat[1][2] = 2 * (q2*q3 - q0*q1);
        rotMat[2][0] = 2 * (q1*q3 - q0*q2);
        rotMat[2][1] = 2 * (q2*q3 + q0*q1);
        rotMat[2][2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    }

    void getAccelVector(float* a, float* accelOut) {
        float accelMag = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        for (int i = 0; i < 3; i++) {
            accelOut[i] = a[i] / accelMag;
        }
    }

    void getMagVector(float* m, float* magOut) {
        float magGaussRaw[3];
        for (int i = 0; i < 3; i++) {
            magGaussRaw[i] = 0;
            for (int j = 0; j < 3; j++) {
                magGaussRaw[i] += mag_Ainv[i][j] * (m[j] - mag_b[j]);
            }
        }
        float rotMat[3][3];
        getRotMat(xHat, rotMat);
        float magGauss_N[3];
        for (int i = 0; i < 3; i++) {
            magGauss_N[i] = 0;
            for (int j = 0; j < 3; j++) {
                magGauss_N[i] += rotMat[i][j] * magGaussRaw[j];
            }
        }
        magGauss_N[2] = 0;
        float magNorm = sqrt(magGauss_N[0] * magGauss_N[0] + magGauss_N[1] * magGauss_N[1]);
        for (int i = 0; i < 2; i++) {
            magGauss_N[i] /= magNorm;
        }
        for (int i = 0; i < 3; i++) {
            magOut[i] = 0;
            for (int j = 0; j < 3; j++) {
                magOut[i] += rotMat[j][i] * magGauss_N[j];
            }
        }
    }

    void getJacobianMatrix(float* reference, float hPrime[3][4]) {
        float* qHatPrev = xHatPrev;
        float e[3][4] = {
            {2 * qHatPrev[2], -2 * qHatPrev[3], 2 * qHatPrev[0], -2 * qHatPrev[1]},
            {-2 * qHatPrev[1], -2 * qHatPrev[0], -2 * qHatPrev[3], -2 * qHatPrev[2]},
            {2 * qHatPrev[0], -2 * qHatPrev[1], -2 * qHatPrev[2], 2 * qHatPrev[3]}
        };
        for (int i = 0; i < 3; i++) {
            float rowSum = 0;
            for (int j = 0; j < 4; j++) {
                rowSum += e[i][j] * qHatPrev[j];
            }
            float refI = reference[i];
            for (int j = 0; j < 4; j++) {
                hPrime[i][j] = (refI - rowSum) * e[i][j];
            }
        }
    }

    void gethPrime(float* accel, float* mag, float hPrime[6][7]) {
        float qHatPrev[4] = {xHatPrev[0], xHatPrev[1], xHatPrev[2], xHatPrev[3]};
        float accelOut[3];
        getAccelVector(accel, accelOut);
        float magOut[3];
        getMagVector(mag, magOut);
        for (int i = 0; i < 3; i++) {
            yHatBar[i] = accelOut[i];
            yHatBar[i + 3] = magOut[i];
        }
        float accelHPrime[3][4];
        getJacobianMatrix(accelReference, accelHPrime);
        float magHPrime[3][4];
        getJacobianMatrix(magReference, magHPrime);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                hPrime[i][j] = accelHPrime[i][j];
                hPrime[i + 3][j] = magHPrime[i][j];
            }
        }
        for (int i = 0; i < 6; i++) {
            for (int j = 4; j < 7; j++) {
                hPrime[i][j] = 0;
            }
        }
    }

    void propagateEKF(float* accel, float* mag) {
        // Prediction step
        for (int i = 0; i < 7; i++) {
            xHatBar[i] = xHat[i];
            for (int j = 0; j < 7; j++) {
                pBar[i][j] = p[i][j] + Q[i][j];
            }
        }
        // Update step
        float hPrime[6][7];
        gethPrime(accel, mag, hPrime);
        float hPrimeT[7][6];
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 7; j++) {
                hPrimeT[j][i] = hPrime[i][j];
            }
        }
        float s[6][6] = {0};
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                for (int k = 0; k < 7; k++) {
                    s[i][j] += hPrime[i][k] * pBar[k][j];
                }
            }
        }
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                s[i][j] += R[i][j];
            }
        }
        // Calculate Kalman gain
        float sInv[6][6];
        invertMatrix(s, sInv);
        for (int i = 0; i < 7; i++) {
            for (int j = 0; j < 6; j++) {
                K[i][j] = 0;
                for (int k = 0; k < 7; k++) {
                    K[i][j] += pBar[i][k] * hPrimeT[k][j];
                }
                for (int k = 0; k < 6; k++) {
                    K[i][j] *= sInv[j][k];
                }
            }
        }
        // Update estimate
        float yBar[6];
        for (int i = 0; i < 6; i++) {
            yBar[i] = (i < 3) ? accel[i] : mag[i - 3];
            yBar[i] -= yHatBar[i];
        }
        for (int i = 0; i < 7; i++) {
            for (int j = 0; j < 6; j++) {
                xHat[i] += K[i][j] * yBar[j];
            }
        }
        normalizeQuat(xHat);
        // Update error covariance
        float KH[7][7] = {0};
        for (int i = 0; i < 7; i++) {
            for (int j = 0; j < 7; j++) {
                for (int k = 0; k < 6; k++) {
                    KH[i][j] += K[i][k] * hPrime[k][j];
                }
            }
        }
        float I_KH[7][7];
        for (int i = 0; i < 7; i++) {
            for (int j = 0; j < 7; j++) {
                I_KH[i][j] = (i == j) ? 1 - KH[i][j] : -KH[i][j];
            }
        }
        for (int i = 0; i < 7; i++) {
            for (int j = 0; j < 7; j++) {
                p[i][j] = 0;
                for (int k = 0; k < 7; k++) {
                    p[i][j] += I_KH[i][k] * pBar[k][j];
                }
            }
        }
    }

void invertMatrix(float input[6][6], float output[6][6]) {
    int n = 6;
    float LU[6][6];
    float tempRow[6];

    // Initialize LU matrix with input
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            LU[i][j] = input[i][j];
            output[i][j] = (i == j) ? 1.0f : 0.0f; // Output matrix initialized as identity matrix
        }
    }

    // Perform LU decomposition with partial pivoting
    for (int k = 0; k < n; k++) {
        // Find pivot
        int pivot = k;
        for (int i = k + 1; i < n; i++) {
            if (fabs(LU[i][k]) > fabs(LU[pivot][k])) {
                pivot = i;
            }
        }

        // Swap rows if necessary
        if (pivot != k) {
            for (int j = 0; j < n; j++) {
                std::swap(LU[pivot][j], LU[k][j]);
                std::swap(output[pivot][j], output[k][j]);
            }
        }

        // Compute multipliers
        for (int i = k + 1; i < n; i++) {
            if (LU[k][k] != 0.0) {
                float multiplier = LU[i][k] / LU[k][k];
                for (int j = k; j < n; j++) {
                    LU[i][j] -= multiplier * LU[k][j];
                }
                for (int j = 0; j < n; j++) {
                    output[i][j] -= multiplier * output[k][j];
                }
            }
        }
    }

    // Back substitution to solve for the inverted matrix
    for (int k = n - 1; k >= 0; k--) {
        if (LU[k][k] != 0.0) {
            for (int j = 0; j < n; j++) {
                tempRow[j] = output[k][j] / LU[k][k];
            }
            for (int i = 0; i < k; i++) {
                for (int j = 0; j < n; j++) {
                    output[i][j] -= LU[i][k] * tempRow[j];
                }
            }
            for (int j = 0; j < n; j++) {
                output[k][j] = tempRow[j];
            }
        }
    }
}
};

System sys; // EKF system object

// Function to propagate EKF
void propagateEKF(float* accel, float* mag) {
    sys.propagateEKF(accel, mag);
}

// Define expected responses
const uint8_t expectedResponse1 = 0x81; // Confirmation on Power On
const uint8_t expectedResponse2 = 0x82; // Confirmation on Power Off
const uint8_t expectedResponse3 = 0x83; // Confirmation on Start Recording

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


bool detectLaunch() {
    static double groundAltitude = bmp.readAltitude(1013.25);
    double currentAltitude = bmp.readAltitude(1013.25);
    double altitudeDifference = currentAltitude - groundAltitude;

    // Assuming launch is detected if the altitude difference is greater than 5 meters
    if (altitudeDifference > 5.0) {
        Serial.println("Launch detected");
        return true;
    }
    return false;
}

void deployFirstStagePyros() {
    Serial.println("Deploying first stage pyros");
    digitalWrite(pyro1Pin, HIGH);
    delay(1000); // Ensure the pyro is activated
    digitalWrite(pyro1Pin, LOW);
}

bool detectFirstStageBurnout() {
    static double lastAltitude = 0;
    static unsigned long lastTime = millis();
    unsigned long currentTime = millis();
    double currentAltitude = bmp.readAltitude(1013.25);

    // Check if altitude remains constant (±0.1 meter) for more than 2 seconds
    if (abs(currentAltitude - lastAltitude) < 0.1) {
        if (currentTime - lastTime > 2000) {
            Serial.println("First stage burnout detected");
            return true;
        }
    } else {
        lastTime = currentTime;
    }
    lastAltitude = currentAltitude;
    return false;
}

void separateStages() {
    Serial.println("Separating stages");
    digitalWrite(pyro1Pin, HIGH);
    delay(1000); // Ensure the separation pyro is activated
    digitalWrite(pyro1Pin, LOW);
}

void lightUpperStageMotor() {
    Serial.println("Lighting upper stage motor");
    digitalWrite(pyro2Pin, HIGH);
    delay(1000); // Ensure the upper stage motor is ignited
    digitalWrite(pyro2Pin, LOW);
}

void deploySecondStageDroguePyros() {
    if (is_apogee_reached(&detector) && !apogeeReached) {
        Serial.println("Apogee Reached!");
        Serial.println("Deploying second stage pyros (drogue chute)");
        delay(30000);
        digitalWrite(pyroDroguePin, HIGH);
        delay(1000); // Ensure the pyro is activated
        digitalWrite(pyroDroguePin, LOW);
    } Serial.println("Apogee not reached yet.");
}

void deployMainParachutePyros() {
        if (apogeeReached && !mainChuteDeployed) {
        double currentAltitude = bmp.readAltitude(1013.25);
        if (currentAltitude <= 500) { // Assuming altitude is in meters
            Serial.println("500 meters above ground level on descent reached");
            Serial.println("Deploying main parachute pyros");
            digitalWrite(pyroMainPin, HIGH);
            delay(1000);
            digitalWrite(pyroMainPin, LOW);
            mainChuteDeployed = true;
        }
    }
}

bool detectLanding() {
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

void enterLowPowerMode() {
    Serial.println("Entering low power mode");

    // Set BNO, GPS to low power mode (I am putting pseudocode for now)
    bno.setExtCrystalUse(false); // Example of setting BNO055 to low power mode

    while (true) {
        transmitData();
        logData();
        delay(30000); // Transmit data every 30 seconds
    }
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
    // Initialize serial communication
    Serial.begin(9600);
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

    // Initialize pins
    pinMode(pyro1Pin, OUTPUT);
    pinMode(pyro2Pin, OUTPUT);
    pinMode(pyroDroguePin, OUTPUT);
    pinMode(pyroMainPin, OUTPUT);

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
    startTime = millis(); // Initialize start time
}

void loop() {    
    // Read current altitude
    double currentAltitude = bmp.readAltitude(1013.25); // Assuming sea level pressure is 1013.25 hPa

    // Detect launch
    if (!launchDetected) {
        launchDetected = detectLaunch();
    }

    // Detect first stage burnout and deploy deployment pyros
    if (launchDetected && !firstStageBurnoutDetected) {
        firstStageBurnoutDetected = detectFirstStageBurnout();
        if (firstStageBurnoutDetected) {
            deployFirstStagePyros();
        }
    }

    // Light upper stage motor 10 seconds after stage separation
    if (firstStageBurnoutDetected && millis() - startTime > 10000) {
        lightUpperStageMotor();
    }
    
    // Update apogee detector with the current altitude
    update_apogee_detector(&detector, currentAltitude);

    // Deploy second stage pyros (drogue chute) if apogee is reached
    deploySecondStageDroguePyros();

    // Deploy main parachute pyros at 500 meters above ground level on descent
    deployMainParachutePyros();

    // Detect landing
    if (detectLanding()) {
        enterLowPowerMode();
    }
    
    // Print current altitude for debugging purposes
    Serial.print("Current altitude: ");
    Serial.println(currentAltitude);

    // Wait for a second before the next loop
    delay(1000);
    
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
    
    // Read acceleration and magnetometer data
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    
    /*
    // Propagate EKF with sensor data
    float accelData[3] = {accel.x(), accel.y(), accel.z()};
    float magData[3] = {mag.x(), mag.y(), mag.z()};
    propagateEKF(accelData, magData);
    */
    
    logData();
    transmitData();
    //handleCameraErrors(); // Check for camera errors
    delay(100);
}