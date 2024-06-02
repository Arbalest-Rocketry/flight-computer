#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include <math.h>
#include <iostream>
#include <cmath>
#include <cstring>

const int sdCardPin = 10;

Adafruit_BMP280 bmp; // bmp object
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // BNO055 object

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

        // DEBUG
        std::cout << "Rotation matrix (from quaternion):" << std::endl;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                std::cout << rotMat[i][j] << " ";
            }
            std::cout << std::endl;
        }
    }

    void getAccelVector(float* a, float* accelOut) {
        float accelMag = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        if (accelMag > 0) { // Avoid division by zero
            for (int i = 0; i < 3; i++) {
                accelOut[i] = a[i] / accelMag;
            }
        } else {
            for (int i = 0; i < 3; i++) {
                accelOut[i] = 0;
            }
        }

        // DEBUG
        std::cout << "Normalized accelerometer vector: ";
        for (int i = 0; i < 3; ++i) {
            std::cout << accelOut[i] << " ";
        }
        std::cout << std::endl;
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
        if (magNorm > 0) { // Avoid division by zero
            for (int i = 0; i < 2; i++) {
                magGauss_N[i] /= magNorm;
            }
        } else {
            for (int i = 0; i < 2; i++) {
                magGauss_N[i] = 0;
            }
        }
        for (int i = 0; i < 3; i++) {
            magOut[i] = 0;
            for (int j = 0; j < 3; j++) {
                magOut[i] += rotMat[j][i] * magGauss_N[j];
            }
        }

        // DEBUG
        std::cout << "Normalized magnetometer vector: ";
        for (int i = 0; i < 3; ++i) {
            std::cout << magOut[i] << " ";
        }
        std::cout << std::endl;
    }

    void getJacobianMatrix(float* reference, float hPrime[3][4]) {
        float* qHatPrev = xHatPrev;
        float e[3][4] = {
            {2 * qHatPrev[2], -2 * qHatPrev[3], 2 * qHatPrev[0], -2 * qHatPrev[1]},
            {-2 * qHatPrev[1], -2 * qHatPrev[0], -2 * qHatPrev[3], -2 * qHatPrev[2]},
            {-2 * qHatPrev[0], 2 * qHatPrev[1], 2 * qHatPrev[2], -2 * qHatPrev[3]}
        };
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                hPrime[i][j] = 0;
                for (int k = 0; k < 3; k++) {
                    hPrime[i][j] += (i == k ? 1 : 0 - reference[i] * reference[k]) * e[k][j];
                }
            }
        }

        // DEBUG
        std::cout << "Jacobian matrix (hPrime):" << std::endl;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 4; ++j) {
                std::cout << hPrime[i][j] << " ";
            }
            std::cout << std::endl;
        }
    }

    void kalman() {
    // Normalize quaternion
    normalizeQuat(xHat);
    
    // DEBUG
    std::cout << "Quaternion (normalized): ";
    for (int i = 0; i < 4; ++i) {
        std::cout << xHat[i] << " ";
    }
    std::cout << std::endl;

    // Prediction Step: Estimate state and error covariance
    // Assuming a simple motion model where xHatBar = xHat for this example

    // Update step: Incorporate measurements
    float accel[3], mag[3];
    sensors_event_t accelEvent;
    sensors_event_t magEvent;
    bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&magEvent, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    
    float accelData[3] = {accelEvent.acceleration.x, accelEvent.acceleration.y, accelEvent.acceleration.z};
    float magData[3] = {magEvent.magnetic.x, magEvent.magnetic.y, magEvent.magnetic.z};
    
    getAccelVector(accelData, accel);
    getMagVector(magData, mag);

    // Here, incorporate the accelerometer and magnetometer data into the EKF update step
    // Update state xHat and covariance matrix p based on the new measurements

    // Example update equations (highly simplified and should be expanded for a real EKF)
    for (int i = 0; i < 3; i++) {
        // Innovation (measurement residual)
        float y_accel = accel[i] - accelReference[i];
        float y_mag = mag[i] - magReference[i];

        // Update state estimate
        xHat[i+4] += K[i][i] * y_accel;  // Update quaternion
        xHat[i+4] += K[i+3][i] * y_mag;  // Update bias

        // Update covariance matrix
        // p = (I - K * C) * p;
    }

    // Normalize quaternion again after update
    normalizeQuat(xHat);
    
    // DEBUG: Print updated quaternion
    std::cout << "Updated Quaternion: ";
    for (int i = 0; i < 4; ++i) {
        std::cout << xHat[i] << " ";
    }
    std::cout << std::endl;

    // DEBUG: Print updated state covariance matrix
    std::cout << "Updated Covariance Matrix: " << std::endl;
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 7; ++j) {
            std::cout << p[i][j] << " ";
        }
        std::cout << std::endl;
    }
}
};

System ekf;

void setup() {
    Serial.begin(9600);
    if (!bno.begin()) {
        Serial.print("No BNO055 detected.");
        while (1);
    }
    if (!bmp.begin()) {
        Serial.print("No BMP280 detected.");
        while (1);
    }
    if (!SD.begin(sdCardPin)) {
        Serial.print("SD card failed.");
        while (1);
    }
    bno.setExtCrystalUse(true);
}

void loop() {
    sensors_event_t accelEvent;
    sensors_event_t magEvent;
    bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&magEvent, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    
    float accelData[3] = {accelEvent.acceleration.x, accelEvent.acceleration.y, accelEvent.acceleration.z};
    float magData[3] = {magEvent.magnetic.x, magEvent.magnetic.y, magEvent.magnetic.z};

    float accelNormalized[3];
    float magNormalized[3];
    
    ekf.getAccelVector(accelData, accelNormalized);
    ekf.getMagVector(magData, magNormalized);

    // Kalman filter step
    ekf.kalman();

    delay(100);
}
/*
int freeMemory() {
    extern int __heap_start, *__brkval;
    int v;
    return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
} */