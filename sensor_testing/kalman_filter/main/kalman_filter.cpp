#include "kalman_filter.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>

void fill(double* array, int size, double value) {
    for (int i = 0; i < size; ++i) {
        array[i] = value;
    }
}

void fill2D(double array[][6], int rows, int cols, double value) {
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            array[i][j] = value;
        }
    }
}

void copy(double* source, double* destination, int size) {
    for (int i = 0; i < size; ++i) {
        destination[i] = source[i];
    }
}

void copy2D(double source[][6], double destination[][6], int rows, int cols) {
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            destination[i][j] = source[i][j];
        }
    }
}

Kalman::Kalman(Adafruit_BMP280 *bmp, Adafruit_BNO055 *bno) {
    this->bmp = bmp;
    this->bno = bno;
    init();
}

void Kalman::init() {
    fill2D(A, 6, 6, 0.0);
    A[0][0] = 1.0; A[1][1] = 1.0;
    A[2][2] = 1.0; A[3][3] = 1.0; A[4][4] = 1.0; A[5][5] = 1.0;

    fill2D(Q, 6, 6, 0.0);
    for (int i = 0; i < 6; ++i) {
        Q[i][i] = 0.01;
    }

    R[0][0] = 3.0; R[0][1] = 0.0;
    R[1][0] = 0.0; R[1][1] = 0.1;

    fill2D(H, 2, 6, 0.0);
    H[0][0] = 1.0; // Altitude measurement
    H[1][5] = 1.0; // Acceleration measurement

    fill2D(current_p_cov, 6, 6, 0.0);
    for (int i = 0; i < 6; ++i) {
        current_p_cov[i][i] = 1.0;
    }

    filteredAltitude = 0.0;
    filteredVelocity = 0.0;

    current_state[0] = bmp->readAltitude(); // Altitude
    current_state[1] = 0.0; // Velocity
    imu::Quaternion q = bno->getQuat();
    current_state[2] = q.w();
    current_state[3] = q.x();
    current_state[4] = q.y();
    current_state[5] = q.z();
}

void Kalman::begin() {
    old_time = millis();
}

void Kalman::update() {
    curr_time = millis();
    dt = ((double)(curr_time - old_time) / 1000.0);
    old_time = curr_time;

    A[0][1] = dt; // Update A matrix with current dt

    predict_state();
    predict_p_cov();
    update_gain();
    adjust_state();
    adjust_p_cov();

    filteredAltitude = adjusted_state[0];
    filteredVelocity = adjusted_state[1];

    copy(adjusted_state, current_state, 6);
    copy2D(adjusted_p_cov, current_p_cov, 6, 6);
}

void Kalman::predict_state() {
    for (int i = 0; i < 6; ++i) {
        predicted_state[i] = 0.0;
        for (int j = 0; j < 6; ++j) {
            predicted_state[i] += A[i][j] * current_state[j];
        }
    }
}

void Kalman::predict_p_cov() {
    double AP[6][6] = {0};
    double APA_T[6][6] = {0}; // Transposed APA

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            for (int k = 0; k < 6; ++k) {
                AP[i][j] += A[i][k] * current_p_cov[k][j];
            }
        }
    }

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            for (int k = 0; k < 6; ++k) {
                APA_T[i][j] += AP[i][k] * A[j][k]; // Note that A[j][k] represents A'
            }
            predicted_p_cov[i][j] = APA_T[i][j] + Q[i][j];
        }
    }
}

void Kalman::update_gain() {
    double PHt[6][2] = {0};
    double HPHt[2][2] = {0};
    double HPHtR[2][2] = {0};
    double HPHtR_inv[2][2] = {0};
    double det;

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 2; ++j) {
            PHt[i][j] = 0.0;
            for (int k = 0; k < 6; ++k) {
                PHt[i][j] += predicted_p_cov[i][k] * H[j][k];
            }
        }
    }

    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            HPHt[i][j] = 0.0;
            for (int k = 0; k < 6; ++k) {
                HPHt[i][j] += H[i][k] * PHt[k][j];
            }
            HPHtR[i][j] = HPHt[i][j] + R[i][j];
        }
    }

    det = HPHtR[0][0] * HPHtR[1][1] - HPHtR[0][1] * HPHtR[1][0];
    HPHtR_inv[0][0] = HPHtR[1][1] / det;
    HPHtR_inv[1][1] = HPHtR[0][0] / det;
    HPHtR_inv[0][1] = -HPHtR[0][1] / det;
    HPHtR_inv[1][0] = -HPHtR[1][0] / det;

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 2; ++j) {
            kalman_gain[i][j] = 0.0;
            for (int k = 0; k < 2; ++k) {
                kalman_gain[i][j] += PHt[i][k] * HPHtR_inv[k][j];
            }
        }
    }
}

void Kalman::adjust_state() {
    double measurement[2] = {bmp->readAltitude(), bno->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER).z() - 9.81};
    double y[2] = {measurement[0], measurement[1]};
    double Hx[2] = {0};

    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 6; ++j) {
            Hx[i] += H[i][j] * predicted_state[j];
        }
    }

    for (int i = 0; i < 2; ++i) {
        y[i] -= Hx[i];
    }

    for (int i = 0; i < 6; ++i) {
        adjusted_state[i] = predicted_state[i];
        for (int j = 0; j < 2; ++j) {
            adjusted_state[i] += kalman_gain[i][j] * y[j];
        }
    }
}

void Kalman::adjust_p_cov() {
    double I_KH[6][6] = {0};
    double KHP[6][6] = {0};
    double KH[6][6] = {0};

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            KH[i][j] = 0.0;
            for (int k = 0; k < 2; ++k) {
                KH[i][j] += kalman_gain[i][k] * H[k][j];
            }
            if (i == j) {
                I_KH[i][j] = 1.0 - KH[i][j];
            } else {
                I_KH[i][j] = -KH[i][j];
            }
        }
    }

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            KHP[i][j] = 0.0;
            for (int k = 0; k < 6; ++k) {
                KHP[i][j] += I_KH[i][k] * predicted_p_cov[k][j];
            }
        }
    }

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            adjusted_p_cov[i][j] = KHP[i][j];
        }
    }
}

double Kalman::getFilteredAltitude() const {
    return filteredAltitude;
}

double Kalman::getFilteredVelocity() const {
    return filteredVelocity;
}

double Kalman::getRawAltitude() const {
    return bmp->readAltitude();
}

double Kalman::getRawAcceleration() const {
    return bno->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER).z() - 9.81;
}