#pragma once
#include <Arduino.h>

class EKF {
private:
    double A[3][3];     // State transition matrix for position
    double Q[3][3];     // Model noise covariance
    double H[2][3];     // Measurement jacobian
    double R[2][2];     // Measurement noise covariance
    double predicted_state[3];
    double adjusted_state[3];
    double current_p_cov[3][3];     // Process covariance
    double predicted_p_cov[3][3];
    double adjusted_p_cov[3][3];
    double EKF_gain[3][2];

    unsigned long old_time;
    unsigned long curr_time;
    double dt;

    // EKF filter functions
    void predict_state();
    void predict_p_cov();
    void update_gain();
    void adjust_p_cov();
    void adjust_state();
    void init();

public:
    double measurement[2];
    double current_state[3];

    EKF();   // Constructor
    void update(double barometerAltitude, double accelZ);   // Main EKF filter caller
    void begin(double initialBarometerAltitude, double initialAccelZ);
};