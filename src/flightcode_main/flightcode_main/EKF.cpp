//=======================================================================================================//    
//=============================           EXTENDED KALMAN FILTER            =============================//
//=======================================================================================================//

#include "EKF.h"

EKF::EKF() {
    init();
}

void EKF::init() {
    Q[0][0] = 0.8; Q[0][1] = 0; Q[0][2] = 0;
    Q[1][0] = 0; Q[1][1] = 0.4; Q[1][2] = 0;
    Q[2][0] = 0; Q[2][1] = 0; Q[2][2] = 0.2;

    R[0][0] = 3; R[0][1] = 0;
    R[1][0] = 0; R[1][1] = 0.1;

    H[0][0] = 1; H[0][1] = 0; H[0][2] = 0;
    H[1][0] = 0; H[1][1] = 0; H[1][2] = 1;

    current_p_cov[0][0] = 3; current_p_cov[0][1] = 0; current_p_cov[0][2] = 0;
    current_p_cov[1][0] = 0; current_p_cov[1][1] = 2; current_p_cov[1][2] = 0;
    current_p_cov[2][0] = 0; current_p_cov[2][1] = 0; current_p_cov[2][2] = 1;
}

void EKF::begin(double initialBarometerAltitude, double initialaccelY) {
    old_time = millis();
    current_state[0] = initialBarometerAltitude; // Altitude
    current_state[1] = 0; // Velocity
    current_state[2] = initialaccelY; // Acceleration
}

void EKF::update(double barometerAltitude, double accelY) {
    curr_time = millis();
    dt = ((double)(curr_time - old_time) / 1000);
    old_time = curr_time;

    A[0][0] = 1; A[0][1] = dt; A[0][2] = dt * dt / 2;
    A[1][0] = 0; A[1][1] = 1; A[1][2] = dt;
    A[2][0] = 0; A[2][1] = 0; A[2][2] = 1;

    measurement[0] = barometerAltitude;
    measurement[1] = accelY;

    predict_state();
    predict_p_cov();
    update_gain();
    adjust_state();
    adjust_p_cov();

    memcpy(current_state, adjusted_state, sizeof(adjusted_state));
    memcpy(current_p_cov, adjusted_p_cov, sizeof(adjusted_p_cov));
}

void EKF::predict_state() {
    predicted_state[0] = (A[0][0] * current_state[0]) + (A[0][1] * current_state[1]) + (A[0][2] * current_state[2]);
    predicted_state[1] = (A[1][0] * current_state[0]) + (A[1][1] * current_state[1]) + (A[1][2] * current_state[2]);
    predicted_state[2] = (A[2][0] * current_state[0]) + (A[2][1] * current_state[1]) + (A[2][2] * current_state[2]);
}

void EKF::predict_p_cov() {
    double B[3][3];
    B[0][0] = (current_p_cov[0][0] * A[0][0]) + (current_p_cov[0][1] * A[0][1]) + (current_p_cov[0][2] * A[0][2]);
    B[0][1] = (current_p_cov[0][0] * A[1][0]) + (current_p_cov[0][1] * A[1][1]) + (current_p_cov[0][2] * A[1][2]);
    B[0][2] = (current_p_cov[0][0] * A[2][0]) + (current_p_cov[0][1] * A[2][1]) + (current_p_cov[0][2] * A[2][2]);
    B[1][0] = (current_p_cov[1][0] * A[0][0]) + (current_p_cov[1][1] * A[0][1]) + (current_p_cov[1][2] * A[0][2]);
    B[1][1] = (current_p_cov[1][0] * A[1][0]) + (current_p_cov[1][1] * A[1][1]) + (current_p_cov[1][2] * A[1][2]);
    B[1][2] = (current_p_cov[1][0] * A[2][0]) + (current_p_cov[1][1] * A[2][1]) + (current_p_cov[1][2] * A[2][2]);
    B[2][0] = (current_p_cov[2][0] * A[0][0]) + (current_p_cov[2][1] * A[0][1]) + (current_p_cov[2][2] * A[0][2]);
    B[2][1] = (current_p_cov[2][0] * A[1][0]) + (current_p_cov[2][1] * A[1][1]) + (current_p_cov[2][2] * A[1][2]);
    B[2][2] = (current_p_cov[2][0] * A[2][0]) + (current_p_cov[2][1] * A[2][1]) + (current_p_cov[2][2] * A[2][2]);

    predicted_p_cov[0][0] = (A[0][0] * B[0][0]) + (A[0][1] * B[1][0]) + (A[0][2] * B[2][0]) + Q[0][0];
    predicted_p_cov[0][1] = (A[0][0] * B[0][1]) + (A[0][1] * B[1][1]) + (A[0][2] * B[2][1]) + Q[0][1];
    predicted_p_cov[0][2] = (A[0][0] * B[0][2]) + (A[0][1] * B[1][2]) + (A[0][2] * B[2][2]) + Q[0][2];
    predicted_p_cov[1][0] = (A[1][0] * B[0][0]) + (A[1][1] * B[1][0]) + (A[1][2] * B[2][0]) + Q[1][0];
    predicted_p_cov[1][1] = (A[1][0] * B[0][1]) + (A[1][1] * B[1][1]) + (A[1][2] * B[2][1]) + Q[1][1];
    predicted_p_cov[1][2] = (A[1][0] * B[0][2]) + (A[1][1] * B[1][2]) + (A[1][2] * B[2][2]) + Q[1][2];
    predicted_p_cov[2][0] = (A[2][0] * B[0][0]) + (A[2][1] * B[1][0]) + (A[2][2] * B[2][0]) + Q[2][0];
    predicted_p_cov[2][1] = (A[2][0] * B[0][1]) + (A[2][1] * B[1][1]) + (A[2][2] * B[2][1]) + Q[2][1];
    predicted_p_cov[2][2] = (A[2][0] * B[0][2]) + (A[2][1] * B[1][2]) + (A[2][2] * B[2][2]) + Q[2][2];
}

void EKF::update_gain() {
    double B[3][2], C[2][2], D[2][2];

    // pH'
    B[0][0] = (predicted_p_cov[0][0] * H[0][0]) + (predicted_p_cov[0][1] * H[0][1]) + (predicted_p_cov[0][2] * H[0][2]);
    B[0][1] = (predicted_p_cov[0][0] * H[1][0]) + (predicted_p_cov[0][1] * H[1][1]) + (predicted_p_cov[0][2] * H[1][2]);

    B[1][0] = (predicted_p_cov[1][0] * H[0][0]) + (predicted_p_cov[1][1] * H[0][1]) + (predicted_p_cov[1][2] * H[0][2]);
    B[1][1] = (predicted_p_cov[1][0] * H[1][0]) + (predicted_p_cov[1][1] * H[1][1]) + (predicted_p_cov[1][2] * H[1][2]);

    B[2][0] = (predicted_p_cov[2][0] * H[0][0]) + (predicted_p_cov[2][1] * H[0][1]) + (predicted_p_cov[2][2] * H[0][2]);
    B[2][1] = (predicted_p_cov[2][0] * H[1][0]) + (predicted_p_cov[2][1] * H[1][1]) + (predicted_p_cov[2][2] * H[1][2]);

    // H(pH') + R
    C[0][0] = (H[0][0] * B[0][0]) + (H[0][1] * B[1][0]) + (H[0][2] * B[2][0]) + R[0][0];
    C[0][1] = (H[0][0] * B[0][1]) + (H[0][1] * B[1][1]) + (H[0][2] * B[2][1]) + R[0][1];
    C[1][0] = (H[1][0] * B[0][0]) + (H[1][1] * B[1][0]) + (H[1][2] * B[2][0]) + R[1][0];
    C[1][1] = (H[1][0] * B[0][1]) + (H[1][1] * B[1][1]) + (H[1][2] * B[2][1]) + R[1][1];

    // (HpH' + R)^-1
    double det = (C[0][0] * C[1][1]) - (C[0][1] * C[1][0]);
    D[0][0] = C[1][1] / det;
    D[0][1] = -1 * C[0][1] / det;
    D[1][0] = -1 * C[1][0] / det;
    D[1][1] = C[0][0] / det;

    // pH' * (HpH' =)
    EKF_gain[0][0] = (B[0][0] * D[0][0]) + (B[0][1] * D[1][0]);
    EKF_gain[0][1] = (B[0][0] * D[0][1]) + (B[0][1] * D[1][1]);

    EKF_gain[1][0] = (B[1][0] * D[0][0]) + (B[1][1] * D[1][0]);
    EKF_gain[1][1] = (B[1][0] * D[0][1]) + (B[1][1] * D[1][1]);

    EKF_gain[2][0] = (B[2][0] * D[0][0]) + (B[2][1] * D[1][0]);
    EKF_gain[2][1] = (B[2][0] * D[0][1]) + (B[2][1] * D[1][1]);
}

void EKF::adjust_state() {
    double B[2];
    B[0] = measurement[0] - ((H[0][0] * predicted_state[0]) + (H[0][1] * predicted_state[1]) + (H[0][2] * predicted_state[2]));
    B[1] = measurement[1] - ((H[1][0] * predicted_state[0]) + (H[1][1] * predicted_state[1]) + (H[1][2] * predicted_state[2]));

    adjusted_state[0] = predicted_state[0] + (EKF_gain[0][0] * B[0]) + (EKF_gain[0][1] * B[1]);
    adjusted_state[1] = predicted_state[1] + (EKF_gain[1][0] * B[0]) + (EKF_gain[1][1] * B[1]);
    adjusted_state[2] = predicted_state[2] + (EKF_gain[2][0] * B[0]) + (EKF_gain[2][1] * B[1]);
}

void EKF::adjust_p_cov() {
    double B[2][3];
    B[0][0] = (H[0][0] * predicted_p_cov[0][0]) + (H[0][1] * predicted_p_cov[1][0]) + (H[0][2] * predicted_p_cov[2][0]);
    B[0][1] = (H[0][0] * predicted_p_cov[0][1]) + (H[0][1] * predicted_p_cov[1][1]) + (H[0][2] * predicted_p_cov[2][1]);
    B[0][2] = (H[0][0] * predicted_p_cov[0][2]) + (H[0][1] * predicted_p_cov[1][2]) + (H[0][2] * predicted_p_cov[2][2]);

    B[1][0] = (H[1][0] * predicted_p_cov[0][0]) + (H[1][1] * predicted_p_cov[1][0]) + (H[1][2] * predicted_p_cov[2][0]);
    B[1][1] = (H[1][0] * predicted_p_cov[0][1]) + (H[1][1] * predicted_p_cov[1][1]) + (H[1][2] * predicted_p_cov[2][1]);
    B[1][2] = (H[1][0] * predicted_p_cov[0][2]) + (H[1][1] * predicted_p_cov[1][2]) + (H[1][2] * predicted_p_cov[2][2]);

    adjusted_p_cov[0][0] = predicted_p_cov[0][0] - ((EKF_gain[0][0] * B[0][0]) + (EKF_gain[0][1] * B[1][0]));
    adjusted_p_cov[0][1] = predicted_p_cov[0][1] - ((EKF_gain[0][0] * B[0][1]) + (EKF_gain[0][1] * B[1][1]));
    adjusted_p_cov[0][2] = predicted_p_cov[0][2] - ((EKF_gain[0][0] * B[0][2]) + (EKF_gain[0][1] * B[1][2]));

    adjusted_p_cov[1][0] = predicted_p_cov[1][0] - ((EKF_gain[1][0] * B[0][0]) + (EKF_gain[1][1] * B[1][0]));
    adjusted_p_cov[1][1] = predicted_p_cov[1][1] - ((EKF_gain[1][0] * B[0][1]) + (EKF_gain[1][1] * B[1][1]));
    adjusted_p_cov[1][2] = predicted_p_cov[1][2] - ((EKF_gain[1][0] * B[0][2]) + (EKF_gain[1][1] * B[1][2]));

    adjusted_p_cov[2][0] = predicted_p_cov[2][0] - ((EKF_gain[2][0] * B[0][0]) + (EKF_gain[2][1] * B[1][0]));
    adjusted_p_cov[2][1] = predicted_p_cov[2][1] - ((EKF_gain[2][0] * B[0][1]) + (EKF_gain[2][1] * B[1][1]));
    adjusted_p_cov[2][2] = predicted_p_cov[2][2] - ((EKF_gain[2][0] * B[0][2]) + (EKF_gain[2][1] * B[1][2]));
}

double EKF::getFilteredAltitude() {
    return current_state[0]; // Altitude
}

double EKF::Ay_filtered() {
    return current_state[2]; // Ay
}