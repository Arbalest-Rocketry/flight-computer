#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>

class Kalman {
public:
    Kalman(Adafruit_BMP280 *bmp, Adafruit_BNO055 *bno);
    void begin();
    void update();
    double getFilteredAltitude() const;
    double getFilteredVelocity() const;
    double getRawAltitude() const;
    double getRawAcceleration() const;

private:
    Adafruit_BMP280 *bmp;
    Adafruit_BNO055 *bno;
    double filteredAltitude;
    double filteredVelocity;
    double current_state[6];
    double predicted_state[6];
    double adjusted_state[6];
    double A[6][6];
    double Q[6][6];
    double R[2][2];
    double H[2][6];
    double current_p_cov[6][6];
    double predicted_p_cov[6][6];
    double adjusted_p_cov[6][6];
    double kalman_gain[6][2];
    unsigned long old_time;
    unsigned long curr_time;
    double dt;

    void init();
    void predict_state();
    void predict_p_cov();
    void update_gain();
    void adjust_state();
    void adjust_p_cov();
};

#endif