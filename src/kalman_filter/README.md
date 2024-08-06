# Extended Kalman Filter (EKF) Implementation

## Introduction

The Extended Kalman Filter (EKF) designed to filter and estimate the altitude using data from a barometer sensor (bmp280) and acceleration data from an IMU (bno055).

## Implementation Details

### State Vector

The state vector in this EKF implementation consists of:
- Altitude
- Velocity
- Acceleration

### Matrices

Several key matrices are used in the EKF algorithm:

- `A`: State transition matrix, which models how the state evolves over time.
- `Q`: Model noise covariance matrix, which represents the uncertainty in the model.
- `H`: Measurement matrix, which maps the state to the measurement space.
- `R`: Measurement noise covariance matrix, which represents the uncertainty in the measurements.
- `current_p_cov`: Process covariance matrix, which represents the uncertainty in the state estimation.

### EKF Steps

The EKF algorithm follows these steps during each update:

1. **Predict State**: 
   - The state vector is updated based on the state transition matrix `A` and the time elapsed (`dt`).

2. **Predict Process Covariance**:
   - The process covariance matrix is updated using the state transition matrix `A` and the model noise covariance matrix `Q`.

3. **Update Gain**:
   - The Kalman gain is calculated using the predicted process covariance and the measurement noise covariance matrix `R`.

4. **Adjust State**:
   - The state vector is updated using the measurement and the Kalman gain to minimize the estimation error.

5. **Adjust Process Covariance**:
   - The process covariance matrix is updated based on the Kalman gain to reflect the reduced uncertainty after incorporating the measurement.

### Initialization

The EKF is initialized with predefined values for the matrices and the initial state. The `begin` method sets the initial state based on the initial barometer altitude and acceleration.

### Usage

To use the EKF, create an EKF object and initialize it with the initial conditions. Then, in a loop, read the sensor data (barometer altitude and IMU acceleration) and update the EKF with this data. The filtered altitude and acceleration can be accessed from the state vector.

### Filtering Across Axes

The EKF implementation is capable of filtering data across all three axes (x, y, and z) by applying the same algorithm. The same state transition and update steps are used for each axis independently. This allows for a comprehensive state estimation that includes not only the altitude but also the velocity and acceleration in all three spatial dimensions. Filtering all axes helps in accurately tracking the motion and orientation of the rocket or any other dynamic system.