# Apogee Detection Implementation

## Introduction

In rocketry, detecting the apogee — the highest point in the rocket's flight — is crucial for triggering key events such as parachute deployment. The algorithm is designed to reliably determine when the rocket reaches its peak altitude, ensuring timely and accurate event triggers for a safe and successful flight.

## Implementation Details

### Rolling Window

A rolling window is used to keep track of the most recent altitude measurements. This helps in smoothing out the data and making more reliable apogee detection decisions.

#### Functions:

- **Initialization**:
  - `init_rolling_window(RollingWindow *rw, double *pBackingArray, size_t capacity)`: Initializes the rolling window with the specified capacity.

- **Add Data Point**:
  - `add_data_point_rolling_window(RollingWindow *rw, double dataPoint)`: Adds a new data point to the rolling window. If the window is full, the oldest data point is removed.

- **Get Latest Data Point**:
  - `get_latest_datapoint_rolling_window(RollingWindow *rw)`: Returns the most recent data point in the rolling window.

- **Get Earliest Data Point**:
  - `get_earliest_datapoint_rolling_window(RollingWindow *rw)`: Returns the oldest data point in the rolling window.

### Apogee Detection

The apogee detection algorithm tracks altitude changes to determine when the rocket has reached its highest point. It uses a threshold of consecutive decreases in altitude to confirm apogee.

#### Functions:

- **Initialization**:
  - `init_apogee_detector(ApogeeDetector *detector, double *backing_array, size_t capacity)`: Initializes the apogee detector with a rolling window for altitude measurements.

- **Update Detector**:
  - `update_apogee_detector(ApogeeDetector *detector, double current_altitude)`: Updates the apogee detector with the current altitude. If the altitude decreases consecutively for a defined threshold, apogee is detected.

- **Check Apogee**:
  - `is_apogee_reached(ApogeeDetector *detector)`: Returns whether apogee has been reached.

### Usage

1. **Initialize the Sensor**:
   - Initialize the BMP280 sensor for altitude measurements.
   
2. **Initialize the Apogee Detector**:
   - Create an `ApogeeDetector` object and initialize it with a backing array and a capacity for the rolling window.

3. **Update the Apogee Detector**:
   - In a loop, read the current altitude from the BMP280 sensor and update the apogee detector with this value.
   
4. **Check for Apogee**:
   - Check if apogee has been reached using the `is_apogee_reached` function and perform any additional logic needed when apogee is detected.

### Why Rolling Window?

Using a rolling window helps in smoothing out altitude readings and makes the apogee detection more reliable by filtering out short-term fluctuations and noise. This approach ensures that the apogee detection is based on consistent decreases in altitude over a period of time, making the detection more robust.