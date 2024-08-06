# Rocket Stages

This directory contains the implementation for the rocket stages of Goose 4. The code manages different stages of the rocket's flight, including launch detection, stage separation, burnout detection, and parachute deployment.

## Overview

### `apogee.cpp` and `apogee.h`
These files manage the rolling window of altitude data and the apogee detector. The apogee detector is responsible for determining the highest point in the rocket's flight.

- **Rolling Window Functions**
  - Maintains a fixed-size buffer of the most recent altitude data points to smooth out readings.
  - Adds new data points and updates the sum of elements for efficient calculation.

- **Apogee Detector Functions**
  - Uses the rolling window to track altitude changes.
  - Determines apogee by detecting consecutive decreases in altitude, indicating that the rocket has passed its peak height.

### `rocket_stages.cpp` and `rocket_stages.h`
These files implement the state machine logic for the rocket's flight stages, using sensor data to make real-time decisions.

### Key Stages and Logic

#### 1. Launch Detection
The launch is detected based on a predefined acceleration threshold. The rocket uses the y-axis acceleration to determine if it has been launched.

**Logic:**
- Monitor the y-axis acceleration.
- Trigger launch detection if the acceleration exceeds the threshold.

```cpp
bool detectLaunch() {
    if (accel.y() > 13) {
        Serial.println("Launch detected based on acceleration.");
        return true;
    }
    return false;
}
```

#### 2. First Stage Burnout Detection
Detecting burnout involves monitoring the reduction in acceleration. When the motor stops firing, the acceleration drops below a certain threshold, indicating burnout.

**Logic:**
- Continuously check the y-axis acceleration.
- Detect burnout if the acceleration falls below the predefined threshold.

```cpp
bool detectBurnout() {
    static int stage = 1;
    float burnoutThreshold = (stage == 1) ? 2.0 : 1.5; 
    if (accel.y() <= burnoutThreshold) {
        Serial.print("Burnout detected at stage ");
        Serial.println(stage);
        stage++; 
        return true;
    }
    return false;
}
```

#### 3. Stage Separation and Upper Stage Ignition
After detecting burnout, the rocket separates the stages and ignites the upper stage motor to continue the ascent.

**Logic:**
- Trigger pyrotechnics to separate the stages.
- Ignite the upper stage motor.

```cpp
void separatestages() {
    deployPyro(pyrS12sep, "Separating stages...");
}

void igniteupperstagemotors() {
    deployPyro(pyroIgniteS2, "Igniting upper stage motors ...");
}
```

#### 4. Apogee Detection
Apogee detection determines the highest point in the rocket's flight using altitude data. It relies on detecting consecutive decreases in altitude.

**Logic:**
- Monitor altitude readings using the rolling window.
- Detect apogee if a series of consecutive decreases in altitude is observed.

```cpp
bool detectApogee() {
    return is_apogee_reached(&detector);
}
```

#### 5. Parachute Deployment
After detecting apogee, the rocket deploys the drogue chute to slow its descent. Later, the main chute is deployed at a lower altitude to further slow the descent for a safe landing.

**Logic:**
- Deploy the drogue chute after detecting apogee.
- Deploy the main chute at a predetermined altitude.

```cpp
void deployS2drogue() {
    deployPyro(pyrS2droguechute, "Deploying second stage drogue pyros");
}

void deployS2main() {
    deployPyro(pyrS2mainchute, "Deploying second stage main pyros");
}
```

#### 6. Landing Detection
Landing is detected by monitoring if the altitude remains constant for a period, indicating that the rocket has landed.

**Logic:**
- Continuously monitor altitude.
- Detect landing if the altitude remains within a small range for a specified time.

```cpp
bool detectLanding(Adafruit_BMP280 &bmp) {
    static double lastAltitude = 0;
    double currentAltitude = bmp.readAltitude(1013.25);
    static unsigned long landedTime = millis();

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
```

#### 7. Low Power Mode
After landing, the rocket enters a low power mode to conserve battery while continuing to transmit and log data periodically.

**Logic:**
- Reduce power consumption by turning off unnecessary components.
- Continue to transmit and log data at intervals.

```cpp
void enterLowPowerMode(void (*logData)(), void (*transmitData)()) {
    isLowPowerModeEntered = true;
    Serial.println("Entering low power mode");

    // Set BNO055 to low power mode
    bno.setMode(OPERATION_MODE_CONFIG);
    delay(25);
    // Use the Adafruit_BNO055 method to set power mode
    bno.enterSuspendMode();  // Assuming this sets the device to a low power state
    delay(25);
    bno.setMode(OPERATION_MODE_NDOF);
    delay(25);
    bno.setExtCrystalUse(false); // Example of setting BNO055 to low power mode
    Serial.println("BNO055 set to low power mode");

    // Set BMP280 to sleep mode
    bmp.setSampling(Adafruit_BMP280::MODE_SLEEP);
    Serial.println("BMP280 set to low power mode");

    // Set LoRa (RFM9x) to sleep mode
    rf95.sleep();
    Serial.println("LoRa module set to low power mode");

    // Ensure no open files on SD card to save power
    Serial.println("Ensure SD card is not accessed to save power");

    while (true) {
        // Call provided functions to transmit and log data
        logData();
        transmitData();
        delay(30000); // Transmit data every 30 seconds
    }
}
```