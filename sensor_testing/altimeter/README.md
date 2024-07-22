
# Altimeter Control and Logging

This directory contains code to interface with an altimeter using a BMP280 sensor and log altitude data to an SD card using a Teensy microcontroller.

## Purpose

To provide a method for debugging and monitoring altitude readings from a BMP280 sensor, logging these readings to an SD card. This setup is useful for applications requiring altitude data logging, such as high-altitude balloon experiments or rocketry.

## Overview of BMP280 Sensor

The BMP280 sensor is an environmental sensor from Bosch that measures temperature and barometric pressure. It is commonly used for weather sensing and can also function as an altimeter due to the relationship between pressure and altitude.

### Key Features
- Barometric pressure measurement with ±1 $hPa$ absolute accuracy
- Temperature measurement with ±1.0°C accuracy
- Altitude measurement with ±1 meter accuracy

### How It Works
The BMP280 sensor calculates altitude based on atmospheric pressure using the barometric formula:

$h = \frac{RT}{Mg} \ln \left( \frac{P_0}{P} \right)$

where:
- $h$ is the altitude
- $R$ is the universal gas constant
- $T$ is the temperature in Kelvin
- $M$ is the molar mass of Earth's air
- $g$ is the acceleration due to gravity
- $P_0$ is the sea-level standard atmospheric pressure
- $P$ is the atmospheric pressure

## Key Components

### BMP280 Sensor Initialization

The BMP280 sensor is initialized and configured to read altitude data. If the sensor or SD card initialization fails, the system will halt.

```cpp
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SD.h>

const int chipSelect = 10;
const int sdPower = LED_BUILTIN;

Adafruit_BMP280 bmp; // Create BMP280 object
File myFile; // File object for SD card

void setup() {
  pinMode(sdPower, OUTPUT);
  Wire.begin();
  Serial.begin(9600);

  if (!bmp.begin()) { // Start BMP280 with address 0x76 (Adafruit default)
    Serial.println("BMP280 not detected. Check wiring or address.");
    while (1); // Stop program
  }

  if (!SD.begin(chipSelect)) { // Initialize SD card
    Serial.println("SD card initialization failed.");
    while (1); // Stop program
  }
}
```

### Reading and Logging Altitude Data

Altitude data is read from the BMP280 sensor and logged to the SD card. If there are any errors in reading the altitude, an error message is printed.

```cpp
void loop() {
  float altitude = bmp.readAltitude(1013.25); // Sea-level pressure in hPa
  if (!isnan(altitude)) {
    Serial.print("Altitude(m): ");
    Serial.println(altitude, 2); // Print altitude with 2 decimal places
    storeState(altitude); // Store current state in log file
  } else {
    Serial.println("Error reading altitude."); // Print error message if altitude reading is invalid
  }
  delay(1000); // Delay for 1 second
}
```

### Storing Data to SD Card

Data is stored to an SD card for later retrieval. This involves opening the log file, writing the data, and closing the file.

```cpp
void storeState(float altitude) {
  digitalWrite(sdPower, HIGH); // Activate SD card power
  myFile = SD.open("log.txt", FILE_WRITE); // Open log file for writing

  if (myFile) { // If file opened successfully
    myFile.print("Altitude(m): ");
    myFile.println(altitude); // Write altitude to log file
    myFile.close(); // Close log file
  } else {
    Serial.println("Error opening log file."); // Print error message if file open failed
  }

  digitalWrite(sdPower, LOW); // Deactivate SD card power
}
```

## Example Serial Monitor Output

```
Altitude(m): 123.45
Altitude(m): 123.56
Altitude(m): 123.67
Error reading altitude.
Altitude(m): 123.78
```

## Setup Instructions
1. **Upload the code to your microcontroller**:
   - Open the Arduino IDE.
   - Load the `altimeter.ino` sketch.
   - Select the appropriate board and port.
   - Upload the sketch.

2. **Monitor the Serial Output**:
   - Open the Serial Monitor in the Arduino IDE.
   - Set the baud rate to 9600 to view the debug output.

---