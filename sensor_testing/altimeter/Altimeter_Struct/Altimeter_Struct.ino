#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SD.h>

const int chipSelect = 10;
const int sdPower = LED_BUILTIN;

Adafruit_BMP280 bmp; // Create BMP280 object
File myFile; // File object for SD card

typedef struct {
  float altitude;
  float velocity;
} State; // Define State struct with altitude and velocity

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

  State init_state = {readAltitude(), 20.0}; // Initial state with altitude and placeholder velocity

  storeState(init_state); // Store initial state in log file
}

void loop() {
  float altitude = readAltitude();
  if (!isnan(altitude)) {
    State current_state = {altitude, 20.0}; // Create current state with updated altitude and placeholder velocity

    Serial.print("Altitude(m): ");
    Serial.println(current_state.altitude, 2); // Print altitude with 2 decimal places

    storeState(current_state); // Store current state in log file
  } else {
    Serial.println("Error reading altitude."); // Print error message if altitude reading is invalid
  }

  delay(1000); // Delay for 1 second

  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'v') { // Check if the command is 'v' to view log file
      viewLogFile(); // Call function to view log file contents
    }
  }
}

float readAltitude() {
  return bmp.readAltitude(1013.25); // Read altitude from BMP280 sensor (pressure at sea level is 1013.25 hPa)
}

void storeState(State state) {
  digitalWrite(sdPower, HIGH); // Activate SD card power
  myFile = SD.open("log.txt", FILE_WRITE); // Open log file for writing

  if (myFile) { // If file opened successfully
    myFile.print("Altitude(m): ");
    myFile.println(state.altitude); // Write altitude to log file
    myFile.close(); // Close log file
  } else {
    Serial.println("Error opening log file."); // Print error message if file open failed
  }

  digitalWrite(sdPower, LOW); // Deactivate SD card power
}

void viewLogFile() {
  digitalWrite(sdPower, HIGH); // Activate SD card power
  myFile = SD.open("log.txt"); // Open log file for reading

  if (myFile) { // If file opened successfully
    Serial.println("log.txt contents:");
    while (myFile.available()) {
      Serial.write(myFile.read()); // Read and print each character from the file
    }
    myFile.close(); // Close log file
  } else {
    Serial.println("Error opening log file."); // Print error message if file open failed
  }

  digitalWrite(sdPower, LOW); // Deactivate SD card power
}
