#include <SPI.h>
#include <SD.h>

const int chipSelect = 10; // Change this pin if necessary

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("Initializing SD card...");

  // Check if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // Don't do anything more:
    while (1);
  }
  Serial.println("Card initialized.");

  // Open the file for reading:
  File dataFile = SD.open("flightlog001.txt");

  // If the file is available, read it:
  if (dataFile) {
    while (dataFile.available()) {
      Serial.write(dataFile.read());
    }
    // Close the file:
    dataFile.close();
  } else {
    // If the file didn't open, print an error:
    Serial.println("Error opening file");
  }
}

void loop() {
  // Nothing happens after setup
}
