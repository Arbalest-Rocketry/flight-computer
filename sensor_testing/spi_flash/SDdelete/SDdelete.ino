#include <SPI.h>
#include <SD.h>

// Define the chip select pin for the SD card
const int chipSelect = BUILTIN_SDCARD; // Change this if using a different SD card module

void setup() {
    Serial.begin(9600);
    while (!Serial) { ; }  // Wait for the serial port to be available

    Serial.println("Initializing SD card...");

    if (!SD.begin(chipSelect)) {
        Serial.println("Initialization failed!");
        return;
    }
    Serial.println("Initialization done.");

    // Call the delete file function
    deleteFile();
}

void loop() {
    // nothing happens after setup finishes.
}

void deleteFile() {
    if (SD.exists("flightlog001.txt")) {
        Serial.println("Deleting file flightlog001.txt...");
        if (SD.remove("flightlog001.txt")) {
            Serial.println("File flightlog001.txt deleted successfully.");
        } else {
            Serial.println("Error deleting flightlog001.txt.");
        }
    } else {
        Serial.println("File flightlog001.txt does not exist.");
    }
}