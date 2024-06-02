#include <RH_RF95.h> // Include the LoRa library
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BMP280.h> // Include the BMP280 sensor library
#include <ArduinoJson.h> // Include the ArduinoJson library for JSON manipulation

#define RFM95_CS 1 // Define the chip select pin for LoRa module
#define RFM95_INT 8 // Define the interrupt pin for LoRa module
#define RFM95_RST 34 // Define the reset pin for LoRa module
#define RF95_FREQ 915.0
RH_RF95 rf95(RFM95_CS, RFM95_INT); // Initialize the LoRa module with the specified pins

Adafruit_BMP280 bmp; // Create BMP280 sensor object

void setup() {
  Serial.begin(9600); // Initialize serial communication
  while (!Serial);

  pinMode(RFM95_RST, OUTPUT); // Set reset pin as output
  digitalWrite(RFM95_RST, HIGH); // Keep reset pin high initially

  // Initialize LoRa module
  if (!rf95.init()) {
    Serial.println("LoRa initialization failed. Check your connections.");
    while (true);
  }

  // Initialize BMP280 sensor
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (true);
  }
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);
}

void loop() {
  // Reading BMP280 sensor data
  float temperature = bmp.readTemperature(); // Read temperature
  float pressure = bmp.readPressure() / 100.0F; // Read pressure in hPa
  float altitude = bmp.readAltitude(); // Read altitude in meters
  
  // Encoding Data as JSON
  StaticJsonDocument<200> jsonDoc; // Create JSON document object
  jsonDoc["temperature"] = temperature; // Add temperature data to JSON object
  jsonDoc["pressure"] = pressure; // Add pressure data to JSON object
  jsonDoc["altitude"] = altitude; // Add altitude data to JSON object
  
  // Serialize JSON to string
  String jsonString;
  serializeJson(jsonDoc, jsonString); // Serialize JSON to string

  // Send JSON data over LoRa
  rf95.send((uint8_t*)jsonString.c_str(), jsonString.length()); // Send the JSON string over LoRa
  rf95.waitPacketSent(); // Wait for the packet to be sent

  // Print JSON string
  Serial.println("JSON data sent:");
  Serial.println(jsonString);
}
