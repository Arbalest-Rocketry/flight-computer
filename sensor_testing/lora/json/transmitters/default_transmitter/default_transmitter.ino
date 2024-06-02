#include <RH_RF95.h> // Include the LoRa library
#include <SPI.h>
#include <ArduinoJson.h> // Include the ArduinoJson library for JSON manipulation

#define RFM95_CS 1 // Define the chip select pin for LoRa module
#define RFM95_INT 8 // Define the interrupt pin for LoRa module
#define RFM95_RST 34 // Define the reset pin for LoRa module

RH_RF95 rf95(RFM95_CS, RFM95_INT); // Initialize the LoRa module with the specified pins

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
}

void loop() {
  // Encoding Data as JSON
  StaticJsonDocument<200> jsonDoc; // Create JSON document object
  float sensorData = 25.5; // Example sensor data
  jsonDoc["sensor_data"] = sensorData; // Add sensor data to JSON object
  
  // Serialize JSON to string
  String jsonString;
  serializeJson(jsonDoc, jsonString); // Serialize JSON to string

  // Send JSON data over LoRa
  rf95.send((uint8_t*)jsonString.c_str(), jsonString.length()); // Send the JSON string over LoRa
  rf95.waitPacketSent(); // Wait for the packet to be sent

  // Print JSON string
  Serial.println("JSON data sent:");
  Serial.println(jsonString);

  delay(5000); // Delay before next iteration
}
