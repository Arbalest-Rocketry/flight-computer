#include <RH_RF95.h> // Include the LoRa library
#include <SPI.h>
#include <ArduinoJson.h> // Include the ArduinoJson library for JSON manipulation

//#include <ArduinoWebsockets.h> // Include the ArduinoWebsockets library for websocket connecction
//

#define RFM95_CS 1 // Define the chip select pin for LoRa module
#define RFM95_INT 8 // Define the interrupt pin for LoRa module
#define RFM95_RST 34 // Define the reset pin for LoRa module

RH_RF95 rf95(RFM95_CS, RFM95_INT); // Initialize the LoRa module with the specified pins
//WebsocketsClient client;

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
//  client.connect("ws://your-server-ip:3000/");

  }

void loop() {
  // Check if data is available to receive
  if (rf95.available()) {
    // Buffer to hold received data
    uint8_t bufReceived[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(bufReceived); // Length of received data

    // Read received data into buffer
    if (rf95.recv(bufReceived, &len)) {
      // Print received data
      Serial.print("Received data: ");
      Serial.write(bufReceived, len);
      Serial.println();

      // Parse received JSON
      StaticJsonDocument<200> jsonDoc;
      DeserializationError error = deserializeJson(jsonDoc, bufReceived, len);

      // Check for parsing errors
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.c_str());
      } else {
//        // Extract sensor data from JSON
//        float sensorData = jsonDoc["sensor_data"];
//        Serial.print("Sensor data: ");
//        Serial.println(sensorData);
      }
    } else {
      Serial.println("Failed to receive message");
    }
  }
}
