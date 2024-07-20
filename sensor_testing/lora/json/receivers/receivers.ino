#include <RH_RF95.h>
#include <SPI.h>

#define RFM95_CS 1 // Define chip select pin for LoRa module
#define RFM95_INT 8 // Define interrupt pin for LoRa module
#define RFM95_RST 34 // Define reset pin for LoRa module
#define RF95_FREQ 915.0 // Define frequency
#define LED_PIN 13 // Define LED pin for status indication

RH_RF95 rf95(RFM95_CS, RFM95_INT); // Initialize the LoRa module with the specified pins

void setup() {
  pinMode(RFM95_RST, OUTPUT); // Set reset pin as output
  pinMode(LED_PIN, OUTPUT); // Set LED pin as output
  digitalWrite(RFM95_RST, HIGH); // Keep reset pin high initially
  digitalWrite(LED_PIN, LOW); // Turn off LED initially

  // Reset the LoRa module
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Initialize LoRa module
  if (!rf95.init()) {
    digitalWrite(LED_PIN, HIGH); // Turn on LED if initialization fails
    while (true);
  }

  // Set the frequency to 915MHz
  if (!rf95.setFrequency(RF95_FREQ)) {
    digitalWrite(LED_PIN, HIGH); // Turn on LED if setting frequency fails
    while (true);
  }

  digitalWrite(LED_PIN, LOW); // Turn off LED if initialization succeeded
}

void loop() {
  // Check if data is available to receive
  if (rf95.available()) {
    // Buffer to hold received data
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf); // Length of received data

    // Read received data into buffer
    if (rf95.recv(buf, &len)) {
      // Blink LED to indicate successful reception
      digitalWrite(LED_PIN, HIGH);
      delay(100); // Keep the LED on for 100 milliseconds
      digitalWrite(LED_PIN, LOW);

      // Print received data
      Serial.print("Received data: ");
      Serial.write(buf, len);
      Serial.println();
    } else {
      // Optional: handle failed message reception
      // Serial.println("Failed to receive message");
    }
  }
}
