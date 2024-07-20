#include <RH_RF95.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>

#define RFM95_CS 1
#define RFM95_INT 8
#define RFM95_RST 34
#define RF95_FREQ 915.0
#define LED_PIN 13

RH_RF95 rf95(RFM95_CS, RFM95_INT);
Adafruit_BNO055 bno;

struct euler_t {
  float yaw;
  float pitch;
  float roll;
};

void eulerToQuaternion(float yaw, float pitch, float roll, float* qr, float* qi, float* qj, float* qk) {
  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);

  *qr = cy * cp * cr + sy * sp * sr;
  *qi = cy * cp * sr - sy * sp * cr;
  *qj = sy * cp * sr + cy * sp * cr;
  *qk = sy * cp * cr - cy * sp * sr;
}

void setup() {
  pinMode(RFM95_RST, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
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

  // Initialize BNO055 sensor
  if (!bno.begin()) {
    digitalWrite(LED_PIN, HIGH); // Turn on LED if initialization fails
    while (true);
  }

  // Set the frequency to 915MHz
  if (!rf95.setFrequency(RF95_FREQ)) {
    digitalWrite(LED_PIN, HIGH); // Turn on LED if setting frequency fails
    while (true);
  }

  rf95.setTxPower(23, false);
  digitalWrite(LED_PIN, LOW); // Turn off LED if everything initializes successfully
}

void loop() {
  sensors_event_t orientationData, accelerometerData, magnetometerData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);

  float qr, qi, qj, qk;
  eulerToQuaternion(orientationData.orientation.x, orientationData.orientation.y, orientationData.orientation.z, &qr, &qi, &qj, &qk);

  DynamicJsonDocument jsonDoc(256);

  // Create buffers to hold the formatted strings
  char buffer[10];

  dtostrf(qr, 1, 2, buffer);
  jsonDoc["quaternion_w"] = buffer;
  dtostrf(qi, 1, 2, buffer);
  jsonDoc["quaternion_x"] = buffer;
  dtostrf(qj, 1, 2, buffer);
  jsonDoc["quaternion_y"] = buffer;
  dtostrf(qk, 1, 2, buffer);
  jsonDoc["quaternion_z"] = buffer;

  String jsonString;
  serializeJson(jsonDoc, jsonString);

  rf95.send((uint8_t*)jsonString.c_str(), jsonString.length());
  rf95.waitPacketSent();

  // Optional: Toggle LED to indicate successful transmission
  digitalWrite(LED_PIN, HIGH);
  delay(100); // Keep the LED on for 100 milliseconds
  digitalWrite(LED_PIN, LOW);
  delay(100); // Delay between transmissions (adjust as needed)
}