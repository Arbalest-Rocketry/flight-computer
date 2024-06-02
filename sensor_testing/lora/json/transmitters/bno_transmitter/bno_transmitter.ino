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
  Serial.begin(9600);
  while (!Serial);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  if (!rf95.init()) {
    Serial.println("LoRa initialization failed. Check your connections.");
    while (true);
  }

  if (!bno.begin()) {
    Serial.println("Could not find a valid BNO055 sensor, check wiring!");
    while (true);
  }
  
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);
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
  
  /*
  dtostrf(accelerometerData.acceleration.x, 1, 2, buffer);
  jsonDoc["acceleration_x"] = buffer;
  dtostrf(accelerometerData.acceleration.y, 1, 2, buffer);
  jsonDoc["acceleration_y"] = buffer;
  dtostrf(accelerometerData.acceleration.z, 1, 2, buffer);
  jsonDoc["acceleration_z"] = buffer;

  dtostrf(magnetometerData.magnetic.x, 1, 2, buffer);
  jsonDoc["magnetic_field_x"] = buffer;
  dtostrf(magnetometerData.magnetic.y, 1, 2, buffer);
  jsonDoc["magnetic_field_y"] = buffer;
  dtostrf(magnetometerData.magnetic.z, 1, 2, buffer);
  jsonDoc["magnetic_field_z"] = buffer;
  */
  
  String jsonString;
  serializeJson(jsonDoc, jsonString);

  rf95.send((uint8_t*)jsonString.c_str(), jsonString.length());
  rf95.waitPacketSent();

  Serial.println("Quaternion data sent:");
  Serial.println(jsonString);
}