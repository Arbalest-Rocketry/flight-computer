#include "gps_class.h"

Gps::Gps() {
  init();
}

void Gps::init() {
}

void Gps::begin_gps() {
    Wire.begin();
    Serial.println("Initializing GPS...");
    test_connection();
}

void Gps::test_connection() {
    if (!i2c_gps.begin(Wire, 400000)) {
        Serial.println("Ooops, no GPS detected ... Check your wiring or I2C ADDR!");
        gpsStruct.diagmsg = pow(2, 2);
        while (1);
    }
    delay(10);
}

void Gps::read_position() {
    while (i2c_gps.available()) {
        gps.encode(i2c_gps.read());
    }

    if (gps.time.isUpdated()) {
        read_data();
    }

    if (gps.location.age() > 100) {
        gpsStruct.lng = 3.14;
        gpsStruct.lat = 3.14;
        gpsStruct.diagmsg = pow(2, 2);
    }
}

void Gps::read_data() {
    if (gps.location.isValid()) {
        gpsStruct.lng = gps.location.lng();
        gpsStruct.lat = gps.location.lat();
        gpsStruct.diagmsg = 0;
    } else {
        gpsStruct.lng = 3.14;
        gpsStruct.lat = 3.14;
        gpsStruct.diagmsg = pow(2, 2);
    }
    
    if (gps.altitude.isValid()) {
        gpsStruct.gpsalt = gps.altitude.meters();
    } else {
        gpsStruct.diagmsg = pow(2, 2);
    }

    Serial.print("Latitude: ");
    Serial.println(gpsStruct.lat);
    Serial.print("Longitude: ");
    Serial.println(gpsStruct.lng);
    Serial.print("Altitude: ");
    Serial.println(gpsStruct.gpsalt);
}