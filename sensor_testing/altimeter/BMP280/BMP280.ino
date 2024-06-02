#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP280.h"
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP280 bmp;
int t1, t2, t_int;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Adafruit BMP280 test");


    if (!bmp.begin(0x77)) { /* Failed to initialize BMP280*/ while (1); } // Halt the program
     bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_125); /* Standby time. */

}

 
void loop() {
  

  for(int i=0; i < 100; i++){
    t1 = millis();
    bmp.readTemperature();
    
    t2 = millis();
    t_int = t2 - t1;
    Serial.println(bmp.readPressure());
    //Serial.println(t_int);
    //delay(200);
  }

}

  
