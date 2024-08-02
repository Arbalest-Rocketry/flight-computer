#ifndef ROCKET_STAGES_H
#define ROCKET_STAGES_H

#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include <RH_RF95.h>
#include <SD.h>
#include "apogee.h"

// Pin definitions
extern const int 
pyrS1droguechute,pyrS1mainchute,pyrS12sep,pyroIgniteS2,pyrS2droguechute,pyrS2mainchute;

extern bool isLowPowerModeEntered;
extern ApogeeDetector detector;
extern Adafruit_BNO055 bno;
extern imu::Vector<3> accel, euler; 
extern RH_RF95 rf95;

// Function declarations for rocket stages logic
bool detectLaunch();
void deployPyro(int pin, const char* message);
void deployS1drogue();
void deployS1main();
void separatestages();
void igniteupperstagemotors();
void deployS2drogue();
void deployS2main();
bool detectBurnout();
bool detectLanding(Adafruit_BMP280 &bmp);
void enterLowPowerMode(void (*logData)(), void (*transmitData)());
bool detectApogee();

#endif /* ROCKET_STAGES_H */