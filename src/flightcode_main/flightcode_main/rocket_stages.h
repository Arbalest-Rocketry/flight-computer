#ifndef ROCKET_STAGES_H
#define ROCKET_STAGES_H

#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include "apogee.h"
#include <SD.h>

// Define the necessary pin numbers as extern
extern const int pyro1, pyro2, pyro_drogue, pyro_main;
extern bool isLowPowerModeEntered;
extern int state; 

// Declare the BNO055 object as external
extern Adafruit_BNO055 bno;
extern Adafruit_BMP280 bmp;

// Function declarations for rocket stages logic
bool preLaunchChecksPassed();
bool detectLaunch();
bool detectBurnout();
void deployChute();
bool is_apogee_reached(ApogeeDetector& detector);
bool detectLanding(const Adafruit_BMP280& bmp);
void lowpowermode(void (*sdwrite)(), void (*transmitData)());
void transmitData();
String zeropad(int num);
void teensysdwrite(const String& msg);
void sdwrite();

//TODO
void deployFirstStagePyros();
void separateStages();
void lightUpperStageMotor();
void deploySecondStageDroguePyros(ApogeeDetector &detector, bool &apogeeReached);
void deployMainParachutePyros(bool &apogeeReached, bool &mainChuteDeployed);

#endif /* ROCKET_STAGES_H */