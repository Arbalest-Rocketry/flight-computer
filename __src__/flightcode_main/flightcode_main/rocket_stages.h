#ifndef ROCKET_STAGES_H
#define ROCKET_STAGES_H

#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include "apogee.h"

// Define the necessary pin numbers as extern
extern const int pyro1Pin;
extern const int pyro2Pin;
extern const int pyroDroguePin;
extern const int pyroMainPin;

// Declare the BNO055 object as external
extern Adafruit_BNO055 bno;

// Function declarations for rocket stages logic
bool detectLaunch(Adafruit_BMP280 &bmp);
void deployFirstStagePyros();
bool detectFirstStageBurnout(Adafruit_BMP280 &bmp);
void separateStages();
void lightUpperStageMotor();
void deploySecondStageDroguePyros(ApogeeDetector &detector, Adafruit_BMP280 &bmp, bool &apogeeReached);
void deployMainParachutePyros(bool &apogeeReached, bool &mainChuteDeployed, Adafruit_BMP280 &bmp);
bool detectLanding(Adafruit_BMP280 &bmp);
void enterLowPowerMode(void (*logData)(), void (*transmitData)());

#endif /* ROCKET_STAGES_H */