#ifndef ROCKET_STAGES_H
#define ROCKET_STAGES_H

#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include "apogee.h"
#include <SD.h>
#include "quaternion.h"

enum RocketState {
    PRE_LAUNCH,
    LAUNCH_DETECTED,
    FIRST_STAGE_BURNOUT,
    STAGE_SEPARATION,
    UPPER_STAGE_IGNITION,
    APOGEE,
    MAIN_CHUTE_DEPLOYMENT,
    LANDING_DETECTED,
    LOW_POWER_MODE
};

extern RocketState currentState;

// Pin definitions
extern const int 
pyrS1droguechute,pyrS1mainchute,pyrS12sep,pyroIgniteS2,pyrS2droguechute,pyrS2mainchute;

extern bool isLowPowerModeEntered;
extern ApogeeDetector detector;
extern Adafruit_BNO055 bno; extern Adafruit_BMP280 bmp;
extern imu::Vector<3> accel;
extern imu::Vector<3> euler;
extern int state; 
extern const int ledblu, ledgrn, ledred, teensyled;
extern EKF ekf;
extern Quaternion q;
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
void lowpowermode(void (*sdwrite)(), void (*transmitData)());
void transmitData();
String zeropad(int num);
void teensysdwrite(const String& msg);
void sdwrite();

#endif /* ROCKET_STAGES_H */