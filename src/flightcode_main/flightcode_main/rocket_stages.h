#ifndef ROCKET_STAGES_H
#define ROCKET_STAGES_H

#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include <RH_RF95.h>
#include <SD.h>
#include "apogee.h"
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
extern const char* stateNames[];

// Pin definitions
extern const int 
pyrS1droguechute,pyrS1mainchute,pyrS12sep,pyroIgniteS2,pyrS2droguechute,pyrS2mainchute;

extern const unsigned long boosterBurpTime;
extern bool boosterBurpDetected, boosterBurnoutCheck;
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
void cutoffpower();
void tiltLock();
bool detectLanding(Adafruit_BMP280 &bmp);
bool detectApogee();
void lowpowermode(void (*sdwrite)(), void (*transmitData)());
void transmitData();
void sdwrite();
void writeDataToSD(File &dataFile, float temperature, float altitude, float filteredAltitude, float accelY, float filteredAy, const char* state);
void logData(const char* filename);
bool ensureFileExists(const char* filename);
String generateNewFileName(const char* baseName);
void methodOn();
void methodOff();
void axisRemapping();

#endif /* ROCKET_STAGES_H */