#ifndef APOGEE_H
#define APOGEE_H

#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>

#define WINDOW_SIZE 20
// Number of consecutive decreases to confirm apogee
#define DECREASE_THRESHOLD /*5*/ 12 // for LC
extern bool apogeeReached;
extern Adafruit_BMP280 bmp; // BMP280 object

typedef struct {
    double *backing_array;
    size_t capacity;
    size_t size;
    size_t front;
    double sum_of_elements;
} RollingWindow;

typedef struct {
    RollingWindow altitude_window;
    double last_altitude;
    int apogee_reached;
    int decrease_count; // Track consecutive decreases
} ApogeeDetector;

void init_rolling_window(RollingWindow *rw, double *pBackingArray, size_t capacity);
size_t mod_rolling_window(size_t index, size_t modulo);
void add_data_point_rolling_window(RollingWindow *rw, double dataPoint);
double get_latest_datapoint_rolling_window(RollingWindow *rw);
double get_earliest_datapoint_rolling_window(RollingWindow *rw);

void init_apogee_detector(ApogeeDetector *detector, double *backing_array, size_t capacity);
void update_apogee_detector(ApogeeDetector *detector, double current_altitude);
int is_apogee_reached(ApogeeDetector *detector);

#endif /* APOGEE_H */