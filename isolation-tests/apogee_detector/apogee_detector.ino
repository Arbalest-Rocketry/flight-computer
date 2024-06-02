#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define WINDOW_SIZE 20
// I will set DECREASE_THRESHOLD to a reasonable higher value during lauch
#define DECREASE_THRESHOLD 2 // Number of consecutive decreases to confirm apogee

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

Adafruit_BMP280 bmp; // bmp object
// Rolling Window Functions
void init_rolling_window(RollingWindow *rw, double *pBackingArray, size_t capacity) {
    rw->backing_array = pBackingArray;
    rw->capacity = capacity;
    rw->size = 0;
    rw->front = 0;
    rw->sum_of_elements = 0.0;
}
size_t mod_rolling_window(size_t index, size_t modulo) {
    return (index % modulo + modulo) % modulo;
}
void add_data_point_rolling_window(RollingWindow *rw, double dataPoint) {
    if (rw->size >= rw->capacity) {
        rw->sum_of_elements -= rw->backing_array[mod_rolling_window(rw->front, rw->capacity)];
        rw->front = mod_rolling_window(rw->front + 1, rw->capacity);
        rw->size--;
    } 
    size_t back_insertion_idx = mod_rolling_window(rw->front + rw->size, rw->capacity);
    rw->backing_array[back_insertion_idx] = dataPoint;
    rw->sum_of_elements += dataPoint;
    rw->size++;
}
double get_latest_datapoint_rolling_window(RollingWindow *rw) {
    return rw->backing_array[mod_rolling_window(rw->front + rw->size - 1, rw->capacity)];
}
double get_earliest_datapoint_rolling_window(RollingWindow *rw) {
    return rw->backing_array[mod_rolling_window(rw->front, rw->capacity)];
}

// Apogee Detector Functions
void init_apogee_detector(ApogeeDetector *detector, double *backing_array, size_t capacity) {
    init_rolling_window(&detector->altitude_window, backing_array, capacity);
    detector->last_altitude = -1.0;
    detector->apogee_reached = 0;
    detector->decrease_count = 0;
}

void update_apogee_detector(ApogeeDetector *detector, double current_altitude) {
    add_data_point_rolling_window(&detector->altitude_window, current_altitude);

    // Check if apogee has already been reached
    if (detector->apogee_reached) {
        return; // Exit the function if apogee has already been reached
    }
    if (detector->last_altitude >= 0 && current_altitude < detector->last_altitude) {
        detector->decrease_count++;
    } else {
        detector->decrease_count = 0; // Reset counter if altitude increases or stays the same
    }
    if (detector->decrease_count >= DECREASE_THRESHOLD) {
        detector->apogee_reached = 1;
    }
    detector->last_altitude = current_altitude;
}

int is_apogee_reached(ApogeeDetector *detector) {
    return detector->apogee_reached;
}

double altitude_backing_array[WINDOW_SIZE];
ApogeeDetector detector;
void setup() {
    Serial.begin(9600);
    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP280 sensor, check wiring!");
        while (1);
    }

    // Calibrate the BMP280 sensor
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_125); /* Standby time. */

    init_apogee_detector(&detector, altitude_backing_array, WINDOW_SIZE);
}

void loop() {
    double current_altitude = bmp.readAltitude(1013.25); 
    update_apogee_detector(&detector, current_altitude);
    
    Serial.print("Current Altitude: ");
    Serial.println(current_altitude);
    
    if (is_apogee_reached(&detector)) {
        Serial.print("Apogee reached at altitude: ");
        Serial.println(detector.last_altitude);
        while (1); // Stop the loop once apogee is detected
    }   
    delay(100);
}
// ---------- SIGNED OFF BY LEROY ---------- //