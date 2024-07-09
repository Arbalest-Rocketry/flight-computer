#include "apogee.h"

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

    if (detector->apogee_reached) {
        return; // Exit the function if apogee has already been reached
    }

    Serial.print("Current altitude: ");
    Serial.println(current_altitude);
    
    if (detector->last_altitude >= 0) {
        if (current_altitude < detector->last_altitude) {
            detector->decrease_count++;
            Serial.print("Decrease count: ");
            Serial.println(detector->decrease_count);
        } else {
            detector->decrease_count = 0; // Reset counter if altitude increases or stays the same
        }
    }

    if (detector->decrease_count >= DECREASE_THRESHOLD) {
        detector->apogee_reached = 1;
        Serial.println("Apogee Reached!");
    }
    detector->last_altitude = current_altitude;
}

int is_apogee_reached(ApogeeDetector *detector) {
    return detector->apogee_reached;
}