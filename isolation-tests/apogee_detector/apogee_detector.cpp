#include "apogee_detector.h"

RollingWindow::RollingWindow() {}

void RollingWindow::init(double *pBackingArray, size_t capacity) {
    backingArray = pBackingArray;
    this->capacity = capacity;
    size = 0;
    front = 0;
    sumOfElements = 0.0;
}

void RollingWindow::addDataPoint(double dataPoint) {
    if (size >= capacity) {
        sumOfElements -= backingArray[modRollingWindow(front, capacity)];
        front = modRollingWindow(front + 1, capacity);
        size--;
    }

    size_t backInsertionIdx = modRollingWindow(front + size, capacity);
    backingArray[backInsertionIdx] = dataPoint;
    sumOfElements += dataPoint;
    size++;
}

double RollingWindow::getLatestDataPoint() {
    return backingArray[modRollingWindow(front + size - 1, capacity)];
}

double RollingWindow::getEarliestDataPoint() {
    return backingArray[modRollingWindow(front, capacity)];
}

size_t RollingWindow::modRollingWindow(size_t index, size_t modulo) {
    return (index % modulo + modulo) % modulo;
}

ApogeeDetector::ApogeeDetector() {}

void ApogeeDetector::init(double *backingArray, size_t capacity) {
    altitudeWindow.init(backingArray, capacity);
    lastAltitude = -1.0;
    apogeeReached = false;
    decreaseCount = 0;
}

void ApogeeDetector::update(double currentAltitude) {
    altitudeWindow.addDataPoint(currentAltitude);

    if (apogeeReached) {
        return;
    }

    if (lastAltitude >= 0 && currentAltitude < lastAltitude) {
        decreaseCount++;
    } else {
        decreaseCount = 0;
    }

    if (decreaseCount >= DECREASE_THRESHOLD) {
        apogeeReached = true;
    }

    lastAltitude = currentAltitude;
}

bool ApogeeDetector::isApogeeReached() {
    return apogeeReached;
}

double ApogeeDetector::getLastAltitude() {
    return lastAltitude;
}