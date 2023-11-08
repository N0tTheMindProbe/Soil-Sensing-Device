#include "CollectSample.h"
#include <Arduino.h>


void CollectSample::getSample(uint8_t pin_number) {
    digitalWrite(solenoidPins[pin_number], HIGH);
    delay(1000);
    digitalWrite(pumpPin, HIGH);
    delay(flushTime);
    digitalWrite(pumpPin, LOW);
    delay(measurementTime);
    digitalWrite(solenoidPins[pin_number], LOW);
}
