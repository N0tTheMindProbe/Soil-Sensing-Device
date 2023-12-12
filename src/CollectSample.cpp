#include "CollectSample.h"
#include <Arduino.h>


void CollectSample::begin(){

    pinMode(pumpPin, OUTPUT);

    for(int pin = 0; pin < 3; pin++){
        pinMode(solenoidPins[pin], OUTPUT);
        }

    //LOW implies off for relay board
    digitalWrite(pumpPin, LOW);
    for(int pin = 0; pin < 3; pin++){
        digitalWrite(solenoidPins[pin], LOW);
        }
}
void CollectSample::getSample(uint8_t pin_number) {
    digitalWrite(solenoidPins[pin_number], HIGH);
    delay(1000);
    digitalWrite(pumpPin, HIGH);
    delay(flushTime);
    digitalWrite(pumpPin, LOW);
    delay(measurementTime);
    digitalWrite(solenoidPins[pin_number], LOW);
}
