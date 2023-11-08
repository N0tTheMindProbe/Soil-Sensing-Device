#ifndef COLLECT_SAMPLE_H
#define COLLECT_SAMPLE_H

#include <Arduino.h>
#include <SPI.h>


class CollectSample {
public:
    
    void getSample(uint8_t pin_number);

private:
    const uint8_t pumpPin;
    const uint8_t* solenoidPins;
    uint8_t solenoidPinCount;
    uint32_t flushTime;
    uint32_t measurementTime;
};

#endif
