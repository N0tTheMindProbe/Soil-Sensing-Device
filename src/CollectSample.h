#ifndef COLLECT_SAMPLE_H
#define COLLECT_SAMPLE_H

#include <Arduino.h>
#include <SPI.h>


class CollectSample {
public:
    
    void getSample(uint8_t pin_number);
    void begin();

private:
    const int solenoidPins[3] = {4,5,6};
    const int pumpPin = 7;
    int flushTime = 6000;
    int measurementTime = 6000;
};

#endif
