#include "Alphasense_IRC_A1.h"

Alphasense_IRC_A1::Alphasense_IRC_A1() {
    // Constructor can initialize the ADC, if needed
}

void Alphasense_IRC_A1::begin() {
    // Initialization code for the sensor
    ads.setGain(GAIN_ONE);
    ads.begin(0x48);
}

float Alphasense_IRC_A1::getCO2PPM() {
    int16_t results = ads.readADC_Differential_2_3();
    float CO2Voltage = ads.computeVolts(results);
    float CO2Current = (CO2Voltage / 250) * 1000;
    return 312.5 * CO2Current - 1250;
}
