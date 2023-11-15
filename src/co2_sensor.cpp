/**
 * co2_sensor.cpp
 */

#include "co2_sensor.h"


//----------------------------------------------------------------------------
//                          PUBLIC SENSOR FUNCTIONS
//----------------------------------------------------------------------------

// Sets up functions
bool AlphasenseCO2::begin() {
    Serial.println("Initializing Alphasense IRC-A1 CO2 sensor");

    // Set the gain for the ADC
    ads.setGain(GAIN_ONE); // 1x gain +/- 4.096V 1 bit = 2mV 0.125mV
    Serial.println("Gain set.");

    // Check if the ADC is operational
    Serial.print("Checking ADC... ");
    if (!ads.begin(ADS1115_ADDRESS)) {
        Serial.println("Failed to initialize ADS.");
        return false;
    }

    Serial.println("ADC initialized successfully.");
    return true;
}


// Function to convert digital value from sensor ADC to CO2 reading in ppm
float AlphasenseCO2::getCO2PPM(void) {
    // Measure the voltage difference across two pins from the CO2 sensor
    int16_t results = ads.readADC_Differential_2_3();

    // Convert digital CO2 value to voltage (V)
    float CO2Voltage = ads.computeVolts(results);

    // Convert voltage to current (mA) - assuming a 250 Ohm resistor is in series
    float CO2Current = (CO2Voltage / 250) * 1000;

    // Convert current to ppm (using a formula recommended by the sensor manufacturer)
    float CO2PPM = 312.5 * CO2Current - 1250;

    return CO2PPM;
}