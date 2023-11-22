/*****************************************************************************
GetValuesCO2.ino

Reads the voltage input from the Alphasense IRC-A1 CO2 sensor 
and converts the value into parts per million (ppm)
*****************************************************************************/


// Include libraries for the Arduino and I2C
#include <Arduino.h>
#include <AlphasenseIRCA1_CO2.h>


// Set delay between sensor readings (in ms)
const unsigned long delayValue = 1000;

// Construct the CO2 sensor instance
AlphasenseCO2 co2_sensor;


// ==========================================================================
//  Arduino Setup Function
// ==========================================================================
void setup() {
    // Begin serial communication at 9600 Baud
    Serial.begin(9600);

    co2_sensor.begin();

    // Print headers for output table
    Serial.print("Time(ms)  ");
    Serial.println("CO2(ppm)");

}

// ==========================================================================
//  Arduino Loop Function
// ==========================================================================
void loop() {

    // Get the CO2 concentration in ppm
    float CO2PPM = co2_sensor.getCO2PPM();
    
    // Print new line to table of time since sketch began
    // and measured values, to help determine warmup and stabilization times
    Serial.print(millis());
    Serial.print("     ");
    Serial.println(CO2PPM);

    // Wait for the next measurement
    delay(delayValue);
}