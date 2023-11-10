/*****************************************************************************
GetValuesCO2.ino

Reads the voltage input from the Alphasense IRC-A1 CO2 sensor 
and converts the value into parts per million (ppm)
*****************************************************************************/


// Include libraries for the Arduino and I2C
#include <Arduino.h>
#include <co2_sensor.h>


// Set delay between sensor readings (in ms)
const unsigned long delayValue = 5000;

// Construct the CO2 sensor instance
AlphasenseCO2 co2_sensor;


void setup() {
    // Begin serial communication at 9600 Baud
    Serial.begin(9600);

    co2_sensor.begin();

}

void loop() {

    // Get the CO2 concentration in ppm
    float CO2PPM = co2_sensor.getCO2PPM();
    Serial.print(CO2PPM);
    Serial.println("ppm");

    // Wait for the next measurement
    delay(delayValue);
}