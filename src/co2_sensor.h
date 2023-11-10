/**
 * co2_sensor.h
 * 
 * Reads the voltage input from the Alphasense IRC-A1 CO2 sensor  
 * and converts the value into parts per million (ppm)
 * 
 * Info: https://www.alphasense.com/products/ndir-safety/
 * Spec: https://www.alphasense.com/wp-content/uploads/2018/04/IRC-A1.pdf
 *       https://www.alphasense.com/wp-content/uploads/2022/10/AAN_202-04_App-Note_V0.pdf
 */

// Header Guards
#ifndef CO2_SENSOR_H_
#define CO2_SENSOR_H_

// Included Dependencies
#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <SPI.h>

/// The assumed address of the ADS1115, 1001 000 (ADDR = GND)
#define ADS1115_ADDRESS 0x48


// Class for Alphasense IRC-A1 CO2 sensor
class AlphasenseCO2 {
    
  public:

    // Sets up functions
    bool begin(void);

    // Function to convert digital value from sensor ADC to CO2 reading in ppm
    float getCO2PPM(void);

  private:

    // Create an instance of the ADC
    Adafruit_ADS1115 ads;

};

#endif
