/**
 * AlphasenseIRCA1_CO2.h
 * 
 * Reads the voltage input from the Alphasense IRC-A1 Carbon Dioxide (CO2) sensor  
 * and converts the value into parts per million (ppm)
 * 
 * Info: https://www.alphasense.com/products/ndir-safety/
 * Spec: https://www.alphasense.com/wp-content/uploads/2018/04/IRC-A1.pdf
 *       https://www.alphasense.com/wp-content/uploads/2017/09/NDIR-Transmitter.pdf
 *       https://www.alphasense.com/wp-content/uploads/2022/10/AAN_202-04_App-Note_V0.pdf
 *       https://www.alphasense.com/wp-content/uploads/2022/10/AAN_201-06_App-Note_V0.pdf
 */

// Header Guards
#ifndef ALPHASENSEIRCA1_CO2_H_
#define ALPHASENSEIRCA1_CO2_H_

// Included Dependencies
#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <SPI.h>

/// The assumed address of the ADS1115, 1001 000 (ADDR = GND)
#define ADS1115_ADDRESS (0x48)


// Class for Alphasense IRC-A1 CO2 sensor
class Alphasense_IRC_A1 {
    
  public:

    // This function sets up the communication to the ADC.
    // It should be run during the arduino "setup" function.
    bool begin(void);

    // This gets values back from the sensor via the ADC.
    // It converts voltage values from sensor ADC to CO2 in ppm
    float getCO2PPM(void);

  private:

    // Create an instance of the ADC
    Adafruit_ADS1115 ads;

};

#endif