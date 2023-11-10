//Reads the voltage input from the Alphasense IRC-A1 CO2 sensor and converts the value into parts per million (ppm)

// Include libraries for the Arduino and I2C
#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <SPI.h>

// Create an instance of the ADC
Adafruit_ADS1115 ads;

// Set delay between sensor readings (in ms)
const unsigned long delayValue = 5000;

// Function to convert digital value from sensor ADC to CO2 reading in ppm
float getCO2PPM(int16_t digitalValue) {
    // Convert digital CO2 value to voltage (V)
    float CO2Voltage = ads.computeVolts(digitalValue);

    // Convert voltage to current (mA) - assuming a 250 Ohm resistor is in series
    float CO2Current = (CO2Voltage / 250) * 1000;

    // Convert current to ppm (using a formula recommended by the sensor manufacturer)
    float CO2PPM = 312.5 * CO2Current - 1250;

    return CO2PPM;
}

void setup() {
    // Begin serial communication at 9600 Baud
    Serial.begin(9600);
    Serial.println("Initializing...");

    // Set the gain for the ADC
    ads.setGain(GAIN_ONE); // 1x gain +/- 4.096V 1 bit = 2mV 0.125mV
    Serial.println("Gain set.");

    // Check if the ADC is operational
    Serial.print("Checking ADC... ");
    if (!ads.begin(0x48)) {
        Serial.println("Failed to initialize ADS.");
        while (1); // Halt the program
    }
    Serial.println("ADC initialized successfully.");
}

void loop() {
    // Measure the voltage difference across two pins from the CO2 sensor
    int16_t results = ads.readADC_Differential_2_3();

    // Convert the results to CO2 concentration in ppm
    float CO2PPM = getCO2PPM(results);
    Serial.print(CO2PPM);
    Serial.println("ppm");

    // Wait for the next measurement
    delay(delayValue);
}




