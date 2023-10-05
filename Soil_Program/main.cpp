//Include libraries for the Arduino and I2C
#include <SPI.h>
#include <Wire.h>
#include <Arduino.h>
#include <math.h>

//Includes the Library for the Analog Digital Converters
#include <Adafruit_ADS1X15.h>

//Includes the libraries for the BME280 sensor
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//Includes the library for the SD card
#include <SD.h>

//Includes the Library for the Real Time Clock
#include <DS3231.h>

//Defines Sea Level Pressure for BME280
#define SEALEVELPRESSURE_HPA (1013.25)

//Adds Definitions to the SD pin and File info for SD card
#define SD_SS_PIN 12
#define FILE_NAME "Test1.txt"
#define LOGGERNAME "Mayfly Soil Vapor Sensor"
#define DATA_HEADER "Date,Time,Solenoid Number,Heater State,Water Level Measurement,O2(%),O2 Volt,CO2(PPM),CO2 Volt,CH4(PPM),CH4 Volt,Ambient Temperature (C),Ambient Humidity (%), Ambient Pressure (hPa), Temperature (C), Humidity (%), Pressure (hPa)"

//Creates Clock Variable for RTC
DS3231 Clock;

//Sets RTC Parameters
bool h24;
bool PM;
bool Century;
String sensor;
const int chipSelect = 10;
int second, minute, hour, month, DayMonth, year;

//Creates variables for different ADCs
Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2;

//Creates variables for the BME280s
Adafruit_BME280 bme1;
Adafruit_BME280 bme2;

//Setting variables for the ADC results
int16_t results2;
float multiplier = 0.125F;
float CO2_Volt;
float CO2_PPM;
float accuracy_modifer;

//Creates variables for BME280 results



int solenoid;
// Setting Valve pin
const int pump_pin = 11;

// Mayfly pins connected to control respective solenoids relays.
const int solenoid_pins[] = {4, 5, 6};




void setup() {

  Wire.begin();

  // Set OUTPUT Pins on Mayfly Microcontroller
  pinMode(pump_pin, OUTPUT);

  for(int pin = 0; pin < 3; pin++){
    pinMode(solenoid_pins[pin], OUTPUT);
  }

  //LOW implies off for relay board
  digitalWrite(pump_pin, LOW);
  for(int pin = 0; pin < 3; pin++){
    digitalWrite(solenoid_pins[pin], LOW);
  }

    //Begins serial communication with 9600 Baud
  Serial.println("About to set ads variables");
  //Set the gain for each ADC located on AA0,AA1, AA2, and AA3 on Mayfly Microcontroller
  ads1.setGain(GAIN_ONE);  // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  //Set the gain for ADS1115 ADC
  ads2.setGain(GAIN_ONE);  // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  Serial.println("ads variable setup complete");
  //Check if ADC on mayfly is operational at 0x48 address
  Serial.println("Checking if ads1 is initialized");
  if (!ads1.begin(0x48)) {
    Serial.println("Failed to initialize ADS1.");
    while (1)
      ;
  }
  
  //Begins serial communication with 9600 Baud
  Serial.begin(9600);

}

void loop() {
  results2 = ads1.readADC_Differential_2_3();

  CO2_Volt = results2 * multiplier / 1000;
  float CO2_Amp = CO2_Volt/250 * 1000;
  CO2_PPM = 312.5 * CO2_Amp - 1250;

  const int flush_time = 6000;
  const int measurement_time = 10000;
  

  //___________Valve Control____________________________//
  
//for loop opens a specified solenoid, extracts gas, takes measurements, and loops to next solenoid

  for (int n = 0; n < 3; n++) {
    int active_solpin = solenoid_pins[n];
    

    //add a list of solenoid pin numbers and go through them to activate solenoid!

    //Careful now! If pin eight is used for a solenoid it triggers the relay sporadically if mayfly is restarted

    //Turn specified solenoid on for delay(seconds)
    digitalWrite(active_solpin, HIGH);
    delay(1000);
     

    digitalWrite(pump_pin, HIGH);
    delay(flush_time);

    delay(10000);
    Serial.print("Solenoid:");
    Serial.print(active_solpin);

    Serial.print(" CO2 = ");
    Serial.print(CO2_PPM);
    Serial.println("PPM, ");

    digitalWrite(pump_pin, LOW);
    delay(measurement_time);

    digitalWrite(active_solpin, LOW);


    // Read Oxygen and CO2 results from the Mayfly ADCs
 

  
    //Compute voltage from ADC results
  





  
    //_______________Algorithm Computation_______________________________________


    //End Barrier for serial monitor


    delay(1000);
  }
}
