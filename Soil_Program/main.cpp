//Include libraries for the Arduino and I2C
#include <SPI.h>
#include <Wire.h>
#include <Arduino.h>
#include <math.h>

//Includes the Library for the Analog Digital Converters
#include <Adafruit_ADS1X15.h>

//Includes the libraries for the BME280 sensor
#include <Adafruit_Sensor.h>
//#include <Adafruit_BME280.h>

//Includes the library for the SD card
#include <SD.h>

//Includes the Library for the Real Time Clock
#include <DS3231.h>

//Adds Definitions to the SD pin and File info for SD card
#define SD_SS_PIN 12
#define FILE_NAME "Test1.txt"
#define LOGGERNAME "Mayfly Soil Vapor Sensor"
#define DATA_HEADER "Date,Time,Solenoid Number,CO2(PPM),CO2 Volt"

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
//Adafruit_BME280 bme1;
//Adafruit_BME280 bme2;

//Setting variables for the ADC results
int16_t results2;

float CO2_Volt;
float CO2_PPM;
float accuracy_modifer;
int active_solpin;

// Setting Valve pin
const int pump_pin = 11;

// Mayfly pins connected to control respective solenoids relays.
const int solenoid_pins[] = {4, 5, 6};


void setupLogFile() {
  //Checks if SD Card is missing
  if (!SD.begin(SD_SS_PIN)) {
    Serial.println("Error: SD card failed to initialise or is missing.");
  }

  //Check if the file already exists
  bool oldFile = SD.exists(FILE_NAME);

  //Open the file in write mode
  File logFile = SD.open(FILE_NAME, FILE_WRITE);

  //Add header information if the file did not already exist
  if (!oldFile) {
    logFile.println(LOGGERNAME);
    logFile.println(DATA_HEADER);
  }

  //Close the file to save it
  logFile.close();
}

String createDataRecord() {
  //Requests data from RTC
  year = Clock.getYear();
  hour = Clock.getHour(h24, PM);
  minute = Clock.getMinute();
  second = Clock.getSecond();
  month = Clock.getMonth(Century);
  DayMonth = Clock.getDate();

  //Create a String type data record in csv format seperated by commas
  String data = "";
  data += month;
  data += "/";
  data += DayMonth;
  data += "/";
  data += "20";
  data += year;
  data += ",";
  data += hour;
  data += ":";
  data += minute;
  data += ",";
  data += active_solpin;
  data += ",";
  data += CO2_PPM;
  data += ",";
  data += CO2_Volt;
  return data;
}

void logData(String rec) {
  //Re-open the file
  File logFile = SD.open(FILE_NAME, FILE_WRITE);

  //Write the CSV data
  logFile.println(rec);

  //Close the file to save it
  logFile.close();
}


void setup() {

  Wire.begin();
    //Begins serial communication with 9600 Baud
  Serial.begin(9600);

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
  


  setupLogFile();

  

}

void loop() {

  const int FLUSH_TIME = 30000;
  const int MEASUREMENT_TIME = 30000; 

  //___________Valve Control____________________________//
  
//for loop opens a specified solenoid, extracts gas, takes measurements, and loops to next solenoid

  for (int n = 0; n < 3; n++) {

    //voltage difference across AA2 and AA3 from CO2 sensor
    results2 = ads1.readADC_Differential_2_3();

    //converts voltage value to ppm
    CO2_Volt = ads1.computeVolts(results2);
    float CO2_Amp = CO2_Volt/250 * 1000;
    CO2_PPM = 312.5 * CO2_Amp - 1250;

    
    active_solpin = solenoid_pins[n];
    hour = Clock.getHour(h24, PM);
    minute = Clock.getMinute();
    second = Clock.getSecond();
    
    //add a list of solenoid pin numbers and go through them to activate solenoid!

    //Careful now! If pin eight is used for a solenoid it triggers the relay sporadically if mayfly is restarted

    //Turn specified solenoid on for delay(seconds)
    digitalWrite(active_solpin, HIGH);
     

    digitalWrite(pump_pin, HIGH);
    delay(FLUSH_TIME);
  

    digitalWrite(pump_pin, LOW);
    delay(MEASUREMENT_TIME);
    Serial.print(hour);
    Serial.print(':');
    Serial.print(minute);
    Serial.print(':');
    Serial.print(second);
    Serial.print(' ');

    Serial.print("Solenoid:");
    Serial.print(active_solpin);

    Serial.print(" CO2 = ");
    Serial.print(CO2_PPM);
    Serial.println("PPM ");
    

    digitalWrite(active_solpin, LOW);

    String dataRec = createDataRecord();

    logData(dataRec);

    
  }
  
}
