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

//Defines Sea Level Pressure for BME280
#define SEALEVELPRESSURE_HPA (1013.25)

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
int second, minute, hour, month, DayMonth, year;

//Creates variables for ADC
Adafruit_ADS1115 ads1;

//Setting variables for the ADC results
int16_t results2;
float CO2_Volt;
float CO2_PPM;


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

    //Begins serial communication with 9600 Baud
  Serial.println("Setting ads variables...");
  //Set the gain for ADC located on AA1 on Mayfly Microcontroller
  ads1.setGain(GAIN_ONE);  // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV

  Serial.println("ADS variable setup complete");

  //Check if ADC on mayfly is operational at 0x48 address
  Serial.println("Checking if ads1 is initialized...");
  if (!ads1.begin(0x48)) {
    Serial.println("Failed to initialize ADS1.");
    while (1);
  }
  Serial.println("Initialization complete.");
  
  setupLogFile(); 

}

void loop() {

  //voltage difference across AA2 and AA3 from CO2 sensor
  results2 = ads1.readADC_Differential_2_3();
 
  //determines voltage value taken from sensor, converts to amps
  CO2_Volt = ads1.computeVolts(results2);
  float CO2_Amp = CO2_Volt/250 * 1000;

  //converts voltage value to ppm (recommended by sensor manufacturer when reading voltage across 250ohm resistor)
  CO2_PPM = 312.5 * CO2_Amp - 1250;

  //the amount of time the Mayfly waits between taking readings (defaults to 10 seconds)
  const int wait_time = 10000;

  //establishes RTC values for serial monitor
  hour = Clock.getHour(h24, PM);
  minute = Clock.getMinute();
  second = Clock.getSecond();


  //prints values to serial monitor with timestamp
  Serial.print(hour);
  Serial.print(':');
  Serial.print(minute);
  Serial.print(':');
  Serial.print(second);
  Serial.print(' ');

  Serial.print(" CO2 = ");
  Serial.print(CO2_PPM);
  Serial.print("PPM, ");

  Serial.print(" CO2 Voltage = ");
  Serial.print(CO2_Volt);
  Serial.println("V ");


  //sends data to sd card
  String dataRec = createDataRecord();

  logData(dataRec);
  delay(wait_time);
  
    
  }




