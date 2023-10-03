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

//Setting variables for the ADC results
int16_t results2;
float multiplier = 0.125F;
float CO2_Volt;
float CO2_PPM;
float accuracy_modifer;


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

//Creates function to save data in a string
void logData(String rec) {
  //Re-open the file
  File logFile = SD.open(FILE_NAME, FILE_WRITE);

  //Write the CSV data
  logFile.println(rec);

  //Close the file to save it
  logFile.close();
}

//Initialise the SD card and creates file if not there
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

void setup() {

  Wire.begin();

  //Begins serial communication with 9600 Baud
  Serial.begin(9600);

  //Set the gain for each ADC located on AA0,AA1, AA2, and AA3 on Mayfly Microcontroller
  ads1.setGain(GAIN_ONE);  // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  //Set the gain for ADS1115 ADC
  ads2.setGain(GAIN_ONE);  // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV

  //Check if ADC on mayfly is operational at 0x48 address
  if (!ads1.begin(0x48)) {
    Serial.println("Failed to initialize ADS1.");
    while (1)
      ;
  }


  //Check if ADS1115 ADC is operational at 0x49 address
  if (!ads2.begin(0x49)) {
    Serial.println("Failed to initialize ADS2.");
    while (1);
  }

  //Initialise log file
  setupLogFile();

}

void loop() {


    // Read Oxygen and CO2 results from the Mayfly ADCs
    results2 = ads1.readADC_Differential_2_3();
  
    //Compute voltage from ADC results
    CO2_Volt = results2 * multiplier / 1000;

    //_______________Algorithm Computation_______________________________________


    //Converts Volts CO2 to PPM
    //CO2_PPM = (1671.4 * abs(CO2_Volt) - 1287.3);
    //CO2_PPM = (1671.4 * abs(CO2_Volt) - 1287.3);
    //CO2_PPM = (-178.56 * pow(abs(CO2_Volt),2)) + (2608.7 * abs(CO2_Volt)) - 2297.3;
    // CO2_PPM = 3746.1 * log(abs(CO2_Volt)) - 52.951;
    CO2_PPM = (687.33 * pow(abs(CO2_Volt),3)) - (4334.2 * pow(abs(CO2_Volt),2)) + (9383.9 * abs(CO2_Volt)) - 5605.8;

    //Error Reading if negative value
    if (CO2_PPM <= 0) {
      CO2_PPM = -999;
    }

    // Creates data string from function
    String dataRec = createDataRecord();

    //Display Measurements to Serial Monitor
    Serial.println("-----------------------------------------------------------------------------");


    Serial.print("CO2 = ");
    Serial.print(CO2_Volt,3);
    Serial.print("V, ");

    Serial.print("CO2 = ");
    Serial.print(CO2_PPM);
    Serial.print("PPM, ");


    Serial.print("\n");
    Serial.println("-----------------------------------------------------------------------------");
    //End Barrier for serial monitor


    //Save the data record to the log file
    logData(dataRec);

    delay(1000);
  
}




