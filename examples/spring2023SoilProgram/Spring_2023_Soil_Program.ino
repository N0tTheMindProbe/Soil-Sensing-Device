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
int16_t results1;
int16_t results2;
int16_t results3;
int16_t results4;
float multiplier = 0.125F;
float O2_Volt;
float CO2_Volt;
float Battery_Lvl;
float Methane_Volt;
float O2_Percentage;
float CO2_PPM;
float Methane_PPM;
float accuracy_modifer;
float O2_Actual_Percentage;

//Creates variables for BME280 results

float atemp;
float apressure;
float ahumidity;
float temp;
float pressure;
float humidity;



int solenoid;
int k;
// Setting Valve pin
int flushPin = 11;

// Mayfly pins connected to control respective solenoids relays.
int solenoid_pins[] = { 23, 7, 9, 10 };

// Pump
int dir1PinA = 8;
int dir2PinA = 6;
int speedPinA = 4;  // Needs to be a PWM pin to be able to control motor speed

// Heating Element
int dir1PinB = 18;
int dir2PinB = 20;
int speedPinB = 5;  // Needs to be a PWM pin to be able to control motor speed

//Methane Calculations
float Rs;
float Ro = 23250;  // ohms
float Rl = 3170;
float Vdd = 5;
float Vload;
float ratio_corr;
float CH4_raw;
float CH4_log;
float CH4_ratio;
float waterLevel;

String heater = "off";



void setup() {

  Wire.begin();

  // Set OUTPUT Pins on Mayfly Microcontroller
  pinMode(dir1PinA, OUTPUT);
  pinMode(dir2PinA, OUTPUT);
  pinMode(dir1PinB, OUTPUT);
  pinMode(dir2PinB, OUTPUT);
  pinMode(speedPinA, OUTPUT);
  pinMode(flushPin, OUTPUT);
  pinMode(solenoid_pins[0], OUTPUT);
  pinMode(solenoid_pins[1], OUTPUT);
  pinMode(solenoid_pins[2], OUTPUT);
  pinMode(solenoid_pins[3], OUTPUT);

  //HIGH implies off for relay board
  digitalWrite(flushPin, HIGH);
  digitalWrite(solenoid_pins[0], HIGH);
  digitalWrite(solenoid_pins[1], HIGH);
  digitalWrite(solenoid_pins[2], HIGH);
  digitalWrite(solenoid_pins[3], HIGH);

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
    while (1)
      ;
  }

  //Sets BME1 I2C Location to 0x76
  if (!bme1.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }
  //Sets BME2 I2C Location to 0x77
  if (!bme2.begin(0x77)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }

  //Initialise log file
  setupLogFile();

}

void loop() {

  //Force Reset of for loop because unknown reason k++ increased infinitely
  k = 0;
  //___________Valve Control____________________________//
  
//for loop opens a specified solenoid, extracts gas, takes measurements, and loops to next solenoid

  for (k = 0; k < 4; k++) {
    int active_solpin = solenoid_pins[k];

    //add a list of solenoid pin numbers and go through them to activate solenoid!

    //Careful now! If pin eight is used for a solenoid it triggers the relay sporadically if mayfly is restarted

    //Turn specified solenoid on for delay(seconds)
    digitalWrite(active_solpin, LOW);
    delay(1000);

    //Turn the Pump ON for delay(seconds) 6000 is enough to fill the entire chamber at max pump speed
    analogWrite(speedPinA, 255);  //Sets speed variable via PWM
    digitalWrite(dir1PinA, LOW);
    digitalWrite(dir2PinA, HIGH);
    delay(6000);

    //Turn the Pump OFF to let gas diffuse in chamber for delay(seconds)
    analogWrite(speedPinA, 0);  //Sets speed variable via PWM
    digitalWrite(active_solpin, HIGH);
    delay(10000);

    // Read Oxygen and CO2 results from the Mayfly ADCs
    results1 = ads1.readADC_Differential_0_1();
    results2 = ads1.readADC_Differential_2_3();
    // Read battery level and methane voltage from ADS1115
    results3 = ads2.readADC_SingleEnded(0);
    results4 = ads2.readADC_SingleEnded(1);

    //Read results from both BME280s. a specifies ambient
    atemp = bme1.readTemperature();
    apressure = bme1.readPressure() / 100.0F;
    ahumidity = bme1.readHumidity();
    temp = bme2.readTemperature();
    pressure = bme2.readPressure() / 100.0F;
    humidity = bme2.readHumidity();


    //Compute voltage from ADC results
    O2_Volt = results1 * multiplier / 1000;
    CO2_Volt = results2 * multiplier / 1000;
    Battery_Lvl = results3 * multiplier / 1000;
    Methane_Volt = results4 * multiplier / 1000;

    //_______________Algorithm Computation_______________________________________

    //Converts Voltage to Percent O2%
    O2_Percentage = (((0.0763 * abs(O2_Volt)) - 0.0841) * 100);

    //Error Reading if negative value
    if (O2_Percentage <= 0) {
      O2_Percentage = -999;
    }

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

    //Converts Volts CH4 to PPM
    Methane_PPM = 8.2318 * pow(2.71828, (2.7182 * Methane_Volt));

    //Error Reading if negative value
    if (Methane_PPM <= 0) {
      Methane_PPM = -999;
    }


    //Heating element connected to L298N H-Bridge. Heat the sensor chamber if temperature is nearing ambient
    if (temp < atemp + 2) {
      //Heating Element ON
      analogWrite(speedPinB, 50);  //Sets voltage supplied to heating element from 0-255
      digitalWrite(dir1PinB, LOW);
      digitalWrite(dir2PinB, HIGH);
      heater = "ON";
    } else {
      //Heating Element OFF
      analogWrite(speedPinB, 0);  //Sets voltage supplied to zero
      digitalWrite(dir1PinB, LOW);
      digitalWrite(dir2PinB, HIGH);
      heater = "OFF";
    }

    //Compute the battery voltage based on ADS1115 readings of voltage divider
    float R_load = 9862.939;
    float R_1 = 99725.277;
    Battery_Lvl = ((Battery_Lvl * (R_load+ R_1)) / R_load);
    //Currently no conditions are set to power off the device if battery voltage is to low

    //Measure voltage from Capacitive Soil Moisture Sensor
    waterLevel = analogRead(A0);

    // Creates data string from function
    String dataRec = createDataRecord();

    //Display Measurements to Serial Monitor
    Serial.println("-----------------------------------------------------------------------------");

    //Informs Solenoid used
    Serial.print("Solenoid");
    solenoid = (k + 1);
    Serial.print(solenoid);
    Serial.print("| ");

    //Informs if heater element is active
    Serial.print("Heater ");
    Serial.print(heater);
    Serial.print("| ");

    // Serial.print("Methane = ");
    // Serial.print(Methane_PPM);
    // Serial.print("PPM, ");

    Serial.print("Water = ");
    Serial.print(waterLevel);
    Serial.print(" Moisture| ");

    Serial.print("CO2 = ");
    Serial.print(CO2_Volt,3);
    Serial.print("V, ");

    Serial.print("CO2 = ");
    Serial.print(CO2_PPM);
    Serial.print("PPM, ");

    Serial.print("O2 Volt= ");
    Serial.print(O2_Volt);
    Serial.print("V| ");

    Serial.print("O2% = ");
    Serial.print(O2_Percentage);
    Serial.print("%| ");

    Serial.print("CH4 Volt = ");
    Serial.print(Methane_Volt);
    Serial.print("V| ");

    Serial.print("CH4 PPM = ");
    Serial.print(CH4_log);
    Serial.print("PPM| ");

    Serial.print("Temperature = ");
    Serial.print(temp);
    Serial.print("C| ");

    Serial.print("RH = ");
    Serial.print(humidity);
    Serial.print("%| ");


    Serial.print("\n");

    Serial.print("Ambient Temperature = ");
    Serial.print(atemp);
    Serial.print("C| ");

    Serial.print(" Ambient RH = ");
    Serial.print(ahumidity);
    Serial.print("%| ");

    Serial.print("\n");
    Serial.println("-----------------------------------------------------------------------------");
    //End Barrier for serial monitor


    //Save the data record to the log file
    logData(dataRec);

    delay(1000);
  }
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
  data += solenoid;
  data += ",";
  data += heater;
  data += ",";
  data += waterLevel;
  data += ",";
  data += O2_Percentage;
  data += ",";
  data += O2_Volt;
  data += ",";
  data += CO2_PPM;
  data += ",";
  data += CO2_Volt;
  data += ",";
  data += CH4_log;
  data += ",";
  data += Methane_Volt;
  data += ",";
  data += atemp;
  data += ",";
  data += ahumidity;
  data += ",";
  data += apressure;
  data += ",";
  data += temp;
  data += ",";
  data += humidity;
  data += ",";
  data += pressure;
  return data;
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

//Creates function to save data in a string
void logData(String rec) {
  //Re-open the file
  File logFile = SD.open(FILE_NAME, FILE_WRITE);

  //Write the CSV data
  logFile.println(rec);

  //Close the file to save it
  logFile.close();
}
