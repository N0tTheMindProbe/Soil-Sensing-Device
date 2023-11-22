/** =========================================================================
 * @file double_logger.ino
 * @brief Example logging at two different timing intervals
 *
 * @author Sara Geleskie Damiano <sdamiano@stroudcenter.org>
 * @copyright (c) 2017-2022 Stroud Water Research Center (SWRC)
 *                          and the EnviroDIY Development Team
 *            This example is published under the BSD-3 license.
 *
 * Build Environment: Visual Studios Code with PlatformIO
 * Hardware Platform: EnviroDIY Mayfly Arduino Datalogger
 *
 * DISCLAIMER:
 * THIS CODE IS PROVIDED "AS IS" - NO WARRANTY IS GIVEN.
 * ======================================================================= */

// ==========================================================================
//  Defines for TinyGSM
// ==========================================================================
/** Start [defines] */
#ifndef TINY_GSM_RX_BUFFER
#define TINY_GSM_RX_BUFFER 64
#endif
#ifndef TINY_GSM_YIELD_MS
#define TINY_GSM_YIELD_MS 2
#endif
/** End [defines] */

// ==========================================================================
//  Include the libraries required for any data logger
// ==========================================================================
/** Start [includes] */
// The Arduino library is needed for every Arduino program.
#include <Arduino.h>

// EnableInterrupt is used by ModularSensors for external and pin change
// interrupts and must be explicitly included in the main program.
#include <EnableInterrupt.h>

// Include the main header for ModularSensors
#include <ModularSensors.h>
/** End [includes] */


// ==========================================================================
//  Data Logging Options
// ==========================================================================
/** Start [logging_options] */
// The name of this program file
const char* sketchName = "soil_sensor_device.cpp";
// Logger ID - we're only using one logger ID for both "loggers"
const char* LoggerID = "";
// The TWO filenames for the different logging intervals
const char* FileName10cm = "Logger_10cmDepth.csv";
const char* FileName20cm = "Logger_20cmDepth.csv";
const char* FileName50cm = "Logger_50cmDepth.csv";
// Your logger's timezone.
const int8_t timeZone = -5;  // Eastern Standard Time
// NOTE:  Daylight savings time will not be applied!  Please use standard time!

// Set the input and output pins for the logger
// NOTE:  Use -1 for pins that do not apply
const int32_t serialBaud = 115200;  // Baud rate for debugging
const int8_t  greenLED   = 8;       // Pin for the green LED
const int8_t  redLED     = 9;       // Pin for the red LED
const int8_t  buttonPin  = 21;      // Pin for debugging mode (ie, button pin)
const int8_t  wakePin    = 31;  // MCU interrupt/alarm pin to wake from sleep
// Mayfly 0.x D31 = A7
// Set the wake pin to -1 if you do not want the main processor to sleep.
// In a SAMD system where you are using the built-in rtc, set wakePin to 1
const int8_t sdCardPwrPin   = -1;  // MCU SD card power pin
const int8_t sdCardSSPin    = 12;  // SD card chip select/slave select pin
const int8_t sensorPowerPin = 22;  // MCU pin controlling main sensor power
/** End [logging_options] */

const int FLUSH_TIME = 10000;
const int MEASUREMENT_TIME = 3000;

// Setting Valve pin
const int pump_pin = 7;

// Mayfly pins connected to control respective solenoids relays.
const int solenoid_pins[] = {4, 5, 6};



// ==========================================================================
//  Wifi/Cellular Modem Options
// ==========================================================================
/** Start [xbee_wifi] */
// For the Digi Wifi XBee (S6B)
#include <modems/DigiXBeeWifi.h>
// Create a reference to the serial port for the modem

HardwareSerial& modemSerial = Serial1;  // Use hardware serial if possible
const int32_t   modemBaud   = 9600;     // All XBee's use 9600 by default

// Modem Pins - Describe the physical pin connection of your modem to your board
// NOTE:  Use -1 for pins that do not apply
const int8_t modemVccPin    = -2;    // MCU pin controlling modem power
const int8_t modemStatusPin = 19;    // MCU pin used to read modem status
const bool useCTSforStatus  = true;  // Flag to use the modem CTS pin for status
const int8_t modemResetPin  = 20;    // MCU pin connected to modem reset pin
const int8_t modemSleepRqPin = 23;   // MCU pin for modem sleep/wake request
const int8_t modemLEDPin = redLED;   // MCU pin connected an LED to show modem
                                     // status (-1 if unconnected)

// Network connection information
const char* wifiId  = "xxxxx";  // WiFi access point, unnecessary for GPRS
const char* wifiPwd = "xxxxx";  // WiFi password, unnecessary for GPRS

DigiXBeeWifi modemXBWF(&modemSerial, modemVccPin, modemStatusPin,
                       useCTSforStatus, modemResetPin, modemSleepRqPin, wifiId,
                       wifiPwd);
// Create an extra reference to the modem by a generic name
DigiXBeeWifi modem = modemXBWF;
/** End [xbee_wifi] */


// ==========================================================================
//  Using the Processor as a Sensor
// ==========================================================================
/** Start [processor_sensor] */
#include <sensors/ProcessorStats.h>

// Create the main processor chip "sensor" - for general metadata
const char*    mcuBoardVersion = "v1.1";
ProcessorStats mcuBoard(mcuBoardVersion);
/** End [processor_sensor] */


// ==========================================================================
//  Maxim DS3231 RTC (Real Time Clock)
// ==========================================================================
/** Start [ds3231] */
#include <sensors/MaximDS3231.h>

// Create a DS3231 sensor object
MaximDS3231 ds3231(1);
/** End [ds3231] */


  // ==========================================================================
  //    Maxim DS18 One Wire Temperature Sensor
  // ==========================================================================
  #include <sensors/MaximDS18.h>

  // OneWire Address [array of 8 hex characters]
  // If only using a single sensor on the OneWire bus, you may omit the address
  // DeviceAddress OneWireAddress1 = {0x28, 0xFF, 0xBD, 0xBA, 0x81, 0x16, 0x03, 0x0C};
  const int8_t OneWirePower = sensorPowerPin;  // Pin to switch power on and off (-1 if unconnected)
  const int8_t OneWireBus = 11;  // Pin attached to the OneWire Bus (-1 if unconnected) (D24 = A0)

  // Create a Maxim DS18 sensor objects (use this form for a known address)
  // MaximDS18 ds18(OneWireAddress1, OneWirePower, OneWireBus);

  // Create a Maxim DS18 sensor object (use this form for a single sensor on bus with an unknown address)
  MaximDS18 ds18(OneWirePower, OneWireBus);

// ==========================================================================
//  Bosch BME280 Environmental Sensor
// ==========================================================================
/** Start [bme280] */
#include <sensors/BoschBME280.h>

const int8_t I2CPower    = sensorPowerPin;  // Power pin (-1 if unconnected)
uint8_t      BMEi2c_addr = 0x76;
// The BME280 can be addressed either as 0x77 (Adafruit default) or 0x76 (Grove
// default) Either can be physically mofidied for the other address

// Create a Bosch BME280 sensor object
BoschBME280 bme280(I2CPower, BMEi2c_addr);
/** End [bme280] */


// ==========================================================================
//  Solenoid/Pump Gas Collector Sequence
// ==========================================================================

#include "CollectSample.h"

CollectSample collector;






// ==========================================================================
//  Creating the Variable Array[s] and Filling with Variable Objects
// ==========================================================================
/** Start [variable_arrays] */
// The variables to record at 1 minute intervals
Variable* variableList_gasMeasurement[] = {
                                    
                                    new MaximDS18_Temp(&ds18),
                                    new MaximDS3231_Temp(&ds3231),

                                    new BoschBME280_Temp(&bme280),
                                    new BoschBME280_Humidity(&bme280),
                                    new BoschBME280_Pressure(&bme280),
                                    new BoschBME280_Altitude(&bme280),
                                    new ProcessorStats_Battery(&mcuBoard),
                                    new ProcessorStats_FreeRam(&mcuBoard)};
// Count up the number of pointers in the 1-minute array
int variableCountGasMeasurement = sizeof(variableList_gasMeasurement) /
    sizeof(variableList_gasMeasurement[0]);
// Create the 1-minute VariableArray object
VariableArray array10cm;

// The variables to record at 5 minute intervals
Variable* variableList_at20cm[] = {//new MaximDS3231_Temp(&ds3231),
                                  // new ProcessorStats_Battery(&mcuBoard),
                                  // new ProcessorStats_FreeRam(&mcuBoard)
                                  };
// Count up the number of pointers in the 5-minute array
int variableCount20cm = sizeof(variableList_at20cm) /
    sizeof(variableList_at20cm[0]);
// Create the 5-minute VariableArray object
VariableArray array20cm;

Variable* variableList_at50cm[] = {new MaximDS3231_Temp(&ds3231)
                                  // new ProcessorStats_Battery(&mcuBoard),
                                  // new ProcessorStats_FreeRam(&mcuBoard)
                                  };
// Count up the number of pointers in the 5-minute array
int variableCount50cm = sizeof(variableList_at50cm) /
    sizeof(variableList_at50cm[0]);
// Create the 5-minute VariableArray object
VariableArray array50cm;
/** End [variable_arrays] */


// ==========================================================================
//  The Logger Object[s]
// ==========================================================================
/** Start [loggers] */
// Create the 1-minute  logger instance
Logger logger10cm;

// Create the 5-minute  logger instance
Logger logger20cm;
/** End [loggers] */

Logger logger50cm;


// ==========================================================================
//  Working Functions
// ==========================================================================
/** Start [working_functions] */
// Flashes the LED's on the primary board
void greenredflash(uint8_t numFlash = 4, uint8_t rate = 75) {
    for (uint8_t i = 0; i < numFlash; i++) {
        digitalWrite(greenLED, HIGH);
        digitalWrite(redLED, LOW);
        delay(rate);
        digitalWrite(greenLED, LOW);
        digitalWrite(redLED, HIGH);
        delay(rate);
    }
    digitalWrite(redLED, LOW);
}


/*
void flushsystem(uint8_t pin_number){
    digitalWrite(solenoid_pins[pin_number], HIGH);
    delay(1000);
    digitalWrite(pump_pin, HIGH);
    delay(FLUSH_TIME);
    digitalWrite(pump_pin, LOW);
    delay(MEASUREMENT_TIME);
    digitalWrite(solenoid_pins[pin_number], LOW);


}
*/
/** End [working_functions] */


// ==========================================================================
//  Arduino Setup Function
// ==========================================================================
/** Start [setup] */
void setup() {
    // Start the primary serial connection
    Serial.begin(serialBaud);

    // Print a start-up note to the first serial port
    Serial.print(F("Now running "));
    Serial.print(sketchName);
    Serial.print(F(" on Logger "));
    Serial.println(LoggerID);
    Serial.println();

    Serial.print(F("Using ModularSensors Library version "));
    Serial.println(MODULAR_SENSORS_VERSION);
    Serial.print(F("TinyGSM Library version "));
    Serial.println(TINYGSM_VERSION);
    Serial.println();

    // Start the serial connection with the modem
    modemSerial.begin(modemBaud);

    // Set up pins for the LED's
    pinMode(greenLED, OUTPUT);
    digitalWrite(greenLED, LOW);
    pinMode(redLED, OUTPUT);
    digitalWrite(redLED, LOW);

    /*
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
    */

    collector.begin();
    // Blink the LEDs to show the board is on and starting up
    greenredflash();

    // Set the timezones for the logger/data and the RTC
    // Logging in the given time zone
    Logger::setLoggerTimeZone(timeZone);
    // It is STRONGLY RECOMMENDED that you set the RTC to be in UTC (UTC+0)
    Logger::setRTCTimeZone(0);

    // Begin the variable array[s], logger[s], and publisher[s]
    array10cm.begin(variableCountGasMeasurement, variableList_gasMeasurement);
    array20cm.begin(variableCountGasMeasurement, variableList_gasMeasurement);
    array50cm.begin(variableCountGasMeasurement, variableList_gasMeasurement);
    logger10cm.begin(LoggerID, 1, &array10cm);
    logger20cm.begin(LoggerID, 1, &array20cm);
    logger50cm.begin(LoggerID, 1, &array50cm);
    logger10cm.setLoggerPins(wakePin, sdCardSSPin, sdCardPwrPin, buttonPin,
                             greenLED);
    logger20cm.setLoggerPins(wakePin, sdCardSSPin, sdCardPwrPin, buttonPin,
                             greenLED);
    logger50cm.setLoggerPins(wakePin, sdCardSSPin, sdCardPwrPin, buttonPin,
                             greenLED);


    // Turn on the modem
    modem.setModemLED(modemLEDPin);

    // Set up the sensors (do this directly on the VariableArray)
    array10cm.setupSensors();
    array20cm.setupSensors();
    array50cm.setupSensors();

    // Print out the current time
    Serial.print(F("Current RTC time is: "));
    Serial.println(Logger::formatDateTime_ISO8601(Logger::getNowUTCEpoch()));
    Serial.print(F("Current localized logger time is: "));
    Serial.println(Logger::formatDateTime_ISO8601(Logger::getNowLocalEpoch()));
    // Connect to the network
    if (modem.connectInternet()) {
        // Synchronize the RTC
        logger10cm.setRTClock(modem.getNISTTime());
        modem.updateModemMetadata();
        // Disconnect from the network
        modem.disconnectInternet();
    }
    // Turn off the modem
    modem.modemSleepPowerDown();

    // Give the loggers different file names
    // If we wanted to auto-generate the file name, that could also be done by
    // not calling this function, but in that case if both "loggers" have the
    // same logger id, they will end up with the same filename
    logger10cm.setFileName(FileName10cm);
    logger20cm.setFileName(FileName20cm);
    logger50cm.setFileName(FileName50cm);

    // Setup the logger files.  Specifying true will put a default header at
    // on to the file when it's created.
    // Because we've already called setFileName, we do not need to specify the
    // file name for this function.
    logger10cm.turnOnSDcard(
        true);  // true = wait for card to settle after power up
    logger10cm.createLogFile(true);  // true = write a new header
    logger20cm.createLogFile(true);  // true = write a new header
    logger50cm.createLogFile(true);  // true = write a new header
    logger10cm.turnOffSDcard(
        true);  // true = wait for internal housekeeping after write

    Serial.println(F("Logger setup finished!\n"));
    Serial.println(F("------------------------------------------"));
    Serial.println();

    // Call the processor sleep
    // Only need to do this for one of the loggers
    logger10cm.systemSleep();
}
/** End [setup] */


// ==========================================================================
//  Arduino Loop Function
// ==========================================================================
/** Start [loop] */
// Because of the way alarms work on the RTC, it will wake the processor and
// start the loop every minute exactly on the minute.
// The processor may also be woken up by another interrupt or level change on a
// pin - from a button or some other input.
// The "if" statements in the loop determine what will happen - whether the
// sensors update, testing mode starts, or it goes back to sleep.
void loop() {
    // Check if the current time is an even interval of the logging interval
    // For whichever logger we call first, use the checkInterval() function.
    if (logger10cm.checkInterval()) {
        // Print a line to show new reading
        Serial.println(F("--------------------->10<---------------------"));
        // Turn on the LED to show we're taking a reading
        digitalWrite(greenLED, HIGH);

        // Send power to all of the sensors (do this directly on the
        // VariableArray)
        Serial.print(F("Powering sensors...\n"));
        array10cm.sensorsPowerUp();
        logger10cm.watchDogTimer.resetWatchDog();
        // Wake up all of the sensors (do this directly on the VariableArray)
        Serial.print(F("Waking sensors...\n"));
        array10cm.sensorsWake();
        logger10cm.watchDogTimer.resetWatchDog();

        collector.getSample(0);
       
        // Update the values from all attached sensors (do this directly on the
        // VariableArray)
        Serial.print(F("Updating sensor values...\n"));
        array10cm.updateAllSensors();
        logger10cm.watchDogTimer.resetWatchDog();
        // Put sensors to sleep (do this directly on the VariableArray)
        Serial.print(F("Putting sensors back to sleep...\n"));
        array10cm.sensorsSleep();
        logger10cm.watchDogTimer.resetWatchDog();
        // Cut sensor power (do this directly on the VariableArray)
        Serial.print(F("Cutting sensor power...\n"));
        array10cm.sensorsPowerDown();
        logger10cm.watchDogTimer.resetWatchDog();

        // Stream the csv data to the SD card
        logger10cm.turnOnSDcard(true);
        logger10cm.logToSD();
        logger10cm.turnOffSDcard(true);
        logger10cm.watchDogTimer.resetWatchDog();

        // Turn off the LED
        digitalWrite(greenLED, LOW);
        // Print a line to show reading ended
        Serial.println(F("---------------------<10>---------------------\n"));
    }
    // Check if the already marked time is an even interval of the logging
    // interval For logger[s] other than the first one, use the
    // checkMarkedInterval() function.
    if (logger20cm.checkMarkedInterval()) {
        // Print a line to show new reading
        Serial.println(F("--------------------->20<---------------------"));
        // Turn on the LED to show we're taking a reading
        digitalWrite(redLED, HIGH);

        // Send power to all of the sensors (do this directly on the
        // VariableArray)
        Serial.print(F("Powering sensors...\n"));
        array20cm.sensorsPowerUp();
        logger10cm.watchDogTimer.resetWatchDog();
        // Wake up all of the sensors (do this directly on the VariableArray)
        Serial.print(F("Waking sensors...\n"));
        array20cm.sensorsWake();
        logger10cm.watchDogTimer.resetWatchDog();

        collector.getSample(1);
        // Update the values from all attached sensors (do this directly on the
        // VariableArray)
        Serial.print(F("Updating sensor values...\n"));
        array20cm.updateAllSensors();
        logger10cm.watchDogTimer.resetWatchDog();
        // Put sensors to sleep (do this directly on the VariableArray)
        Serial.print(F("Putting sensors back to sleep...\n"));
        array20cm.sensorsSleep();
        logger10cm.watchDogTimer.resetWatchDog();
        // Cut sensor power (do this directly on the VariableArray)
        Serial.print(F("Cutting sensor power...\n"));
        array20cm.sensorsPowerDown();
        logger10cm.watchDogTimer.resetWatchDog();

        // Stream the csv data to the SD card
        logger20cm.turnOnSDcard(true);
        logger20cm.logToSD();
        logger20cm.turnOffSDcard(true);
        logger10cm.watchDogTimer.resetWatchDog();

        // Turn off the LED
        digitalWrite(redLED, LOW);
        // Print a line to show reading ended
        Serial.println(F("--------------------<20>---------------------\n"));
    }


    // Check if the already marked time is an even interval of the logging
    // interval For logger[s] other than the first one, use the
    // checkMarkedInterval() function.
    if (logger50cm.checkMarkedInterval()) {
        // Print a line to show new reading
        Serial.println(F("--------------------->50<---------------------"));
        // Turn on the LED to show we're taking a reading
        digitalWrite(redLED, HIGH);

        // Send power to all of the sensors (do this directly on the
        // VariableArray)
        Serial.print(F("Powering sensors...\n"));
        array50cm.sensorsPowerUp();
        logger10cm.watchDogTimer.resetWatchDog();
        // Wake up all of the sensors (do this directly on the VariableArray)
        Serial.print(F("Waking sensors...\n"));
        array50cm.sensorsWake();
        logger10cm.watchDogTimer.resetWatchDog();

        collector.getSample(2);
        // Update the values from all attached sensors (do this directly on the
        // VariableArray)
        Serial.print(F("Updating sensor values...\n"));
        array50cm.updateAllSensors();
        logger10cm.watchDogTimer.resetWatchDog();
        // Put sensors to sleep (do this directly on the VariableArray)
        Serial.print(F("Putting sensors back to sleep...\n"));
        array50cm.sensorsSleep();
        logger10cm.watchDogTimer.resetWatchDog();
        // Cut sensor power (do this directly on the VariableArray)
        Serial.print(F("Cutting sensor power...\n"));
        array50cm.sensorsPowerDown();
        logger10cm.watchDogTimer.resetWatchDog();

        // Stream the csv data to the SD card
        logger50cm.turnOnSDcard(true);
        logger50cm.logToSD();
        logger50cm.turnOffSDcard(true);
        logger10cm.watchDogTimer.resetWatchDog();
        
        // Turn off the LED
        digitalWrite(redLED, LOW);
        // Print a line to show reading ended
        Serial.println(F("--------------------<50>---------------------\n"));
    }
    // Once a day, at noon, sync the clock
    if (Logger::markedLocalEpochTime % 86400 == 43200) {
        // Turn on the modem
        modem.modemWake();
        // Connect to the network
        if (modem.connectInternet()) {
            // Synchronize the RTC
            logger10cm.setRTClock(modem.getNISTTime());
            // Disconnect from the network
            modem.disconnectInternet();
        }
        // Turn off the modem
        modem.modemSleepPowerDown();
    }

    // Call the processor sleep
    // Only need to do this for one of the loggers
    logger10cm.systemSleep();
}
/** End [loop] */
