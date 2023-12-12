# 1 "/var/folders/tx/14mbtmrs0rq2b6mkfxdh54r00000gp/T/tmpl030n4_e"
#include <Arduino.h>
# 1 "/Users/aaufdenkampe/Documents/Arduino/Soil-Sensing-Device/examples/AlphasenseCO2_simple_logging/AlphasenseCO2_simple_logging.ino"
# 25 "/Users/aaufdenkampe/Documents/Arduino/Soil-Sensing-Device/examples/AlphasenseCO2_simple_logging/AlphasenseCO2_simple_logging.ino"
#include <Arduino.h>



#include <EnableInterrupt.h>


#include <ModularSensors.h>
# 41 "/Users/aaufdenkampe/Documents/Arduino/Soil-Sensing-Device/examples/AlphasenseCO2_simple_logging/AlphasenseCO2_simple_logging.ino"
const char* sketchName = "AlphasenseCO2_simple_logging.ino";

const char* LoggerID = "AlphasenseCO2_simple_logging";

const uint8_t loggingInterval = 2;

const int8_t timeZone = -6;




const int32_t serialBaud = 115200;
const int8_t greenLED = 8;
const int8_t redLED = 9;
const int8_t buttonPin = 21;
const int8_t wakePin = 31;



const int8_t sdCardPwrPin = -1;
const int8_t sdCardSSPin = 12;
const int8_t sensorPowerPin = 22;







#include <sensors/ProcessorStats.h>


const char* mcuBoardVersion = "v1.1";
ProcessorStats mcuBoard(mcuBoardVersion);



Variable* mcuBoardBatt = new ProcessorStats_Battery(
    &mcuBoard, "12345678-abcd-1234-ef00-1234567890ab");
Variable* mcuBoardAvailableRAM = new ProcessorStats_FreeRam(
    &mcuBoard, "12345678-abcd-1234-ef00-1234567890ab");
Variable* mcuBoardSampNo = new ProcessorStats_SampleNumber(
    &mcuBoard, "12345678-abcd-1234-ef00-1234567890ab");







#include <sensors/MaximDS3231.h>


MaximDS3231 ds3231(1);


Variable* ds3231Temp =
    new MaximDS3231_Temp(&ds3231, "12345678-abcd-1234-ef00-1234567890ab");
# 118 "/Users/aaufdenkampe/Documents/Arduino/Soil-Sensing-Device/examples/AlphasenseCO2_simple_logging/AlphasenseCO2_simple_logging.ino"
#include <sensors/AlphasenseCO2.h>


const int8_t AlphasenseCO2Power = sensorPowerPin;
const uint8_t AlphasenseCO2ADSi2c_addr = 0x48;
const uint8_t AlphasenseCO2NumberReadings = 28;




AlphasenseCO2 alphasenseCO2(AlphasenseCO2Power, AlphasenseCO2ADSi2c_addr,
                            AlphasenseCO2NumberReadings);


Variable* alphasenseCO2CO2 = new AlphasenseCO2_CO2(
    &alphasenseCO2, "12345678-abcd-1234-ef00-1234567890ab");
Variable* alphasenseCO2voltage = new AlphasenseCO2_Voltage(
    &alphasenseCO2, "12345678-abcd-1234-ef00-1234567890ab");
# 147 "/Users/aaufdenkampe/Documents/Arduino/Soil-Sensing-Device/examples/AlphasenseCO2_simple_logging/AlphasenseCO2_simple_logging.ino"
Variable* variableList[] = {
    mcuBoardSampNo,
    mcuBoardAvailableRAM,
    mcuBoardBatt,
    ds3231Temp,
    alphasenseCO2CO2,
    alphasenseCO2voltage,
};

int variableCount = sizeof(variableList) / sizeof(variableList[0]);

VariableArray varArray(variableCount, variableList);
# 167 "/Users/aaufdenkampe/Documents/Arduino/Soil-Sensing-Device/examples/AlphasenseCO2_simple_logging/AlphasenseCO2_simple_logging.ino"
Logger dataLogger(LoggerID, loggingInterval, &varArray);
# 176 "/Users/aaufdenkampe/Documents/Arduino/Soil-Sensing-Device/examples/AlphasenseCO2_simple_logging/AlphasenseCO2_simple_logging.ino"
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
float getBatteryVoltage();
void setup();
void loop();
#line 190 "/Users/aaufdenkampe/Documents/Arduino/Soil-Sensing-Device/examples/AlphasenseCO2_simple_logging/AlphasenseCO2_simple_logging.ino"
float getBatteryVoltage() {
    if (mcuBoard.sensorValues[0] == -9999) mcuBoard.update();
    return mcuBoard.sensorValues[0];
}






void setup() {


    Serial.begin(serialBaud);


    Serial.print(F("\n\nNow running "));
    Serial.print(sketchName);
    Serial.print(F(" on Logger "));
    Serial.println(LoggerID);
    Serial.println();

    Serial.print(F("Using ModularSensors Library version "));
    Serial.println(MODULAR_SENSORS_VERSION);




    pinMode(greenLED, OUTPUT);
    digitalWrite(greenLED, LOW);
    pinMode(redLED, OUTPUT);
    digitalWrite(redLED, LOW);

    greenredflash();





    Logger::setLoggerTimeZone(timeZone);

    Logger::setRTCTimeZone(0);


    dataLogger.setLoggerPins(wakePin, sdCardSSPin, sdCardPwrPin, buttonPin,
                             greenLED);


    dataLogger.begin();





    if (getBatteryVoltage() > 3.4) {
    Serial.println(F("Setting up sensors..."));
    varArray.setupSensors();
    }
# 256 "/Users/aaufdenkampe/Documents/Arduino/Soil-Sensing-Device/examples/AlphasenseCO2_simple_logging/AlphasenseCO2_simple_logging.ino"
    if (getBatteryVoltage() > 3.4) {
        Serial.println(F("Setting up file on SD card"));
        dataLogger.turnOnSDcard(true);

        dataLogger.createLogFile(true);
        dataLogger.turnOffSDcard(true);

    }




    Serial.println(F("Putting processor to sleep\n"));
    dataLogger.systemSleep();

}






void loop() {
    dataLogger.logData();
}