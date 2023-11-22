//modified from the ModularSensor AnalogElectricConductivity Sensor code
//currently not functional



#ifndef Alphasense_IRC_A1_H
#define Alphasense_IRC_A1_H

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <SPI.h>

class Alphasense_IRC_A1 {
public:
    Alphasense_IRC_A1();
    void begin();
    float getCO2PPM();

private:
    Adafruit_ADS1115 ads;
};

#endif // Alphasense_IRC_A1_H

// Header Guards
#ifndef SRC_SENSORS_ALPHASENSE_IRC_A1_H_
#define SRC_SENSORS_ALPHASENSE_IRC_A1_H_

#ifdef MS_ALPHASENSECO2_DEBUG
#define MS_DEBUGGING_STD "AlphasenseCO2"
#endif
#ifdef MS_ALPHASENSECO2_DEBUG_DEEP
#define MS_DEBUGGING_DEEP "AlphasenseCO2"
#endif
// Included Dependencies
#include "ModSensorDebugger.h"
#undef MS_DEBUGGING_STD
#undef MS_DEBUGGING_DEEP
#include "SensorBase.h"
#include "VariableBase.h"
#include "math.h"

/** @ingroup sensor_analog_cond */
/**@{*/

// Sensor Specific Defines
/// @brief Sensor::_numReturnedValues; we only get one value from the analog
/// conductivity sensor.
#define ALPHASENSE_IRC_A1_NUM_VARIABLES 1
/// @brief Sensor::_incCalcValues; we don't calculate any additional values -
/// though we recommend users include a temperature sensor and calculate
/// specific conductance in their own program.
#define ALPHASENSE_IRC_A1_INC_CALC_VARIABLES 0

/**
 * @anchor sensor_analog_cond_parts_timing
 * @name Sensor Timing
 * The timing for analog conductivity via resistance.
 */
/**@{*/
/// @brief Sensor::_warmUpTime_ms; giving 2ms for warm-up.
#define ALPHASENSE_IRC_A1_WARM_UP_TIME_MS 30000
/// @brief Sensor::_stabilizationTime_ms; we give just a tiny delay for
/// stabilization.
#define ALPHASENSE_IRC_A1_STABILIZATION_TIME_MS 10000
/**
 * @brief Sensor::_measurementTime_ms; we assume the analog voltage is measured
 * instantly.
 *
 * It's not really *quite* instantly, but it is very fast and the time to
 * measure is included in the read function.
 * On ATmega based boards (UNO, Nano, Mini, Mega), it takes about 100
 * microseconds (0.0001 s) to read an analog input, so the maximum reading rate
 * is about 10,000 times a second.
 */
#define ALPHASENSE_IRC_A1_MEASUREMENT_TIME_MS 0
/**@}*/


/**
 * @anchor sensor_analog_cond_parts_ec
 * @name Electrical Conductance
 * The humidity variable from an AOSong DHT
 * - Range: low 100's when open air, for short circuit: a high number
 * - Accuracy: needs determining for each combination of ADC. ADC_REF, and
 * series R. its designed as a very simple relative EC measurement
 *
 * {{ @ref AnalogElecConductivity_EC::AnalogElecConductivity_EC }}
 */
/**@{*/
/**
 * @brief Decimals places in string representation; EC should have 1
 *
 * Range of 0-3V3 with 10bit ADC - resolution of 0.003 = 3 µS/cm.
 */
#define ALPHASENSE_IRC_A1_CO2_RESOLUTION 1
/// @brief Sensor vensor variable number; EC is stored in sensorValues[0].
#define ALPHASENSE_IRC_A1_CO2_VAR_NUM 0
/// @brief Variable name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/variablename/);
/// "electricalConductivity"
#define ALPHASENSE_IRC_A1_CO2_VAR_NAME "carbonDioxide"
/// @brief Variable unit name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/units/);
/// "microsiemenPerCentimeter" (µS/cm)
#define ALPHASENSE_IRC_A1_CO2_UNIT_NAME "partPerMillion"
/// @brief Default variable short code; "anlgEc"
#define ALPHASENSE_IRC_A1_CO2_DEFAULT_CODE "anlgEc"
/**@}*/

#if !defined ANALOG_CO2_ADC_RESOLUTION
/**
 * @brief Default resolution (in bits) of the voltage measurement
 *
 * The default for all boards is 10, use a build flag to change this, if
 * necessary.
 */
#define ANALOG_CO2_ADC_RESOLUTION 10
#endif  // ANALOG_EC_ADC_RESOLUTION
/// @brief The maximum possible value of the ADC - one less than the resolution
/// shifted up one bit.
#define ANALOG_CO2_ADC_MAX ((1 << ANALOG_CO2_ADC_RESOLUTION) - 1)
/// @brief The maximum possible range of the ADC - the resolution shifted up one
/// bit.
#define ANALOG_CO2_ADC_RANGE (1 << ANALOG_CO2_ADC_RESOLUTION)

/* clang-format off */
#if !defined ANALOG_CO2_ADC_REFERENCE_MODE
#if defined (ARDUINO_ARCH_AVR) || defined (DOXYGEN)
/**
 * @brief The voltage reference mode for the processor's ADC.
 *
 * For an AVR board, this must be one of:
 * - `DEFAULT`: the default built-in analog reference of 5 volts (on 5V Arduino
 * boards) or 3.3 volts (on 3.3V Arduino boards)
 * - `INTERNAL`: a built-in reference, equal to 1.1 volts on the ATmega168 or
 * ATmega328P and 2.56 volts on the ATmega32U4 and ATmega8 (not available on the
 * Arduino Mega)
 * - `INTERNAL1V1`: a built-in 1.1V reference (Arduino Mega only)
 * - `INTERNAL2V56`: a built-in 2.56V reference (Arduino Mega only)
 * - `EXTERNAL`: the voltage applied to the AREF pin (0 to 5V only) is used as the
 * reference.
 *
 * If not set on an AVR board `DEFAULT` is used.
 *
 * For the best accuracy, use an `EXTERNAL` reference with the AREF pin
 * connected to the power supply for the EC sensor.
 */
#define ANALOG_CO2_ADC_REFERENCE_MODE DEFAULT
#endif
#if defined (ARDUINO_ARCH_SAMD) || defined (DOXYGEN)
/**
 * @brief The voltage reference mode for the processor's ADC.
 *
 * For a SAMD board, this must be one of:
 * - `AR_DEFAULT`: the default built-in analog reference of 3.3V
 * - `AR_INTERNAL`: a built-in 2.23V reference
 * - `AR_INTERNAL1V0`: a built-in 1.0V reference
 * - `AR_INTERNAL1V65`: a built-in 1.65V reference
 * - `AR_INTERNAL2V23`: a built-in 2.23V reference
 * - `AR_EXTERNAL`: the voltage applied to the AREF pin is used as the reference
 *
 * If not set on an SAMD board `AR_DEFAULT` is used.
 *
 * For the best accuracy, use an `EXTERNAL` reference with the AREF pin
 * connected to the power supply for the EC sensor.
 *
 * @see https://www.arduino.cc/reference/en/language/functions/analog-io/analogreference/
 */
#define ANALOG_EC_ADC_REFERENCE_MODE AR_DEFAULT
#endif
#if !defined ANALOG_CO2_ADC_REFERENCE_MODE
#error The processor ADC reference type must be defined!
#endif  // ANALOG_CO2_ADC_REFERENCE_MODE
#endif  // ARDUINO_ARCH_SAMD
/* clang-format on */

#if !defined RSERIES_OHMS_DEF
/**
 * @brief The default resistance (in ohms) of the measuring resistor.
 * This should not be less than 300 ohms when measuring EC in water.
 */
#define RSERIES_OHMS_DEF 250
#endif  // RSERIES_OHMS_DEF

#if !defined SENSOREC_KONST_DEF
/**
 * @brief Cell Constant For EC Measurements.
 *
 * This should be measured following the calibration example on
 * https://hackaday.io/project/7008-fly-wars-a-hackers-solution-to-world-hunger/log/24646-three-dollar-ec-ppm-meter-arduino.
 *
 * Mine was around 2.9 with plugs being a standard size they should all be
 * around the same. If you get bad readings you can use the calibration script
 * and fluid to get a better estimate for K.
 * Default to 1.0, and can be set at startup.
 */
#define SENSOREC_KONST_DEF 1.0
#endif  // SENSOREC_KONST_DEF

/**
 * @brief Class for the analog Electrical Conductivity monitor
 *
 * @ingroup sensor_analog_cond
 */
class AlphasenseCO2 : public Sensor {
 public:
    /**
     * @brief Construct a new AnalogElecConductivity object.
     *
     * @param powerPin The port pin providing power to the EC probe.
     * Needs to be switched, and assumed to be same V as the dataPin's ADC.
     * @param dataPin The processor ADC port pin to read the voltage from the EC
     * probe.  Not all processor pins can be used as analog pins.  Those usable
     * as analog pins generally are numbered with an "A" in front of the number
     * - ie, A1.
     * @param Rseries_ohms The resistance of the resistor series (R) in the
     * line; optional with default value of 499.
     * @param sensorEC_Konst The cell constant for the sensing circuit; optional
     * with default value of 2.88 - which is what has been measured for a
     * typical standard sized lamp-type plug.
     * @param measurementsToAverage The number of measurements to average;
     * optional with default value of 1.
     */
    AlphasenseCO2(int8_t powerPin, int8_t dataPin,
                           float   Rseries_ohms          = RSERIES_OHMS_DEF,
                           float   sensorEC_Konst        = SENSOREC_KONST_DEF,
                           uint8_t measurementsToAverage = 1);

    /**
     * @brief Destroy the AnalogElecConductivity object - no action needed.
     */
    ~AlphasenseCO2();

    /**
     * @brief Report the sensor info.
     *
     * @return **String** Text describing how the sensor is attached to the mcu.
     */
    String getSensorLocation(void) override;

    /**
     * @copydoc Sensor::addSingleMeasurementResult()
     */
    bool addSingleMeasurementResult(void) override;

    /**
     * @brief Set EC constants for internal calculations.
     * Needs to be set at startup if different from defaults
     *
     * @param sourceResistance_ohms series R used in calculations for EC
     *
     * other possible K, not specified yet:
     *    float  appliedV_V,
     *    uint8_t probeType
     */
    void setEC_k(float sourceResistance_ohms) {
        _Rseries_ohms = sourceResistance_ohms;
    }

    /**
     * @brief reads the calculated EC from an analog pin using the analog pin
     * number set in the constructor.
     *
     * @return The electrical conductance value
     */
    float readCO2(void);
    /**
     * @brief reads the calculated EC from an analog pin.
     *
     * @param analogPinNum Analog port pin number
     * @return The electrical conductance value
     */
    float readCO2(uint8_t analogPinNum);

 private:
    int8_t _CO2PowerPin;
    int8_t _CO2AdcPin;

    float* _ptrWaterTemperature_C;

    /// @brief The resistance of the circiut resistor plus any series port
    /// resistance
    float _Rseries_ohms = RSERIES_OHMS_DEF;

    /// @brief the cell constant for the circuit
    float _sensorCO2_Konst = SENSOREC_KONST_DEF;
};

/**
 * @brief The variable class used for electricalConductivity measured using an
 * analog pin connected to electrodes submerged in the medium
 *
 * @ingroup sensor_analog_cond
 *
 */
class Alphasense_CO2 : public Variable {
 public:
    /**
     * @brief Construct a new  AnalogElecConductivity_EC object.
     *
     * @param parentSense The parent AnalogElecConductivity providing the result
     * values.
     * @param uuid A universally unique identifier (UUID or GUID) for the
     * variable; optional with the default value of an empty string.
     * @param varCode A short code to help identify the variable in files;
     * optional with a default value of "anlgEc".
     */
    Alphasense_CO2(
        AlphasenseCO2* parentSense, const char* uuid = "",
        const char* varCode = ALPHASENSE_IRC_A1_CO2_DEFAULT_CODE)
        : Variable(parentSense,
                   (const uint8_t)ALPHASENSE_IRC_A1_CO2_VAR_NUM,
                   (uint8_t)ALPHASENSE_IRC_A1_CO2_RESOLUTION,
                   ALPHASENSE_IRC_A1_CO2_VAR_NAME,
                   ALPHASENSE_IRC_A1_CO2_UNIT_NAME, varCode, uuid) {}

    /**
     * @brief Construct a new AnalogElecConductivity_EC object.
     *
     * @note This must be tied with a parent AnalogElecConductivity before it
     * can be used.
     */
    Alphasense_CO2()
        : Variable((const uint8_t)ALPHASENSE_IRC_A1_CO2_VAR_NUM,
                   (uint8_t)ALPHASENSE_IRC_A1_CO2_RESOLUTION,
                   ALPHASENSE_IRC_A1_CO2_VAR_NAME,
                   ALPHASENSE_IRC_A1_CO2_UNIT_NAME,
                   ALPHASENSE_IRC_A1_CO2_DEFAULT_CODE) {}
    /**
     * @brief Destroy the AnalogElecConductivity_EC object - no action needed.
     */
    ~Alphasense_CO2() {}
};
/**@}*/
#endif  // SRC_SENSORS_ANALOGELECCONDUCTIVITY_H_
