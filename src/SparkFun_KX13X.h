/**
 * @file sfDevKX13X.h
 * @brief Arduino-specific implementation for the SparkFun KX13X Sensor family.
 *
 * @details
 * This file provides the Arduino-specific implementation of the KX13X driver
 * classes. The SfeKX134ArdI2C, SfeKX132ArdI2C, SfeKX134ArdSPI, and SfeKX132ArdSPI
 * classes inherit from the base driver classes and implement the I2C or SPI
 * communication interfaces using Arduino's Wire and SPI libraries.
 *
 * Key features:
 * - Arduino I2C and SPI initialization
 * - Connection verification
 * - Toolkit integration
 *
 * @section Classes
 * - SfeKX134ArdI2C: I2C implementation for KX134
 * - SfeKX132ArdI2C: I2C implementation for KX132
 * - SfeKX134ArdSPI: SPI implementation for KX134
 * - SfeKX132ArdSPI: SPI implementation for KX132
 *
 * @section Dependencies
 * - Arduino.h
 * - SparkFun_Toolkit.h
 * - sfTk/sfDevKX13X.h
 *
 * @author Elias Santistevan @SparkFun Electronics
 * @date October 2022
 * @copyright Copyright (c) 2022-2025, SparkFun Electronics Inc. All rights reserved.
 *
 * @section License
 * SPDX-License-Identifier: MIT
 *
 * @section Product_Links
 * - Qwiic KX134: https://www.sparkfun.com/sparkfun-triple-axis-accelerometer-breakout-kx134-qwiic.html
 * - Qwiic KX132: https://www.sparkfun.com/sparkfun-triple-axis-accelerometer-breakout-kx132-qwiic.html
 *
 * @see https://github.com/sparkfun/SparkFun_KX13X_Arduino_Library
 */

#pragma once

// clang-format off
#include <SparkFun_Toolkit.h>
#include "sfTk/sfDevKX13X.h"
#include <Arduino.h>
// clang-format on

/**
 * @class SfeKX134ArdI2C
 * @brief Arduino I2C implementation for the KX134 sensor.
 *
 * This class provides Arduino-specific I2C communication for the KX134 sensor.
 * It inherits from the base driver and implements the I2C interface using Arduino's Wire library.
 * The class manages device addressing and connection verification.
 *
 * Example usage:
 * @code
 * SfeKX134ArdI2C sensor;
 * if (sensor.begin()) {
 *     // Sensor initialized successfully
 * }
 * @endcode
 *
 * @note Uses the Arduino Wire library for I2C communication.
 */
class SfeKX134ArdI2C : public sfDevKX134
{
public:
    SfeKX134ArdI2C() {}

    /**
     * @brief Initializes the KX134 sensor with I2C communication.
     * @param address I2C address of the sensor.
     * @param wirePort Reference to the Wire port.
     * @return True if initialization succeeds, false otherwise.
     */
    bool begin(const uint8_t &address = KX13X_ADDRESS_HIGH, TwoWire &wirePort = Wire)
    {
        if (_theI2CBus.init(wirePort, address) != ksfTkErrOk)
            return false;

        setCommunicationBus(&_theI2CBus);

        if (!isConnected())
            return false;

        return true;
    }

    /**
     * @brief Checks if the KX134 sensor is connected and responding.
     * @return True if connected, false otherwise.
     */
    bool isConnected(void)
    {
        if (_theI2CBus.ping() != ksfTkErrOk)
            return false;

        // Check the device ID
        return (getUniqueID() == KX134_WHO_AM_I);
    }

    /**
     * @brief Returns the I2C device address.
     * @return The I2C address.
     */
    uint8_t getDeviceAddress(void)
    {
        return _theI2CBus.address();
    }

private:
    sfTkArdI2C _theI2CBus;
};

/**
 * @class SfeKX132ArdI2C
 * @brief Arduino I2C implementation for the KX132 sensor.
 *
 * This class provides Arduino-specific I2C communication for the KX132 sensor.
 * It inherits from the base driver and implements the I2C interface using Arduino's Wire library.
 * The class manages device addressing and connection verification.
 */
class SfeKX132ArdI2C : public sfDevKX132
{
public:
    SfeKX132ArdI2C() {}

    /**
     * @brief Initializes the KX132 sensor with I2C communication.
     * @param address I2C address of the sensor.
     * @param wirePort Reference to the Wire port.
     * @return True if initialization succeeds, false otherwise.
     */
    bool begin(const uint8_t &address = KX13X_ADDRESS_HIGH, TwoWire &wirePort = Wire)
    {
        if (_theI2CBus.init(wirePort, address) != ksfTkErrOk)
            return false;

        setCommunicationBus(&_theI2CBus);

        if (!isConnected())
            return false;
        return true;
    }

    /**
     * @brief Checks if the KX132 sensor is connected and responding.
     * @return True if connected, false otherwise.
     */
    bool isConnected(void)
    {
        if (_theI2CBus.ping() != ksfTkErrOk)
            return false;

        // Check the device ID
        return (getUniqueID() == KX132_WHO_AM_I);
    }

    /**
     * @brief Returns the I2C device address.
     * @return The I2C address.
     */
    uint8_t getDeviceAddress(void)
    {
        return _theI2CBus.address();
    }

private:
    sfTkArdI2C _theI2CBus;
};


/**
 * @class SfeKX134ArdSPI
 * @brief Arduino SPI implementation for the KX134 sensor.
 *
 * This class provides SPI communication for the KX134 sensor using Arduino's SPI library.
 * It inherits from the base driver and manages SPI initialization and connection verification.
 */
class SfeKX134ArdSPI : public sfDevKX134
{
public:
    /**
     * @brief Initializes the KX134 sensor with SPI communication.
     * @param csPin Chip select pin.
     */
    bool begin(uint8_t csPin)
    {
        setCommunicationBus(&_theSPIBus);
        _theSPIBus.init(csPin, true);
        return isConnected();
    }

    bool begin(SPIClass &spiPort, SPISettings kxSettings, uint8_t csPin)
    {
        setCommunicationBus(&_theSPIBus);
        _theSPIBus.init(spiPort, kxSettings, csPin, true);
        return isConnected();
    }

    /**
     * @brief Checks if the KX134 sensor is connected and responding.
     * @return True if connected, false otherwise.
     */
    bool isConnected(void)
    {
        return (getUniqueID() == KX134_WHO_AM_I);
    }

private:
    sfTkArdSPI _theSPIBus;
};

/**
 * @class SfeKX132ArdSPI
 * @brief Arduino SPI implementation for the KX132 sensor.
 *
 * This class provides SPI communication for the KX132 sensor using Arduino's SPI library.
 * It inherits from the base driver and manages SPI initialization and connection verification.
 */
class SfeKX132ArdSPI : public sfDevKX132
{
public:
    /**
     * @brief Initializes the KX132 sensor with SPI communication.
     * @param csPin Chip select pin.
     */
    bool begin(uint8_t csPin)
    {
        // Setup a SPI object and pass into the superclass
        setCommunicationBus(&_theSPIBus);

        // Initialize the SPI bus class with the chip select pin, SPI port defaults to SPI,
        // and SPI settings are set to class defaults.
        _theSPIBus.init(csPin, true);

        return isConnected();
    }

    bool begin(SPIClass &spiPort, SPISettings kxSettings, uint8_t csPin)
    {
        // Setup a SPI object and pass into the superclass
        setCommunicationBus(&_theSPIBus);

        // Initialize the SPI bus class with provided SPI port, SPI setttings, and chip select pin.
        _theSPIBus.init(spiPort, kxSettings, csPin, true);

        return isConnected();
    }

    /**
     * @brief Checks if the KX132 sensor is connected and responding.
     * @return True if connected, false otherwise.
     */
    bool isConnected(void)
    {
        // Check the device ID
        return (getUniqueID() == KX132_WHO_AM_I);
    }

    private:
        sfTkArdSPI _theSPIBus;
};