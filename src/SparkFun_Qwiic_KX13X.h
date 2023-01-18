// SparkFun_Qwiic_KX13X.h
//
// This is a library written for the SparkFun Triple Axis Accelerometer - KX132/KX134
//
// SparkFun sells these boards at its website: www.sparkfun.com
//
// Do you like this library? Help support SparkFun. Buy a board!
//
// Written by Elias Santistevan @ SparkFun Electronics, October 2022
//
// Product:
// SparkFun Triple Axis Accelerometer - KX132/KX134 (Qwiic)
//	* KX132 - https://www.sparkfun.com/products/17871
//	* KX134 - https://www.sparkfun.com/products/17589
//
//  Repository:
//		https://github.com/sparkfun/SparkFun_KX13X_Arduino_Library
//
// SparkFun code, firmware, and software is released under the MIT
// License(http://opensource.org/licenses/MIT).
//
// SPDX-License-Identifier: MIT
//
//    The MIT License (MIT)
//
//    Copyright (c) 2022 SparkFun Electronics
//    Permission is hereby granted, free of charge, to any person obtaining a
//    copy of this software and associated documentation files (the "Software"),
//    to deal in the Software without restriction, including without limitation
//    the rights to use, copy, modify, merge, publish, distribute, sublicense,
//    and/or sell copies of the Software, and to permit persons to whom the
//    Software is furnished to do so, subject to the following conditions: The
//    above copyright notice and this permission notice shall be included in all
//    copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED
//    "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//    NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
//    PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
//    ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

// The following class implements the	methods to set, get, and read from the SparkFun Triple
// Axis Acceleromter - KX132/KX134.

#pragma once

#include <SPI.h>
#include <Wire.h>
#include "SparkFun_KX13X_regs.h"
#include "sfe_bus.h"

#define KX13X_ADDRESS_HIGH 0x1F
#define KX13X_ADDRESS_LOW 0x1E

#define KX132_WHO_AM_I 0x3D
#define KX134_WHO_AM_I 0x46

// CNTL1 GSEL<1:0>
#define SFE_KX132_RANGE2G 0x00
#define SFE_KX132_RANGE4G 0x01
#define SFE_KX132_RANGE8G 0x02
#define SFE_KX132_RANGE16G 0x03

// CNTL1 GSEL<1:0>
#define SFE_KX134_RANGE8G 0x00
#define SFE_KX134_RANGE16G 0x01
#define SFE_KX134_RANGE32G 0x02
#define SFE_KX134_RANGE64G 0x03

#define TOTAL_ACCEL_DATA_8BIT 3
#define TOTAL_ACCEL_DATA_16BIT 6

#define XLSB 0
#define XMSB 1
#define YLSB 2
#define YMSB 3
#define ZLSB 4
#define ZMSB 5

#define SPI_READ 0x80 // OR'ed at most sig BIT with register address

#define DEFAULT_SETTINGS 0xC0 // CNTL1: Hi-Performance mode; Data-Ready disabled; minimum G-range; Tap & Tilt disabled
#define INT_SETTINGS 0xE0 // CNTL1: Hi-Performance mode; Data-Ready enabled; minimum G-range; Tap & Tilt disabled
#define SOFT_INT_SETTINGS 0xE1
#define BUFFER_SETTINGS 0xE2
#define TILT_SETTINGS 0xE3

struct outputData
{
  float xData;
  float yData;
  float zData;
};

struct rawOutputData
{
  int16_t xData;
  int16_t yData;
  int16_t zData;
};

class QwDevKX13X
{
public:
  QwDevKX13X() : _i2cAddress{0}, _cs{0} {};

  int writeRegisterRegion(uint8_t reg, uint8_t *data, uint16_t length);
  int writeRegisterByte(uint8_t reg, uint8_t data);
  int readRegisterRegion(uint8_t reg, uint8_t *data, uint16_t length);
  void setCommunicationBus(sfe_KX13X::QwIDeviceBus &theBus, uint8_t i2cAddress);
  void setCommunicationBus(sfe_KX13X::QwIDeviceBus &theBus);

  uint8_t getUniqueID();
  bool initialize(uint8_t settings = DEFAULT_SETTINGS);

  // General Settings
  bool enableAccel(bool enable = true);
  bool softwareReset();
  int8_t getOperatingMode();
  bool setRange(uint8_t);
  bool setInterruptPin(bool enable, uint8_t polarity = 0, uint8_t pulseWidth = 0, bool latchControl = false);
  bool enableDataEngine(bool enable = true);
  bool setOutputDataRate(uint8_t);
  float getOutputDataRate();
  bool dataReady();
  bool runCommandTest();
  uint8_t readAccelState();
  bool getRawAccelData(rawOutputData *);
  bool getRawAccelRegisterData(rawOutputData *);
  bool getRawAccelBufferData(rawOutputData *, int sixteenBit = -1); // Set sixteenBit to 0 to read 8-bit data. Set to 1 to read 16-bit data.
  float readOutputDataRate();

  // Tap/Double settings
  bool enableTapEngine(bool enable = true);
  bool setTapDataRate(uint8_t rate);
  bool enableDirecTapInterupt(bool enable = true);
  bool enableDoubleTapInterrupt(bool enable = true);

  // Tilt Settings
  bool enableTiltEngine(bool enable = true);
  bool setTiltDataRate(uint8_t rate);

  // Wake/Sleep Settings
  bool setWakeDataRate(uint8_t rate);
  bool enableSleepEngine(bool enable = true);
  bool enableWakeEngine(bool enable = true);
  bool forceWake();
  bool forceSleep();

  // Interrupt Settings
  bool configureInterruptPin(uint8_t pinVal);
  bool routeHardwareInterrupt(uint8_t, uint8_t pin = 1);
  bool enablePhysInterrupt(bool enable = true, uint8_t pin = 1);
  bool setPinMode(bool activeHigh = true, uint8_t pin = 1);
  bool setLatchControl(bool pulsed = true, uint8_t pin = 1);
  bool setPulseWidth(uint8_t width, uint8_t pin = 1);
  bool clearInterrupt();
  bool tapDetected();
  int8_t getDirection();
  bool unknownTap();
  bool waterMarkReached();
  bool bufferFull();
  bool freeFall();
  bool doubleTapDetected();
  bool tiltChange();

  // Buffer Setttings
  bool setBufferThreshold(uint8_t);
  bool setBufferOperationMode(uint8_t operationMode);
  bool setBufferResolution(bool sixteenBit = true);
  bool enableBufferInt(bool enable = true);
  bool enableSampleBuffer(bool enable = true);
  uint16_t getSampleLevel();
  bool clearBuffer();

  rawOutputData rawAccelData;

protected:
  sfe_KX13X::QwIDeviceBus *_sfeBus;
  uint8_t _i2cAddress;
  uint8_t _cs;
  int _range = -1; // Keep a local copy of the range. Default to "unknown" (-1).
};

class QwDevKX132 : public QwDevKX13X
{
public:
  QwDevKX132(){};

  bool init(void);
  bool getAccelData(outputData *userData);
  bool convAccelData(outputData *userAccel, rawOutputData *rawAccelData);

  // KX132 conversion values	- all 16 bit resolution
  const double convRange2G = .000061;
  const double convRange4G = .000122;
  const double convRange8G = .000244;
  const double convRange16G = .000488;

private:
};

class QwDevKX134 : public QwDevKX13X
{
public:
  QwDevKX134(){};

  bool init(void);
  bool getAccelData(outputData *userData);
  bool convAccelData(outputData *userAccel, rawOutputData *rawAccelData);

  // KX134 conversion values	- all 16 bit resolution
  const double convRange8G = .000244;
  const double convRange16G = .000488;
  const double convRange32G = .000977;
  const double convRange64G = .001953;

private:
};
