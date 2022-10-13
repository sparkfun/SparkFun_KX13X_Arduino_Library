/******************************************************************************
SparkFun_Qwiic_KX13X.h
Elias Santistevan @ SparkFun Electronics
Original Creation Date: March 2021

This code is Lemonadeware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#pragma once

#include <SPI.h>
#include <Wire.h>
#include "SparkFun_KX13X_regs.h"
#include "sfe_bus.h"

#define KX13X_ADDRESS_HIGH 0x1F
#define KX13X_ADDRESS_LOW 0x1E

#define KX132_WHO_AM_I  0x3D
#define KX134_WHO_AM_I  0x46

#define KX132_RANGE2G  0x00
#define KX132_RANGE4G  0x01
#define KX132_RANGE8G  0x02
#define KX132_RANGE16G 0x03

#define KX134_RANGE8G  0x00
#define KX134_RANGE16G 0x01
#define KX134_RANGE32G 0x02
#define KX134_RANGE64G 0x03

#define TOTAL_ACCEL_DATA_8BIT 3 

#define XLSB 0
#define XMSB 1
#define YLSB 2
#define YMSB 3
#define ZLSB 4
#define ZMSB 5

#define SPI_READ 0x80 // OR'ed at most sig BIT with register address

#define DEFAULT_SETTINGS 0xC0  
#define INT_SETTINGS 0xE0  
#define SOFT_INT_SETTINGS 0xE1  
#define BUFFER_SETTINGS 0xE2  
#define TILT_SETTINGS 0xE3  

#define BUFFER_16BIT_SAMPLES 0x01
#define BUFFER_8BIT_SAMPLES 0x00
#define BUFFER_MODE_FIFO 0x00
#define BUFFER_MODE_STREAM 0x01
#define BUFFER_MODE_TRIGGER 0x02

struct outputData { 
  float xData;
  float yData;
  float zData;
};

struct rawOutputData {
  int16_t xData;
  int16_t yData;
  int16_t zData;
};


class QwDevKX13X
{
  public:

    QwDevKX13X() : _i2cAddress{0}, _cs{0} {};

    //////////////////////////////////////////////////////////////////////////////////
    // writeRegisterRegion()
    //
    //
    //  Parameter    Description
    //  ---------    -----------------------------
    //  reg          register to read from
    //  data         Array to store data in
		//  length       Size of data in bytes (8 bits): 2 bytes = length of two
    //  retval       -1 = error, 0 = success
    int writeRegisterRegion(uint8_t reg, uint8_t *data, uint16_t length);

    //////////////////////////////////////////////////////////////////////////////////
    // writeRegisterByte()
    //
    //
    //  Parameter    Description
    //  ---------    -----------------------------
    //  reg          register to read from
    //  data         Array to store data in
    //  retval       -1 = error, 0 = success
		//
    int writeRegisterByte(uint8_t reg, uint8_t data);

    //////////////////////////////////////////////////////////////////////////////////
    // readRegisterRegion()
    //
    //
    //  Parameter    Description
    //  ---------    -----------------------------
    //  reg          register to read from
    //  data         Array to store data in
    //  length       Length of the data to read
    //  retval       -1 = error, 0 = success

    int readRegisterRegion(uint8_t reg, uint8_t *data, uint16_t length);

    //////////////////////////////////////////////////////////////////////////////////
    // setCommunicationBus()
    //
    // Called to set the Communication Bus object to use
    //
    //  Parameter    Description
    //  ---------    -----------------------------
    //  theBus       The Bus object to use
    //  idBus        The bus ID for the target device.
    //

		void setCommunicationBus(QwIDeviceBus &theBus, uint8_t i2cAddress);
		void setCommunicationBus(QwIDeviceBus &theBus);

		uint8_t getUniqueID();

		bool initialize(uint8_t settings = DEFAULT_SETTINGS);
		
		// General Settings
		bool enableAccel(bool enable = true);
		bool softwareReset();
		int8_t getOperatingMode();
    bool setRange(uint8_t);
    bool setInterruptPin(bool enable, uint8_t polarity = 0, uint8_t pulseWidth = 0, bool latchControl = false);
		bool enableDataEngine(bool enable = true);
		bool enableTapEngine(bool enable = true);
		bool enableTiltEngine(bool enable = true);
    bool setOutputDataRate(uint8_t);
		float getOutputDataRate();
    bool routeHardwareInterrupt(uint8_t, uint8_t pin = 1);
    bool dataReady();
		bool runCommandTest();
    uint8_t readAccelState();
    bool getRawAccelData(rawOutputData*);
		bool setTapDataRate(uint8_t rate);
    float readOutputDataRate();

		// Interrupt Settings
		bool configureInterruptPin(uint8_t pinVal);
		bool enablePhysInterrupt(bool enable = true, uint8_t pin = 1);
		bool setPinMode(bool activeLow = true, uint8_t pin = 1);
		bool setLatchControl(bool latch = true, uint8_t pin = 1);
		bool setPulseWidth(uint8_t width, uint8_t pin = 1);
		bool enableDirecTapInterupt(bool enable = true);
		bool enableDoubleTapInterrupt(bool enable = true);
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

		QwIDeviceBus *_sfeBus;			 //The generic connection to user's chosen SPI hardware
		uint8_t _i2cAddress; 
		uint8_t _cs;
};
    
class QwDevKX132 : public QwDevKX13X
{
  public:

		QwDevKX132() {};

		bool init(void);
		bool getAccelData(outputData *userData);
		bool convAccelData(outputData *userAccel, rawOutputData *rawAccelData);

		//KX132 conversion values	- all 16 bit resolution
    const double convRange2G =  .00006;
    const double convRange4G =  .00012;
    const double convRange8G =  .00024;
    const double convRange16G = .00048;

	private: 
};

class QwDevKX134 : public QwDevKX13X
{
  public:

		QwDevKX134() {};

		bool init(void);
		bool getAccelData(outputData *userData);
		bool convAccelData(outputData *userAccel, rawOutputData *rawAccelData);

		//KX134 conversion values	- all 16 bit resolution
    const double convRange8G =  .00024;
    const double convRange16G = .00049;
    const double convRange32G = .00098;
    const double convRange64G = .00195;

	private: 
};
