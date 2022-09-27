/******************************************************************************
SparkFun_Qwiic_KX13X.h
Elias Santistevan @ SparkFun Electronics
Original Creation Date: March 2021

This code is Lemonadeware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#pragma once

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <SPI.h>
#include <Wire.h>

#define KX13X_ADDRESS_HIGH 0x1F
#define KKX13X_ADDRESS_LOW 0x1E

#define KX132_WHO_AM_I  0x3D
#define KX134_WHO_AM_I  0x46

#define TOTAL_ACCEL_DATA_16BIT 6 
#define TOTAL_ACCEL_DATA_8BIT 3 
#define MAX_BUFFER_LENGTH 32 

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

#define COTR_DEF_STATE 0x55
#define COTR_POS_STATE 0xAA

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


class QwiicKX13xCore
{
	public:

    QwiicKX13xCore();

		uint8_t beginCore(uint8_t, TwoWire &wirePort);
		uint8_t beginSPICore(uint8_t, uint32_t, SPIClass &spiPort);
		bool initialize(uint8_t settings = DEFAULT_SETTINGS);
		bool runCommandTest();
    bool accelControl(bool);
    uint8_t readAccelState();
    bool setRange(uint8_t);
    bool setOutputDataRate(uint8_t);
    float readOutputDataRate();
    bool setInterruptPin(bool enable, uint8_t polarity = 0, uint8_t pulseWidth = 0, bool latchControl = false);
    bool routeHardwareInterrupt(uint8_t, uint8_t pin = 1);
    bool clearInterrupt();
    bool dataTrigger();
    bool setBufferThreshold(uint8_t);
    bool setBufferOperation(uint8_t, uint8_t);
    bool enableBuffer(bool, bool);

    bool getRawAccelData(rawOutputData*);

		KX13X_STATUS_t readRegister(uint8_t*, uint8_t);
		KX13X_STATUS_t writeRegister(uint8_t, uint8_t, uint8_t, uint8_t);
		KX13X_STATUS_t readMultipleRegisters(uint8_t, uint8_t dataBuffer[] , uint16_t);
    KX13X_STATUS_t overBufLenI2CRead(uint8_t, uint8_t dataBuffer[] , int16_t);

    // CPOL and CPHA are demonstrated on pg 25 of Specification Data sheet  
    // CPOL = 0, CPHA = 0 SPI_MODE0
    SPISettings kxSPISettings;
    rawOutputData rawAccelData;
    outputData userAccel;
    

	private:

		TwoWire *_i2cPort;		//The generic connection to user's chosen I2C hardware
		SPIClass *_spiPort;			 //The generic connection to user's chosen SPI hardware

		uint8_t _deviceAddress; 
		uint32_t _spiPortSpeed; // max port speed is 10MHz 
		uint8_t _cs;

    
};

class QwiicKX132 : public QwiicKX13xCore
{
  public:

    QwiicKX132();
    bool begin(uint8_t kxAddress = KX13X_ADDRESS_HIGH, TwoWire &i2cPort = Wire);
    bool beginSPI(uint8_t, uint32_t spiPortSpeed = 2000000, SPIClass &spiPort = SPI);
    outputData getAccelData();
    bool convAccelData(outputData*, rawOutputData*); 

  private: 

    const double convRange2G =  .00006103518784142582;
    const double convRange4G =  .0001220703756828516;
    const double convRange8G =  .0002441407513657033;
    const double convRange16G = .0004882811975463118;

#define KX132_RANGE2G  0x00
#define KX132_RANGE4G  0x01
#define KX132_RANGE8G  0x02
#define KX132_RANGE16G 0x03

};

class QwiicKX134 : public QwiicKX13xCore
{
  public: 

    QwiicKX134();
    bool begin(uint8_t kxAddress = KX13X_ADDRESS_HIGH, TwoWire &i2cPort = Wire);
    bool beginSPI(uint8_t, uint32_t spiPortSpeed = 10000000, SPIClass &spiPort = SPI);
    outputData getAccelData();
    bool convAccelData(outputData*, rawOutputData*); 

  private: 

    const double convRange8G =  .000244140751365703299;
    const double convRange16G = .000488281197546311838;
    const double convRange32G = .000976523950926236762;
    const double convRange64G = .001953125095370342112;

#define KX134_RANGE8G  0x00
#define KX134_RANGE16G 0x01
#define KX134_RANGE32G 0x02
#define KX134_RANGE64G 0x03
};
    
