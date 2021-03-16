/******************************************************************************
SparkFun_Qwiic_KX13X.cpp
SparkFun Qwiic KX13X Library Source File
Andy England @ SparkFun Electronics
Original Creation Date: January 8, 2021

This file implements the QwiicKX13X class, prototyped in SparkFun_Qwiic_KX13X.h

Development environment specifics:
	IDE: Arduino 1.8.12
	Hardware Platform: Arduino Uno/SparkFun Redboard
	Qwiic Button Version: 1.0.0
    Qwiic Switch Version: 1.0.0

This code is Lemonadeware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#include "SparkFun_Qwiic_KX13X.h"

QwiicKX13xCore::QwiicKX13xCore { }; //Constructor

bool QwiicKX13xCore::beginCore(uint8_t deviceAddress, TwoWire &wirePort)
{
  _deviceAddress = deviceAddress; //If provided, store the I2C address from user
  _i2cPort = &wirePort;
  _i2cPort->beginTransmission(_deviceAddress);
	
  if( _i2cPort->endTransmission() != 0 ) 
    return false; //Error: Sensor did not ack
  else	
    return true;
}

bool QwiicKX13xCore::beginSPICore(uint8_t CSPin, uint32_t spiPortSpeed, SPIClass &spiPort)
{
	_i2cPort = NULL;
	_spiPortSpeed = spiPortSpeed;

	if( _spiPortSpeed > 10000000 )
    _spiPortSpeed = 10000000;

	_cs = CSPin;
	pinMode(_cs, OUTPUT);
	digitalWrite(_cs, HIGH);
	//writeBit(INC1, SPI3E, 1); //Enable SPI
	//_i2cPort->end();
 
// CPOL and CPHA are demonstrated on pg 25 of Specification Data sheet  
// CPOL = 0, CPHA = 0 SPI_MODE0
#ifdef _AVR_
  kxSPISettings = SPISettings(spiPortSpeed, MSBFIRST, SPI_MODE0)
#endif
#ifdef _MK20DX256_ //Teensy
  kxSPISettings = SPISettings(spiPortSpeed, MSBFIRST, SPI_MODE0)
#endif
#ifdef ESP32
  kxSPISettings = SPISettings(spiPortSpeed, SPI_MSBFIRST, SPI_MODE0)
#endif

}

uint8_t QwiicKX13xCore::initialize()
{
	uint8_t initializationValue = 0xC0;
	writeRegister(CNTL1, initializationValue);
	delay(1000);
	return readRegister(CNTL1);
}

//Tests functionality of the integrated circuit
bool QwiicKX13xCore::runCommandTest()
{
	if( readRegister(COTR) == COTR_DEFAULT ) {
		writeBit(CNTL2, COTC, true);// Set the COTC bit to tru to start the test
		if( readRegister(COTR) == COTR_SUCCESS )
			return true;
	}
  else
    return false;
}

//Wait a certain time for incoming I2C bytes before giving up
//Returns false if failed
bool QwiicKX13xCore::waitForI2C()
{
	for(size_t counter = 0; counter < 100; counter++) { //Don't got more than 255 
		if( _i2cPort->available() > 0 )
			return true;
    delay(1);
	}

	return false;
}

//Blocking wait for QwiicKX13X to assert (pull low) the INT pin
//indicating it's ready for comm. Can take more than 104ms
//after a hardware reset
bool QwiicKX13xCore::waitForSPI()
{
	/*for (uint8_t counter = 0; counter < 125; counter++) //Don't got more than 255
	{
		if (digitalRead(_int) == LOW)
			return (true);
		delay(1);
	}*/

	return (false);
}

bool QwiicKX13xCore::getReadings()
{
	return readMultipleRegisters(XOUT_L, _accelData, 6);
}

int16_t QwiicKX13xCore::getAccelX()
{
	return ((_accelData[XMSB] << 8) | _accelData[XLSB]);
}

int16_t QwiicKX13xCore::getAccelY()
{
	return ((_accelData[YMSB] << 8) | _accelData[YLSB]);
}

int16_t QwiicKX13xCore::getAccelZ()
{
	return ((_accelData[ZMSB] << 8) | _accelData[ZLSB]);
}

bool QwiicKX13xCore::readBit(uint8_t regAddr, uint8_t bitAddr)
{
	return ((readRegister(regAddr) & (1 << bitAddr)) >> bitAddr);
}

bool QwiicKX13xCore::writeBit(uint8_t regAddr, uint8_t bitAddr, bool bitToWrite)
{
	uint8_t value = readRegister(regAddr);
	value &= ~(1 << bitAddr);
	value |= bitToWrite << bitAddr;
	return writeRegister(regAddr, value);
}

uint8_t QwiicKX13xCore::readRegister(uint8_t reg)
{

	if( _i2cPort == NULL ) {

    uint8_t regData;

		_spiPort->beginTransaction(kxSPISettings);
		digitalWrite(_cs, LOW);
		reg |= SPI_READ; 
    regData = _spiPort->transfer(reg);
		digitalWrite(_cs, HIGH);
		_spiPort->endTransaction();
		return regData;
	}

	else {
		_i2cPort->beginTransmission(_deviceAddress);
		_i2cPort->write(reg);
		_i2cPort->endTransmission();

    if( _i2cPort->requestFrom(static_cast<uint8_t>(_deviceAddress), static_cast<uint8_t>(1)) != 0 )
      return _i2cPort->read();
    
    return KX13x_I2C_ERROR;
	}
}

//Sends multiple requests to sensor until all data bytes are received from sensor
//The shtpData buffer has max capacity of MAX_PACKET_SIZE. Any bytes over this amount will be lost.
//Arduino I2C read limit is 32 bytes. Header is 4 bytes, so max data we can read per interation is 28 bytes
KX13x_STATUS_t QwiicKX13xCore::readMultipleRegisters(uint8_t reg, uint8_t *dataBuffer, uint8_t numBytes)
{
	
	if( _i2cPort == NULL ) {
		_spiPort->beginTransaction(kxSPISettings);
		digitalWrite(_cs, LOW);
		reg |= SPI_READ;
		databuffer[0] = _spiPort->transfer(reg); //first byte on transfer of address and read bit
		for(size_t i = 1; i < numBytes; i++) {
			dataBuffer[i] = _spiPort->transfer(0); //Assuming this will initiate auto-increment behavior
		}
		digitalWrite(_cs, HIGH);
		_spiPort->endTransaction();
		return KX13x_SUCCESS;
	}
	else {
		_i2cPort->beginTransmission(_deviceAddress);
		_i2cPort->write(reg);

    if( _i2cPort->endTransmission() != 0 )
      return KX13x_I2C_ERROR; //Error: Sensor did not ack

		_i2cPort->requestFrom(static_cast<uint8_t>(_deviceAddress), numBytes);
		for(size_t i = 0; i < numBytes; i++) {
			dataBuffer[i] = _i2cPort->read();
		}

    return KX13x_SUCCESS;
	}
}

KX13x_STATUS_t QwiicKX13xCore::writeRegister(uint8_t reg, uint8_t data)
{

	if( _i2cPort == NULL ) {

		_spiPort->beginTransaction(kxSPISettings);
		digitalWrite(_cs, LOW);
		_spiPort->transfer(reg |= SPI_WRITE);
		_spiPort->transfer(data); 
		digitalWrite(_cs, HIGH);
		_spiPort->endTransaction();
    return KX13x_SUCCESS;
	}

	else { 
		_i2cPort->beginTransmission(_deviceAddress);
		_i2cPort->write(reg); // Move to register
		_i2cPort->write(data); 
		uint8_t i2cResult = _i2cPort->endTransmission();
		if( i2cResult != 0 )
			return KX13x_SUCCESS;
	}

}


//*********************************
//*********************************
//*********************************
//*********************************

QwiicKX132::QwiicKX132 { }


//*********************************
//*********************************
//*********************************
//*********************************
QwiicKX134::QwiicKX134 { }




