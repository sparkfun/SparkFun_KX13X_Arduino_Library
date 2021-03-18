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

QwiicKX13xCore::QwiicKX13xCore() { } //Constructor

uint8_t QwiicKX13xCore::beginCore(uint8_t deviceAddress, TwoWire &i2cPort)
{
  _deviceAddress = deviceAddress; //If provided, store the I2C address from user
  _i2cPort = &i2cPort;
  uint8_t partID;
  KX13X_STATUS_t status = readRegister(&partID, KX13X_WHO_AM_I);
  if( status != KX13X_SUCCESS ) 
    return status;
  else    
    return partID;
}

uint8_t QwiicKX13xCore::beginSPICore(uint8_t CSPin, uint32_t spiPortSpeed, SPIClass &spiPort)
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
#ifdef ESP32
  kxSPISettings = SPISettings(spiPortSpeed, SPI_MSBFIRST, SPI_MODE0);
#else 
  kxSPISettings = SPISettings(spiPortSpeed, MSBFIRST, SPI_MODE0);
#endif
//#ifdef _MK20DX256_ //Teensy
 // kxSPISettings = SPISettings(spiPortSpeed, MSBFIRST, SPI_MODE0)
//#endif

  uint8_t partID;
  KX13X_STATUS_t status = readRegister(&partID, KX13X_WHO_AM_I);
  if( status != KX13X_SUCCESS ) 
    return status;
  else    
    return partID;
}

uint8_t QwiicKX13xCore::initialize()
{
  return true;
}

//Tests functionality of the integrated circuit
bool QwiicKX13xCore::runCommandTest()
{
  return true;
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

bool QwiicKX13xCore::readBit(uint8_t regAddr, uint8_t bitAddr)
{
	return true;
    //((readRegister(regAddr) & (1 << bitAddr)) >> bitAddr);
}

bool QwiicKX13xCore::writeBit(uint8_t regAddr, uint8_t bitAddr, bool bitToWrite)
{
	uint8_t value;
  readRegister(&value, regAddr);
	value &= ~(1 << bitAddr);
	value |= bitToWrite << bitAddr;
  writeRegister(regAddr, value);
	return true;
}

KX13X_STATUS_t QwiicKX13xCore::getRawAccelData(outputData userAccel){

  uint8_t tempRegData[TOTAL_ACCEL_DATA] {}; 
  KX13X_STATUS_t returnError;

  returnError = readMultipleRegisters(KX13X_XADP_L, tempRegData, TOTAL_ACCEL_DATA);
  if( returnError == KX13X_SUCCESS ) {
      userAccel.xData = tempRegData[XLSB]; 
      userAccel.xData |= (static_cast<uint16_t>(tempRegData[XMSB]) << 8); 
      userAccel.yData = tempRegData[YLSB]; 
      userAccel.yData |= (static_cast<uint16_t>(tempRegData[YMSB]) << 8); 
      userAccel.zData = tempRegData[ZLSB]; 
      userAccel.zData |= (static_cast<uint16_t>(tempRegData[ZMSB]) << 8); 
      return returnError;
  }
  else
    return returnError;
     
    
}

KX13X_STATUS_t QwiicKX13xCore::readRegister(uint8_t *dataPointer, uint8_t reg)
{

	if( _i2cPort == NULL ) {
		_spiPort->beginTransaction(kxSPISettings);
		digitalWrite(_cs, LOW);
		reg |= SPI_READ; 
    *dataPointer = _spiPort->transfer(reg);
		digitalWrite(_cs, HIGH);
		_spiPort->endTransaction();
    return KX13X_SUCCESS;
	}

	else {
		_i2cPort->beginTransmission(_deviceAddress);
		_i2cPort->write(reg);

    _i2cPort->requestFrom(static_cast<uint8_t>(_deviceAddress), static_cast<uint8_t>(1));
    *dataPointer = _i2cPort->read();
    uint8_t i2cResult = _i2cPort->endTransmission(); 
    if( i2cResult != 0 )
      return KX13X_I2C_ERROR; //Error: Sensor did not ack
    return KX13X_SUCCESS;
	}
}

//Sends multiple requests to sensor until all data bytes are received from sensor
//The shtpData buffer has max capacity of MAX_PACKET_SIZE. Any bytes over this amount will be lost.
//Arduino I2C read limit is 32 bytes. Header is 4 bytes, so max data we can read per interation is 28 bytes
KX13X_STATUS_t QwiicKX13xCore::readMultipleRegisters(uint8_t reg, uint8_t dataBuffer[], uint8_t numBytes)
{
	
	if( _i2cPort == NULL ) {
		_spiPort->beginTransaction(kxSPISettings);
		digitalWrite(_cs, LOW);
		reg |= SPI_READ;
		dataBuffer[0] = _spiPort->transfer(reg); //first byte on transfer of address and read bit
		for(size_t i = 1; i < numBytes; i++) {
			dataBuffer[i] = _spiPort->transfer(0); //Assuming this will initiate auto-increment behavior
		}
		digitalWrite(_cs, HIGH);
		_spiPort->endTransaction();
		return KX13X_SUCCESS;
	}
	else {

    if( numBytes > MAX_BUFFER_LENGTH){
      KX13X_STATUS_t returnError;
      returnError = overBufLenI2CRead(reg, dataBuffer, numBytes);
      return returnError;
    }

		_i2cPort->beginTransmission(_deviceAddress);
		_i2cPort->write(reg);
		_i2cPort->requestFrom(static_cast<uint8_t>(_deviceAddress), numBytes);
		for(size_t i = 0; i < numBytes; i++) {
			dataBuffer[i] = _i2cPort->read();
		}

    uint8_t i2cResult = _i2cPort->endTransmission();
    if( i2cResult != 0 )
      return KX13X_I2C_ERROR; //Error: Sensor did not ack
    return KX13X_SUCCESS;
	}
}

KX13X_STATUS_t QwiicKX13xCore::overBufLenI2CRead(uint8_t reg, uint8_t *dataBuffer, uint8_t numBytes)
{
  uint8_t resizedRead; 
  uint8_t i2cResult; 
  size_t arrayPlaceHolder = 0;

  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(reg); 

  while(numBytes > 0) 
  {
    if( numBytes > MAX_BUFFER_LENGTH )
      resizedRead = MAX_BUFFER_LENGTH; 
    else
      resizedRead = numBytes; 

		_i2cPort->requestFrom(static_cast<uint8_t>(_deviceAddress), resizedRead, false); //false = repeated start

		for(size_t i = arrayPlaceHolder; i < resizedRead; i++) {
      arrayPlaceHolder++;
			dataBuffer[i] = _i2cPort->read();
    }	
    numBytes = numBytes - MAX_BUFFER_LENGTH; // end condition
  }

  i2cResult = _i2cPort->endTransmission();
  if( i2cResult != 0 )
    return KX13X_I2C_ERROR; //Error: Sensor did not ack
  else
    return KX13X_SUCCESS;
}

KX13X_STATUS_t QwiicKX13xCore::writeRegister(uint8_t reg, uint8_t data)
{

	if( _i2cPort == NULL ) {

		_spiPort->beginTransaction(kxSPISettings);
		digitalWrite(_cs, LOW);
		_spiPort->transfer(reg |= SPI_WRITE);
		_spiPort->transfer(data); 
		digitalWrite(_cs, HIGH);
		_spiPort->endTransaction();
    return KX13X_SUCCESS;
	}

	else { 
		_i2cPort->beginTransmission(_deviceAddress);
		_i2cPort->write(reg); // Move to register
		_i2cPort->write(data); 

		uint8_t i2cResult = _i2cPort->endTransmission();
		if( i2cResult != 0 )
			return KX13X_I2C_ERROR;
    else
      return KX13X_SUCCESS;
	}

}



//*************** KX132 ******************
//****************************************
//****************************************
//****************************************

QwiicKX132::QwiicKX132() { }

bool QwiicKX132::begin(uint8_t kxAddress, TwoWire &i2cPort){

  if( kxAddress != KX13X_DEFAULT_ADDRESS && kxAddress != KX13X_ALT_ADDRESS )
    return false;

  uint8_t partID = beginCore(kxAddress, i2cPort); 
  if( partID == KX132_WHO_AM_I ) 
    return true; 
  else 
    return false;
}

bool QwiicKX132::beginSPI(uint8_t csPin, uint32_t spiPortSpeed, SPIClass &spiPort){

  uint8_t partID = beginSPICore(csPin, spiPortSpeed, spiPort); 
  if( partID == KX132_WHO_AM_I ) 
    return true; 
  else 
    return false;
}


//*************** KX134 ******************
//****************************************
//****************************************
//****************************************
QwiicKX134::QwiicKX134() { }

bool QwiicKX134::begin(uint8_t kxAddress, TwoWire &i2cPort){

  if( kxAddress != KX13X_DEFAULT_ADDRESS && kxAddress != KX13X_ALT_ADDRESS )
    return false;

  uint8_t partID = beginCore(kxAddress, i2cPort); 
  if( partID == KX134_WHO_AM_I ) 
    return true; 
  else 
    return false;
}

//
bool QwiicKX134::beginSPI(uint8_t csPin, uint32_t spiPortSpeed, SPIClass &spiPort){

  uint8_t partID = beginSPICore(csPin, spiPortSpeed, spiPort); 
  if( partID == KX134_WHO_AM_I ) 
    return true; 
  else 
    return false;
}



