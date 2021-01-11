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

bool QwiicKX13X::begin(uint8_t deviceAddress, TwoWire &wirePort)
{
    _deviceAddress = deviceAddress; //If provided, store the I2C address from user
	_i2cPort = &wirePort;
    _i2cPort->beginTransmission(_deviceAddress);
	
    if (_i2cPort->endTransmission() != 0)
	{
      return (false); //Error: Sensor did not ack
	}
	return(true);
}

bool QwiicKX13X::beginSPI()
{

}

bool QwiicKX13X::initialize()
{
	bool returnVal = writeRegister(CNTL1, 0xC0);
	delay(1000);
	return returnVal;
}

//Tests functionality of the integrated circuit
bool QwiicKX13X::runCommandTest()
{
	if(readRegister(COTR) == COTR_DEFAULT)
	{
		writeBit(CNTL2, COTC, true);// Set the COTC bit to tru to start the test
		if(readRegister(COTR) == COTR_SUCCESS)
		{
			return true;
		}
	}
	return false;
}

//Wait a certain time for incoming I2C bytes before giving up
//Returns false if failed
bool QwiicKX13X::waitForI2C()
{
	for (uint8_t counter = 0; counter < 100; counter++) //Don't got more than 255
	{
		if (_i2cPort->available() > 0)
			return (true);
		delay(1);
	}

	return (false);
}

//Blocking wait for QwiicKX13X to assert (pull low) the INT pin
//indicating it's ready for comm. Can take more than 104ms
//after a hardware reset
bool QwiicKX13X::waitForSPI()
{
	/*for (uint8_t counter = 0; counter < 125; counter++) //Don't got more than 255
	{
		if (digitalRead(_int) == LOW)
			return (true);
		delay(1);
	}*/

	return (false);
}

bool QwiicKX13X::getReadings()
{
	return getData(XOUT_L, _accelData, 6);
}

int16_t QwiicKX13X::getAccelX()
{
	return ((_accelData[XMSB] << 8) | _accelData[XLSB]);
}

int16_t QwiicKX13X::getAccelY()
{
	return ((_accelData[YMSB] << 8) | _accelData[YLSB]);
}

int16_t QwiicKX13X::getAccelZ()
{
	return ((_accelData[ZMSB] << 8) | _accelData[ZLSB]);
}

//Sends multiple requests to sensor until all data bytes are received from sensor
//The shtpData buffer has max capacity of MAX_PACKET_SIZE. Any bytes over this amount will be lost.
//Arduino I2C read limit is 32 bytes. Header is 4 bytes, so max data we can read per interation is 28 bytes
bool QwiicKX13X::getData(uint8_t startingRegister, uint8_t * dataBuffer, uint8_t bytesToGet)
{
    _i2cPort->beginTransmission(_deviceAddress);
	_i2cPort->write(startingRegister);
    if (_i2cPort->endTransmission() != 0)
      return (false); //Error: Sensor did not ack

	_i2cPort->requestFrom(static_cast<uint8_t>(_deviceAddress), bytesToGet);
	for (uint8_t i = 0; i < bytesToGet; i++)
	{
		dataBuffer[i] = _i2cPort->read();
	}
	return true;
}

bool QwiicKX13X::readBit(uint8_t regAddr, uint8_t bitAddr)
{
	return ((readRegister(regAddr) & (1 << bitAddr)) >> bitAddr);
}

bool QwiicKX13X::writeBit(uint8_t regAddr, uint8_t bitAddr, bool bitToWrite)
{
	uint8_t value = readRegister(regAddr);
	value &= ~(1 << bitAddr);
	value |= bitToWrite << bitAddr;
	return writeRegister(regAddr, value);
}

uint8_t QwiicKX13X::readRegister(uint8_t addr)
{
	_i2cPort->beginTransmission(_deviceAddress);
	_i2cPort->write(addr);
	_i2cPort->endTransmission();

    //typecasting the 1 parameter in requestFrom so that the compiler
    //doesn't give us a warning about multiple candidates
    if (_i2cPort->requestFrom(static_cast<uint8_t>(_deviceAddress), static_cast<uint8_t>(1)) != 0)
    {
        return _i2cPort->read();
    }
	return false;
}
//Given the data packet, send the header then the data
//Returns false if sensor does not ACK
//TODO - Arduino has a max 32 byte send. Break sending into multi packets if needed.
bool QwiicKX13X::writeRegister(uint8_t startingRegister, uint8_t data)
{

	if (_i2cPort == NULL) //Do SPI
	{
		//Wait for QwiicKX13X to indicate it is available for communication
		/*if (waitForSPI() == false)
			return (false); //Something went wrong

		//QwiicKX13X has max CLK of 3MHz, MSB first,
		//The QwiicKX13X uses CPOL = 1 and CPHA = 1. This is mode3
		_spiPort->beginTransaction(SPISettings(_spiPortSpeed, MSBFIRST, SPI_MODE3));
		digitalWrite(_cs, LOW);

		//Send the 4 byte packet header
		_spiPort->transfer(packetLength & 0xFF);			 //Packet length LSB
		_spiPort->transfer(packetLength >> 8);				 //Packet length MSB
		_spiPort->transfer(channelNumber);					 //Channel number
		_spiPort->transfer(sequenceNumber[channelNumber]++); //Send the sequence number, increments with each packet sent, different counter for each channel

		//Send the user's data packet
		for (uint8_t i = 0; i < dataLength; i++)
		{
			_spiPort->transfer(shtpData[i]);
		}

		digitalWrite(_cs, HIGH);
		_spiPort->endTransaction();*/
	}
	else //Do I2C
	{
		//if(packetLength > I2C_BUFFER_LENGTH) return(false); //You are trying to send too much. Break into smaller packets.

		_i2cPort->beginTransmission(_deviceAddress);

		//Send the 4 byte packet header
		_i2cPort->write(startingRegister);					  //Channel number
		_i2cPort->write(data); 

		uint8_t i2cResult = _i2cPort->endTransmission();

		if (i2cResult != 0)
		{

			return (false);
		}
	}

	return (true);
}
