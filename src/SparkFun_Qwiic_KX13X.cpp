/******************************************************************************
SparkFun_Qwiic_KX13X.cpp
SparkFun Qwiic KX13X Library Source File
Elias Santistevan @ SparkFun Electronics
Original Creation Date: March, 2021

This file implements the QwiicKX13XCore, QwiicKX132, and QwiicKX134 class

Development environment specifics:
IDE: Arduino 1.8.12

This code is Lemonadeware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#include "SparkFun_Qwiic_KX13X.h"


bool QwDevKX13X::init(void)
{
  if( !_sfebus->ping(_i2cAddress) )
		return false;

	if( getUniqueID() != KX132_WHO_AM_I )
		return false;

	return true; 
}

uint8_t QwDevKX13X::getUniqueID()
{
	uint8_t tempVal;
	int retVal = readRegisterRegion(SFE_KX13X_WHO_AM_I, &tempVal, 1);

	if( retVal != 0 )
		return 0; 

	return tempVal;	

}

////////////////////////////////////////////////////////////////////////////////////
// setCommunicationBus()
//
// Method to set the bus object that is used to communicate with the device
//
//  Parameter    Description
//  ---------    -----------------------------
//  theBus       The communication bus object
//  i2cAddress   I2C address for the 6DoF

void QwDevISM330DHCX::setCommunicationBus(QwIDeviceBus &theBus, uint8_t i2cAddress)
{
    _sfeBus = &theBus;
		_i2cAddress = i2cAddress; 
}

////////////////////////////////////////////////////////////////////////////////////
// setCommunicationBus()
//
// Overloaded option for setting the data bus (theBus) object to a SPI bus object.
//
//  Parameter    Description
//  ---------    -----------------------------
//  theBus       The communication bus object
//  

void QwDevISM330DHCX::setCommunicationBus(QwIDeviceBus &theBus)
{
    _sfeBus = &theBus;
}

// This function sets various register with regards to these pre-determined
// settings. These settings are set according to "AN092 Getting Started" guide and can easily
// have additional presets added.
bool QwDevKX13X::initialize(uint8_t settings)
{

  int retVal; 

  if( !accelControl(false) )
    return false; 
  
  
  if( settings == DEFAULT_SETTINGS )
    retVal = writeRegisterRegion(SFE_KX13X_CNTL1, 0x00, DEFAULT_SETTINGS, 0);

  if( settings == INT_SETTINGS ){
    setInterruptPin(true, 1);
    routeHardwareInterrupt(HI_DATA_READY);
    retVal = writeRegisterRegion(SFE_KX13X_CNTL1, 0x00, INT_SETTINGS, 0);
  }

  if( settings == SOFT_INT_SETTINGS ){
    retVal = writeRegisterRegion(SFE_KX13X_CNTL1, 0x00, INT_SETTINGS, 0);
  }

  if( settings == BUFFER_SETTINGS ){
    setInterruptPin(true, 1);
    routeHardwareInterrupt(HI_BUFFER_FULL);
    enableBuffer(true, true);
    setBufferOperation(BUFFER_MODE_FIFO, BUFFER_16BIT_SAMPLES);
    retVal = writeRegisterRegion(SFE_KX13X_CNTL1, 0x00, INT_SETTINGS, 0);
  }


  if( retVal  != 0 )
    return true;
  else
    return false;
}

bool QwDevKX13X::setOperatingMode(bool standby){

  uint8_t tempVal;
  int retVal;

  retVal = readRegisterRegion(SFE_KX13X_CNTL1, &tempVal, 1);
	
  if( retVal != 0 )
    return false;

	tempVal = (tempVal | standby);

  retVal = writeRegisterByte(SFE_KX13X_CNTL1, tempVal);

  if( retVal != 0 )
    return false;

	return true;
  
}

// Address: 0x1B, bit[7]: default value is: 0x00
// This function reads whether the accelerometer is in stand by or an active
// mode. 
uint8_t QwDevKX13X::getOpertingMode(){

  uint8_t tempVal;
	int retVal;

  retVal = readRegisterRegion(SFE_KX13X_CNTL1, &tempVal, 1);

	if( retVal != 0 )
		return 2; // Not a possible value for the PC1 bit.

  return (tempVal  >> 7);

}
// Address: 0x1B, bit[1:0]: default value is: 0x00 (2g)
// This function sets the acceleration range of the accelerometer outputs.
// Possible KX132 arguments: 0x00 (2g), 0x01 (4g), 0x02 (8g), 0x03 (16g)
// Possible KX134 arguments: 0x00 (8g), 0x01 (16g), 0x02 (32g), 0x03 (64g)
// KX13X needs to be set into standby mode to change this value
bool QwDevKX13X::setRange(uint8_t range){

	int retVal; 

  if( range > 3 )
    return false;

  retVal = writeRegisterByte(SFE_KX13X_CNTL1, range);

  if( retVal != 0 )
    return false;

	return true;
  
}

bool QwDevKX13X::enableDataEngine();
{
	int retVal; 
	uint8_t tempVal; 

	retVal = readRegisterRegion(SFE_KX13X_CNTL1, &tempVal, 1);

  if( retVal != 0 )
    return false;
	
	tempVal = tempVal | 0x20; 

  retVal = writeRegisterByte(SFE_KX13X_CNTL1, tempVal);

  if( retVal != 0 )
    return false;

	return true;
}

bool QwDevKX13X::enableTapEngine();
{
	int retVal; 
	uint8_t tempVal; 

	retVal = readRegisterRegion(SFE_KX13X_CNTL1, &tempVal, 1);

  if( retVal != 0 )
    return false;
	
	tempVal = tempVal | 0x40; 

  retVal = writeRegisterByte(SFE_KX13X_CNTL1, tempVal);

  if( retVal != 0 )
    return false;

	return true;
}

bool QwDevKX13X::enableTiltEngine();
{
	int retVal; 
	uint8_t tempVal; 

	retVal = readRegisterRegion(SFE_KX13X_CNTL1, &tempVal, 1);

  if( retVal != 0 )
    return false;
	
	tempVal = tempVal | 0x01; 

  retVal = writeRegisterByte(SFE_KX13X_CNTL1, tempVal);

  if( retVal != 0 )
    return false;

	return true;
}

//Address: 0x21, bits[3:0] - default value is 0x06 (50Hz)
//Sets the refresh rate of the accelerometer's data. 
// 0.781 * (2 * (n)) derived from pg. 26 of Techincal Reference Manual
bool QwDevKX13X::setOutputDataRate(uint8_t rate)
{

  if( rate > 15 )
    return false;

	uint8_t tempVal;
  int retVal;

	retVal = readRegisterRegion(SFE_KX13X_CNTL1, &tempVal, 1);

  if( retVal != 0 )
    return false;
	
	tempVal = tempVal | rate; 

  retVal = writeRegisterByte(SFE_KX13X_ODCNTL, tempVal);

  if( retVal != 0 )
    return false;

	return true;
}

// Address:0x21 , bit[3:0]: default value is: 0x06 (50Hz)
// Gets the accelerometer's output data rate. 
float QwDevKX13X::getOutputDataRate()
{
	int retVal; 
  uint8_t tempVal;

  retVal = readRegisterRegion(SFE_KX13X_ODCNTL, &tempVal);
	
	if( retVal != 0 )
		return 0.0;

  tempVal = tempVal & 0x0F;

  return (0.78 * ( pow(2,(float)tempVal)));
}


// Address: 0x22, bit[7:4] default value is 0000.
// This register controls the various interrupt settings, all of which can be
// set here. Note: trying to set just one will set the others to their default
// state.
// Thish configures all of the bits related to the interrupt pin.
bool QwDevKX13X::configureInterruptPin(uint8_t pinVal){
  
	int retVal;

  retVal = writeRegisterByte(SFE_KX13X_INC1, pinVal);

  if( retVal != 0 )
    return false;

	return true;
}

bool QwDevKX13X::enablePhysInterrupt(bool enable)
{
	int retVal;
	uint8_t tempVal;

	retVal = readRegisterRegion(SFE_KX13X_INC1, &tempVal, 1);

	if( retVal != 0 )
		return false;

	tempVal = tempVal | (enable << 6);

	retVal = writRegisterByte(SFE_KX13X_INC1, tempVal);

	if( retVal != 0 )
		return false;

	return true; 
}

bool QwDevKX13X::setPinMode(bool activeLow)
{
	int retVal;
	uint8_t tempVal;

	retVal = readRegisterRegion(SFE_KX13X_INC1, &tempVal, 1);

	if( retVal != 0 )
		return false;

	tempVal = tempVal | (activeLow << 5);

	retVal = writRegisterByte(SFE_KX13X_INC1, tempVal);

	if( retVal != 0 )
		return false;

	return true; 
}

bool QwDevKX13X::setLatchControl(bool pulse)
{
	int retVal;
	uint8_t tempVal;

	retVal = readRegisterRegion(SFE_KX13X_INC1, &tempVal, 1);

	if( retVal != 0 )
		return false;

	tempVal = tempVal | (pulse << 4);

	retVal = writRegisterByte(SFE_KX13X_INC1, tempVal);

	if( retVal != 0 )
		return false;

	return true; 
}

bool QwDevKX13X::setPulseWidth(uint8_t width)
{
	int retVal;
	uint8_t tempVal;

	if( width > 4 ) 
		return false; 

	retVal = readRegisterRegion(SFE_KX13X_INC1, &tempVal, 1);

	if( retVal != 0 )
		return false;

	tempVal = tempVal | (width << 6);

	retVal = writRegisterByte(SFE_KX13X_INC1, tempVal);

	if( retVal != 0 )
		return false;

	return true; 
}


// Address: 0x25, bits[7:0]: default value is 0: disabled
// Enables anyone of the various interrupt settings to be routed to hardware
// interrupt pin one or pin two.
bool QwDevKX13X::routeHardwareInterrupt(uint8_t rdr, uint8_t pin)
{

  int retVal;

  if( rdr > 0x80 ) 
    return false;
  

  if( pin == 1 )
	{
    retVal = writeRegisterByte(SFE_KX13X_INC4, rdr);

    if( retVal != 0 )
      return false;
  }

  if( pin == 2 ) 
	{
    retVal = writeRegisterByte(KX13X_INC6, rdr);

    if( retVal != 0 )
      return true;
    
  }

  return false;

}

// Address: 0x1A , bit[7:0]: default value is: 0x00
// This function reads the interrupt latch release register, thus clearing any
// interrupts. 
bool QwDevKX13X::clearInterrupt()
{
  
  int retVal;
  uint8_t tempVal;

  retVal = readRegisterRegion(SFE_KX13X_INT_REL, &tempVal, 1);

  if( retVal != 0 )
		 return false;
	
	return true;
}

// Address: 0x17 , bit[4]: default value is: 0
// This function triggers collection of data by the KX13X.
bool QwDevKX13X::dataTrigger()
{
  
  int retVal;
  uint8_t tempVal;

  retVal = readRegisterRegion(SFE_KX13X_INS2, &tempVal);

  if( retVal  != 0 )
      return false;

	if( tempVal & 0x10 )
      return true;

	return false;
}

// Address: 0x5E , bit[7:0]: default value is: unknown
// This function sets the number of samples (not bytes) that are held in the
// buffer. Each sample is one full word of X,Y,Z data and the minimum that this
// can be set to is two. The maximum is dependent on the resolution: 8 or 16bit,
// set in the BUF_CNTL2 (0x5F) register (see "setBufferOperation" below).  
bool QwDevKX13X::setBufferThreshold(uint8_t threshold){

  if( threshold < 2 || threshold > 171 )
    return false;

  
  uint8_t tempVal;
  uint8_t resolution;
  int retVal;
  retVal = readRegisterRegion(&tempVal, SFE_KX13X_BUF_CNTL2);
  resolution = (tempVal & 0x40) >> 6; 
  if( retVal != SFE_KX13X_SUCCESS )
    return false;

  if( threshold > 86 && resolution == 1 ) // 1 = 16bit resolution, max samples: 86
    threshold = 86; 
  
  retVal = writeRegisterRegion(SFE_KX13X_BUF_CNTL1, 0x00, threshold, 0);
  if( retVal  != 0 )
    return true;
  else 
    return false;

}

// Address: 0x5F, bits[6] and bits[1:0]: default value is: 0x00
// This functions sets the resolution and operation mode of the buffer. This does not include
// the threshold - see "setBufferThreshold". This is a "On-the-fly" register, accel does not need
// to be powered own to adjust settings.
bool QwDevKX13X::setBufferOperation(uint8_t operationMode, uint8_t resolution){

  if( resolution > 1 )
    return false;
  if( operationMode > 2 )
    return false; 


  uint8_t combinedSettings = (resolution << 6) | operationMode;
  int retVal;
  retVal = writeRegisterRegion(SFE_KX13X_BUF_CNTL2, 0xBC, combinedSettings, 0);
  if( retVal  != 0 )
    return true;
  else
    return false;
}

// Address: 0x5F, bit[7] and bit[5]: default values is: 0x00
// This functions enables the buffer and also whether the buffer triggers an
// interrupt when full. This is a "On-the-fly" register, accel does not need
// to be powered down to adjust settings.
bool QwDevKX13X::enableBuffer(bool enable, bool enableInterrupt){

  if( ( enable != true && enable != false)  && (enableInterrupt != true && enableInterrupt != false) )
    return false;

  uint8_t combinedSettings = (enable << 7) | (enableInterrupt << 5); 
  int retVal;
  retVal = writeRegisterRegion(SFE_KX13X_BUF_CNTL2, 0x5F, combinedSettings, 0);
  if( retVal  != 0 )
    return true;
  else
    return false;
}

// Address: 0x1C, bit[6]: default value is: 0x00 
//Tests functionality of the integrated circuit by setting the command test
//control bit, then checks the results in the COTR register (0x12): 0xAA is a
//successful read, 0x55 is the default state. 
bool QwDevKX13X::runCommandTest()
{
  
  uint8_t tempVal;
  int retVal;

  retVal = writeRegisterRegion(SFE_KX13X_CNTL2, 0xBF, 1, 6);
  if( retVal != SFE_KX13X_SUCCESS )
    return false;

  retVal = readRegisterRegion(&tempVal, SFE_KX13X_COTR);
  if( retVal  != 0 && tempVal == COTR_POS_STATE)
    return true;
  else 
    return false;
}

// Address:0x08 - 0x0D or 0x63 , bit[7:0]
// Reads acceleration data from either the buffer or the output registers
// depending on if the user specified buffer usage.
bool QwDevKX13X::getRawAccelData(rawOutputData *rawAccelData){

  
  uint8_t tempVal;
  int retVal;
  uint8_t tempRegData[TOTAL_ACCEL_DATA_16BIT] {}; 

  retVal = readRegisterRegion(&tempVal, SFE_KX13X_INC4);
  if( retVal != SFE_KX13X_SUCCESS )
    return false;

  if( tempVal & 0x40 ){ // If Buffer interrupt is enabled, then we'll read accelerometer data from buffer register.
    retVal = readRegisterRegion(SFE_KX13X_BUF_READ, tempRegData, TOTAL_ACCEL_DATA_16BIT);
  }
  else
    retVal = readRegisterRegion(SFE_KX13X_XOUT_L, tempRegData, TOTAL_ACCEL_DATA_16BIT);



  if( retVal  != 0 ) {
    rawAccelData->xData = tempRegData[XLSB]; 
    rawAccelData->xData |= (static_cast<uint16_t>(tempRegData[XMSB]) << 8); 
    rawAccelData->yData = tempRegData[YLSB]; 
    rawAccelData->yData |= (static_cast<uint16_t>(tempRegData[YMSB]) << 8); 
    rawAccelData->zData = tempRegData[ZLSB]; 
    rawAccelData->zData |= (static_cast<uint16_t>(tempRegData[ZMSB]) << 8); 
    return true;
  }

  return false;
}



//***************************************** KX132 ******************************************
//******************************************************************************************
//******************************************************************************************
//******************************************************************************************

// Constructor
QwiicKX132::QwiicKX132() { }

// Uses the beginCore function to check that the part ID from the "who am I"
// register matches the correct value. Uses I2C for data transfer.
bool QwiicKX132::init(uint8_t kxAddress, TwoWire &i2cPort){

  if( kxAddress != KX13X_DEFAULT_ADDRESS && kxAddress != KX13X_ALT_ADDRESS )
    return false;

  uint8_t partID = getUniqueID(kxAddress, i2cPort); 
  if( partID == KX132_WHO_AM_I ) 
    return true; 
  else 
    return false;
  
}

// Grabs raw accel data and passes it to the following function to be
// converted.
outputData QwiicKX132::getAccelData(){
  
  if( getRawAccelData(&rawAccelData) &&
    convAccelData(&userAccel, &rawAccelData) )
      return userAccel;

  userAccel.xData = 0xFF;
  userAccel.yData = 0xFF;
  userAccel.zData = 0xFF;
  return userAccel;
}

// Converts acceleration data according to the set range value. 
bool QwiicKX132::convAccelData(outputData *userAccel, rawOutputData *rawAccelData){

  uint8_t regVal;
  uint8_t range; 
  int retVal;

  retVal = readRegisterRegion(SFE_KX13X_CNTL1, &regVal, 1);

  if( retVal != 0 )
    return false; 

  range = (regVal & 0x18) >> 3;
  

  switch( range ) {
    case KX132_RANGE2G:
      userAccel->xData = (float)rawAccelData->xData * convRange2G;
      userAccel->yData = (float)rawAccelData->yData * convRange2G;
      userAccel->zData = (float)rawAccelData->zData * convRange2G;
      break;
    case KX132_RANGE4G:
      userAccel->xData = (float)rawAccelData->xData * convRange4G;
      userAccel->yData = (float)rawAccelData->yData * convRange4G;
      userAccel->zData = (float)rawAccelData->zData * convRange4G;
      break;
    case KX132_RANGE8G:
      userAccel->xData = (float)rawAccelData->xData * convRange8G;
      userAccel->yData = (float)rawAccelData->yData * convRange8G;
      userAccel->zData = (float)rawAccelData->zData * convRange8G;
      break;
    case KX132_RANGE16G:
      userAccel->xData = (float)rawAccelData->xData * convRange16G;
      userAccel->yData = (float)rawAccelData->yData * convRange16G;
      userAccel->zData = (float)rawAccelData->zData * convRange16G;
      break;
		default:
			return false;
  }

  return true;
}

//***************************************** KX134 ******************************************
//******************************************************************************************
//******************************************************************************************
//******************************************************************************************

//Constructor
QwiicKX134::QwiicKX134() { }


// Grabs raw accel data and passes it to the following function to be
// converted.
outputData QwiicKX134::getAccelData(){

  if( getRawAccelData(&rawAccelData) &&
    convAccelData(&userAccel, &rawAccelData) )
      return userAccel;

  userAccel.xData = 0xFF;
  userAccel.yData = 0xFF;
  userAccel.zData = 0xFF;
  return userAccel;
}

// Converts acceleration data according to the set range value. 
bool QwiicKX134::convAccelData(outputData *userAccel, rawOutputData *rawAccelData){

  uint8_t regVal;
  uint8_t range; 
  int retVal;

  retVal = readRegisterRegion(SFE_KX13X_CNTL1, &regVal, 1);

  if( retVal != 0 )
    return false; 

  range = (regVal & 0x18) >> 3;

  switch( range ) {
    case KX134_RANGE8G:
      userAccel->xData = (float)rawAccelData->xData * convRange8G;
      userAccel->yData = (float)rawAccelData->yData * convRange8G;
      userAccel->zData = (float)rawAccelData->zData * convRange8G;
      break;                                               
    case KX134_RANGE16G:                                   
      userAccel->xData = (float)rawAccelData->xData * convRange16G;
      userAccel->yData = (float)rawAccelData->yData * convRange16G;
      userAccel->zData = (float)rawAccelData->zData * convRange16G;
      break;                                               
    case KX134_RANGE32G:                                   
      userAccel->xData = (float)rawAccelData->xData * convRange32G;
      userAccel->yData = (float)rawAccelData->yData * convRange32G;
      userAccel->zData = (float)rawAccelData->zData * convRange32G;
      break;                                               
    case KX134_RANGE64G:                                   
      userAccel->xData = (float)rawAccelData->xData * convRange64G;
      userAccel->yData = (float)rawAccelData->yData * convRange64G;
      userAccel->zData = (float)rawAccelData->zData * convRange64G;
      break;

  }

  return true;
}


int QwDevKX13X::readRegisterRegion(uint8_t reg, uint8_t *value, uint8_t len)
{
	_sfeBus->readRegisterRegion(_i2cAddress, reg, *value, len);
}

int QwDevKX13X::writeRegisterRegion(uint8_t reg, uint8_t *value, uint8_t len)
{
	_sfeBus->writeRegisterRegion(_i2cAddress, reg, *value, len);
}

int QwDevKX13X::writeRegisterByte(uint8_t reg, uint8_t value)
{
	_sfeBus->writeRegisterByte(_i2cAddress, reg, value);
}

