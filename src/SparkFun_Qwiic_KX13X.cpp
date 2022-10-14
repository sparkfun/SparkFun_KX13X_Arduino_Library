#include "SparkFun_Qwiic_KX13X.h"


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
//  Parameter:    
//  theBus-The communication bus object
//  i2cAddress-I2C address for the 6DoF
void QwDevKX13X::setCommunicationBus(QwIDeviceBus &theBus, uint8_t i2cAddress)
{
    _sfeBus = &theBus;
		_i2cAddress = i2cAddress; 
}

////////////////////////////////////////////////////////////////////////////////////
// setCommunicationBus()
//
// Overloaded option for setting the data bus (theBus) object to a SPI bus object.
//
//  Parameter:
//  theBus-The communication bus object
//  
void QwDevKX13X::setCommunicationBus(QwIDeviceBus &theBus)
{
    _sfeBus = &theBus;
}

// This function sets various register with regards to these pre-determined
// settings. These settings are set according to "AN092 Getting Started" guide and can easily
// have additional presets added.
bool QwDevKX13X::initialize(uint8_t settings)
{

  int retVal; 

  if( !enableAccel(true) )
    return false; 
  
  
  if( settings == DEFAULT_SETTINGS )
    retVal = writeRegisterByte(SFE_KX13X_CNTL1, DEFAULT_SETTINGS);

  if( settings == INT_SETTINGS )
	{
    enablePhysInterrupt();
    routeHardwareInterrupt(0x10);
    retVal = writeRegisterByte(SFE_KX13X_CNTL1, INT_SETTINGS);
  }

  if( settings == BUFFER_SETTINGS )
	{
    enablePhysInterrupt();
    routeHardwareInterrupt(0x40);//Buffer full interrupt
    enableSampleBuffer(); //Enable buffer
    setBufferOperationMode(0x00); //FIFO
    retVal = writeRegisterByte(SFE_KX13X_CNTL1, INT_SETTINGS);
  }


  if( retVal  != 0 )
    return false;

	return true;
}

//////////////////////////////////////////////////
// softwareReset()
//
// Resets the accelerometer
//
bool QwDevKX13X::softwareReset()
{

  uint8_t reset = 0x80;
  int retVal;

  retVal = writeRegisterByte(SFE_KX13X_CNTL2, reset);
	
	// Logic is inverted here - if we reset using I2C the 
	// accelerometer immediately shuts off which results
	// in a NACK.
  if( retVal != 0 )
    return true;

	return false;

}

//////////////////////////////////////////////////
// enableAccel()
//
// Enables acceleromter data. In addition
// some settings can only be set when the accelerometer is 
// powered down
//
// Parameter: 
// enable - enables or disables the accelerometer
//
//

bool QwDevKX13X::enableAccel(bool enable)
{

  uint8_t tempVal;
  int retVal;

  retVal = readRegisterRegion(SFE_KX13X_CNTL1, &tempVal, 1);
	
  if( retVal != 0 )
    return false;

	tempVal = (tempVal | (enable << 7));

  retVal = writeRegisterByte(SFE_KX13X_CNTL1, tempVal);

  if( retVal != 0 )
    return false;

	return true;
  
}

//////////////////////////////////////////////////
// getOperatingMode()
//
// Retrieves the current operating mode - low/high power mode
//

int8_t QwDevKX13X::getOperatingMode(){

  uint8_t tempVal;
	int retVal;

  retVal = readRegisterRegion(SFE_KX13X_CNTL1, &tempVal, 1);

	if( retVal != 0 )
		return retVal; 

  return (tempVal  >> 7);

}

//////////////////////////////////////////////////
// setRange()
//
// Sets the operational g-range of the accelerometer.
//
// Parameter:
// range - sets the range of the accelerometer 2g - 32g depending
// on the version. 
//
bool QwDevKX13X::setRange(uint8_t range)
{

	int retVal; 

  if( range > 3 )
    return false;

  retVal = writeRegisterByte(SFE_KX13X_CNTL1, range);

  if( retVal != 0 )
    return false;

	return true;
  
}

//////////////////////////////////////////////////
// enableDataEngine()
//
// Enables the data ready bit. 
//
// Parameter:
// enable - enable/disables the data ready bit. 
//
bool QwDevKX13X::enableDataEngine(bool enable)
{
	int retVal; 
	uint8_t tempVal; 

	retVal = readRegisterRegion(SFE_KX13X_CNTL1, &tempVal, 1);

  if( retVal != 0 )
    return false;
	
	tempVal = tempVal | (enable << 5); 

  retVal = writeRegisterByte(SFE_KX13X_CNTL1, tempVal);

  if( retVal != 0 )
    return false;

	return true;
}

//////////////////////////////////////////////////
// enableTapEngine()
//
// Enables the tap and double tap features of the accelerometers
//
// Parameter:
// enable - enables the tap/double tap feature
//
bool QwDevKX13X::enableTapEngine(bool enable)
{
	int retVal; 
	uint8_t tempVal; 

	retVal = readRegisterRegion(SFE_KX13X_CNTL1, &tempVal, 1);

  if( retVal != 0 )
    return false;
	
	tempVal = tempVal | (enable << 2); 

  retVal = writeRegisterByte(SFE_KX13X_CNTL1, tempVal);

  if( retVal != 0 )
    return false;

	return true;
}


//////////////////////////////////////////////////
// enableTiltEngine()
//
// Enables the tilt detection feature. 
//
// Parameter:
// enable - enables the tilt feature
//
bool QwDevKX13X::enableTiltEngine(bool enable)
{
	int retVal; 
	uint8_t tempVal; 

	retVal = readRegisterRegion(SFE_KX13X_CNTL1, &tempVal, 1);

  if( retVal != 0 )
    return false;
	
	tempVal = tempVal | enable; 

  retVal = writeRegisterByte(SFE_KX13X_CNTL1, tempVal);

  if( retVal != 0 )
    return false;

	return true;
}


//////////////////////////////////////////////////
// enableWakeEngine()
//
// Enables the wake detection feature. 
//
// Parameter:
// enable - enables/disables the wake detection feature
//
bool QwDevKX13X::enableWakeEngine(bool enable)
{
	int retVal; 
	uint8_t tempVal; 

	retVal = readRegisterRegion(SFE_KX13X_CNTL4, &tempVal, 1);

  if( retVal != 0 )
    return false;
	
	tempVal = tempVal | (enable << 5); 

  retVal = writeRegisterByte(SFE_KX13X_CNTL4, tempVal);

  if( retVal != 0 )
    return false;

	return true;
}

//////////////////////////////////////////////////
// enableSleepEngine()
//
// Enables the sleep feature. 
//
// Parameter:
// enable - enables/disables the sleep feature
//
bool QwDevKX13X::enableSleepEngine(bool enable)
{
	int retVal; 
	uint8_t tempVal; 

	retVal = readRegisterRegion(SFE_KX13X_CNTL4, &tempVal, 1);

  if( retVal != 0 )
    return false;
	
	tempVal = tempVal | (enable << 4); 

  retVal = writeRegisterByte(SFE_KX13X_CNTL4, tempVal);

  if( retVal != 0 )
    return false;

	return true;
}


//////////////////////////////////////////////////
// setOutputDataRate()
//
// Changes the rate at which accelerometer data is generated.
//
// Parameter:
// rate - determines the rate to be applied.
//
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

//////////////////////////////////////////////////
// setTapDataRate()
//
// Changes the rate at which tap data is generated.
//
// Parameter:
// rate - determines the rate to be applied.
//
bool QwDevKX13X::setTapDataRate(uint8_t rate)
{

  if( rate > 7 )
    return false;

	uint8_t tempVal;
  int retVal;

	retVal = readRegisterRegion(SFE_KX13X_CNTL3, &tempVal, 1);

  if( retVal != 0 )
    return false;
	
	tempVal = tempVal | (rate << 3); 

  retVal = writeRegisterByte(SFE_KX13X_CNTL3, tempVal);

  if( retVal != 0 )
    return false;

	return true;
}


//////////////////////////////////////////////////
// setTiltDataRate()
//
// Changes the rate at which the tilt position is polled.
//
// Parameter:
// rate - determines the rate to be applied.
//
bool QwDevKX13X::setTiltDataRate(uint8_t rate)
{

  if( rate > 3 )
    return false;

	uint8_t tempVal;
  int retVal;

	retVal = readRegisterRegion(SFE_KX13X_CNTL3, &tempVal, 1);

  if( retVal != 0 )
    return false;
	
	tempVal = tempVal | (rate << 6); 

  retVal = writeRegisterByte(SFE_KX13X_CNTL3, tempVal);

  if( retVal != 0 )
    return false;

	return true;
}


//////////////////////////////////////////////////
// setWakeDataRate()
//
// Changes the rate at which the wake function is performed.
//
// Parameter:
// rate - determines the rate to be applied.
//
bool QwDevKX13X::setWakeDataRate(uint8_t rate)
{

  if( rate > 7 )
    return false;

	uint8_t tempVal;
  int retVal;

	retVal = readRegisterRegion(SFE_KX13X_CNTL3, &tempVal, 1);

  if( retVal != 0 )
    return false;
	
	tempVal = tempVal | rate; 

  retVal = writeRegisterByte(SFE_KX13X_CNTL3, tempVal);

  if( retVal != 0 )
    return false;

	return true;
}
//////////////////////////////////////////////////
// getOutputDataRate()
//
// Retrieves the output data rate of the acceleromter.
//
float QwDevKX13X::getOutputDataRate()
{
	int retVal; 
  uint8_t tempVal;

  retVal = readRegisterRegion(SFE_KX13X_ODCNTL, &tempVal, 1);
	
	if( retVal != 0 )
		return 0.0;

  tempVal = tempVal & 0x0F;

  return (0.78 * ( pow(2,(float)tempVal)));
}


//////////////////////////////////////////////////
// configureInterruptPin()
//
// This allows you to configure the entire interrupt register
//
// Parameter:
// pinVal - register value to set, note that this overwrites
// everything in the register.
//
bool QwDevKX13X::configureInterruptPin(uint8_t pinVal){
  
	int retVal;

  retVal = writeRegisterByte(SFE_KX13X_INC1, pinVal);

  if( retVal != 0 )
    return false;

	return true;
}


//////////////////////////////////////////////////
// enablePhysInterrupt()
//
// Enables interrupts to be routed to the interrupt pins. 
//
// Parameters:
// enable - Enables interrupts to report to the physical interrupt pins
// pin - This determines which pin to route the interrupts.
//
bool QwDevKX13X::enablePhysInterrupt(bool enable, uint8_t pin)
{
	int retVal;
	uint8_t tempVal;

	if( pin == 1 )
	{
		retVal = readRegisterRegion(SFE_KX13X_INC1, &tempVal, 1);

		if( retVal != 0 )
			return false;

		tempVal = tempVal | (enable << 5);

		writeRegisterByte(SFE_KX13X_INC1, tempVal);

	}

	if( pin == 2 )
	{
		retVal = readRegisterRegion(SFE_KX13X_INC5, &tempVal, 1);

		if( retVal != 0 )
			return false;

		tempVal = tempVal | (enable << 5);

		writeRegisterByte(SFE_KX13X_INC5, tempVal);

	}

	return true; 
}

//////////////////////////////////////////////////
// setPinMode()
//
// Sets the active state of the physical interupt pins
//
// Parameters:
// enable - Enables interrupts to report to the physical interrupt pins
// pin - This determines which pin to route the interrupts.
//
bool QwDevKX13X::setPinMode(bool activeLow, uint8_t pin)
{
	int retVal;
	uint8_t tempVal;

	if( pin > 2 ) 
		return false;

	if( pin == 1 )
	{
		retVal = readRegisterRegion(SFE_KX13X_INC1, &tempVal, 1);

		if( retVal != 0 )
			return false;

		tempVal = tempVal | (activeLow << 5);

		writeRegisterByte(SFE_KX13X_INC1, tempVal);
	}

	if( pin == 2 )
	{
		retVal = readRegisterRegion(SFE_KX13X_INC5, &tempVal, 1);

		if( retVal != 0 )
			return false;

		tempVal = tempVal | (activeLow << 5);

		writeRegisterByte(SFE_KX13X_INC5, tempVal);
	}

	return true; 
}

//////////////////////////////////////////////////
// setLatchControl()
//
// Determines whether interrupts are pulsed (default) or latched. 
// If they are latched then the interrupt must be released by reading
// the INT_REL register - clearInterrupt();
//
// Parameters:
// latch - True enables latch behavior, false enables pulse behavior (default)
//
bool QwDevKX13X::setLatchControl(bool latch, uint8_t pin)
{
	int retVal;
	uint8_t tempVal;
	
	if( pin > 2 )
		return false;

	if( pin == 1 )
	{
		retVal = readRegisterRegion(SFE_KX13X_INC1, &tempVal, 1);

		if( retVal != 0 )
			return false;

		tempVal = tempVal | (latch << 3);

		writeRegisterByte(SFE_KX13X_INC1, tempVal);
	}


	if( pin == 2 )
	{
		retVal = readRegisterRegion(SFE_KX13X_INC5, &tempVal, 1);

		if( retVal != 0 )
			return false;

		tempVal = tempVal | (latch << 3);

		writeRegisterByte(SFE_KX13X_INC5, tempVal);
	}

	return true; 
}

//////////////////////////////////////////////////
// setPulseWidth()
//
// Determines the width of the interrupt pulse 
//
// Parameters:
// width - The width setting to be applied. 
// pin - the pin to be configured. 
//
bool QwDevKX13X::setPulseWidth(uint8_t width, uint8_t pin)
{
	int retVal;
	uint8_t tempVal;

	if( (width > 4) | (pin > 2) ) 
		return false; 

	if( pin == 1 )
	{
		retVal = readRegisterRegion(SFE_KX13X_INC1, &tempVal, 1);

		if( retVal != 0 )
			return false;

		tempVal = tempVal | (width << 6);

		writeRegisterByte(SFE_KX13X_INC1, tempVal);
	}

	if( pin == 2 )
	{
		retVal = readRegisterRegion(SFE_KX13X_INC5, &tempVal, 1);

		if( retVal != 0 )
			return false;

		tempVal = tempVal | (width << 6);

		writeRegisterByte(SFE_KX13X_INC5, tempVal);
	}

	return true; 
}


//////////////////////////////////////////////////
// setPulseWidth()
//
// This determines which interrupt is routed to a particular physical
// interrupt pin.
//
// Parameters:
// rdr - The selected interrupt - watermark, tap/double tap, tilt, data ready etc.
// pin - The physical hardware pin that will receive the interrupt. 
//
bool QwDevKX13X::routeHardwareInterrupt(uint8_t rdr, uint8_t pin)
{

  int retVal;

	if( pin > 2 )
		return false;

  if( pin == 1 )
	{
    retVal = writeRegisterByte(SFE_KX13X_INC4, rdr);

    if( retVal != 0 )
      return false;
  }

  if( pin == 2 ) 
	{
    retVal = writeRegisterByte(SFE_KX13X_INC6, rdr);

    if( retVal != 0 )
      return false;
    
  }

  return true;

}


//////////////////////////////////////////////////
// clearInterrupt()
//
// Clears any latched interrupt by reading the INT_REL register.
//
bool QwDevKX13X::clearInterrupt()
{
  
  int retVal;
  uint8_t tempVal;

  retVal = readRegisterRegion(SFE_KX13X_INT_REL, &tempVal, 1);

  if( retVal != 0 )
		 return false;
	
	return true;
}

//////////////////////////////////////////////////
// enableDirecTapInterupt()
//
// Enables reporting on the direction of the latest generated tap. 
//
// Parameter: 
// enable - enables/disables directional tap reporting. 
//
bool QwDevKX13X::enableDirecTapInterupt(bool enable)
{
	int retVal;
	uint8_t tempVal;

  retVal = readRegisterRegion(SFE_KX13X_TDTRC, &tempVal, 1);

	if( retVal != 0 )
		return false;

	tempVal = tempVal | enable;

	retVal = writeRegisterByte(SFE_KX13X_TDTRC, tempVal);

	if( retVal != 0 )
		return false;

	return true; 
}


//////////////////////////////////////////////////
// enableDirecTapInterupt()
//
// Enables the double tap interrupt. 
//
// Parameter: 
// enable - enables/disables the double tap interrupt
//
bool QwDevKX13X::enableDoubleTapInterrupt(bool enable)
{
	int retVal;
	uint8_t tempVal;

  retVal = readRegisterRegion(SFE_KX13X_TDTRC, &tempVal, 1);

	if( retVal != 0 )
		return false;

	tempVal = tempVal | (enable << 1);

	retVal = writeRegisterByte(SFE_KX13X_TDTRC, tempVal);

	if( retVal != 0 )
		return false;

	return true; 
}

//////////////////////////////////////////////////
// dataReady()
//
// Checks the data ready bit indicating new accelerometer data
// is ready in the X/Y/Z Out regsiters. This is cleared automatically
// on read. 
//
//
bool QwDevKX13X::dataReady()
{
  
  int retVal;
  uint8_t tempVal;

  retVal = readRegisterRegion(SFE_KX13X_INS2, &tempVal, 1);

  if( retVal != 0 )
		return false;

	if( tempVal & 0x10 )
		return true;

	return false;
}

//////////////////////////////////////////////////
// freeFall()
//
// Checks the free fall interrupt bit indicating free fall
// has been detected.
//
bool QwDevKX13X::freeFall()
{
  
  int retVal;
  uint8_t tempVal;

  retVal = readRegisterRegion(SFE_KX13X_INS2, &tempVal, 1);

  if( retVal != 0 )
		return false;

	if( tempVal & 0x80 )
		return true;

	return false;
}


//////////////////////////////////////////////////
// bufferFull()
//
// Checks the buffer full interrupt bit indicating that the 
// buff is full. 
//
bool QwDevKX13X::bufferFull()
{
  
  int retVal;
  uint8_t tempVal;

  retVal = readRegisterRegion(SFE_KX13X_INS2, &tempVal, 1);

  if( retVal != 0 )
		return false;

	if( tempVal & 0x40 )
		return true;

	return false;
}


//////////////////////////////////////////////////
// waterMarkReached()
//
// Checks the watermark interrupt bit indicating it has been reached.
// buff is full. 
//
bool QwDevKX13X::waterMarkReached()
{
  
  int retVal;
  uint8_t tempVal;

  retVal = readRegisterRegion(SFE_KX13X_INS2, &tempVal, 1);

  if( retVal != 0 )
		return false;

	if( tempVal & 0x10 )
		return true;

	return false;
}


//////////////////////////////////////////////////
// tapDetected()
//
// Checks the tap interrupt bit indicating that a tap has
// been detected. 
//
bool QwDevKX13X::tapDetected()
{
  
  int retVal;
  uint8_t tempVal;

  retVal = readRegisterRegion(SFE_KX13X_INS2, &tempVal, 1);

  if( retVal != 0 )
		return false;


	tempVal = tempVal & 0x0C; // Three states of interest: single tap detected
														// undefined, and no tap.
	
	if( tempVal == 0x04 ) // True if tap - not undefined or no tap.
		return true;

	return false;
}

//////////////////////////////////////////////////
// getDirection()
//
// If the tap direction bit is enabled, this register will report
// the direction of the detected tap. 
//
int8_t QwDevKX13X::getDirection()
{
  
  int retVal;
  uint8_t tempVal;

  retVal = readRegisterRegion(SFE_KX13X_INS1, &tempVal, 1);

  if( retVal != 0 )
		return retVal;

	return tempVal;
}


//////////////////////////////////////////////////
// unknowntap()
//
// if the accelerometer is unsure whether it has in fact 
// detected a tap, it will report an "unknown" state. in that
// case this function will return true. good for error checking. 
// 
bool QwDevKX13X::unknownTap()
{
  
  int retVal;
  uint8_t tempVal;

  retVal = readRegisterRegion(SFE_KX13X_INS2, &tempVal, 1);

  if( retVal != 0 )
		return false;

	tempVal = tempVal & 0x0C; // Three states of interest: single tap detected
														// undefined, and no tap.
	
	if( tempVal == 0x0C ) // True if undefined
		return true;

	return false;
}


//////////////////////////////////////////////////
// doubleTapDetected()
//
// Checks the double tap interrupt bit indicating that 
// a double tap has been detected. 
// 
bool QwDevKX13X::doubleTapDetected()
{
  
  int retVal;
  uint8_t tempVal;

  retVal = readRegisterRegion(SFE_KX13X_INS2, &tempVal, 1);

  if( retVal != 0 )
		return false;

	tempVal = tempVal & 0x0C; // Two states of interest: single tap detected
														// and undefined.
	
	if( tempVal == 0x08 ) // True if tap - not undefined.
		return true;

	return false;
}


//////////////////////////////////////////////////
// tiltChange()
//
// Checks the tilt change interrupt bit indicating that 
// the accelerometer has been tipped. 
// 
bool QwDevKX13X::tiltChange()
{
  
  int retVal;
  uint8_t tempVal;

  retVal = readRegisterRegion(SFE_KX13X_INS2, &tempVal, 1);

  if( retVal != 0 )
		return false;

	if( tempVal == 0x01 ) 
		return true;

	return false;
}


//////////////////////////////////////////////////
// setBufferThreshold()
//
// Sets the number of samples that can be held in the buffer. 
// 
// Parameter:
// threshold - This value determines the number of samples that 
// will be store in the buffer. Can not exceed 171 for 8 bit resolution
// and 86 for 16 bit resolution. 
//
bool QwDevKX13X::setBufferThreshold(uint8_t threshold)
{

  int retVal;
  uint8_t tempVal;
  uint8_t resolution;

  if( threshold < 2 || threshold > 171 )
    return false;

  retVal = readRegisterRegion(SFE_KX13X_BUF_CNTL2, &tempVal, 1);

  if( retVal != 0 )
    return false;

  resolution = (tempVal & 0x40) >> 6; // Isolate it, move it

  if( threshold > 86 && resolution == 1 ) // 1 = 16bit resolution, max samples: 86
    threshold = 86; 
  
  retVal = writeRegisterByte(SFE_KX13X_BUF_CNTL1, threshold);

  if( retVal != 0 )
    return true;

	return false;

}


//////////////////////////////////////////////////
// setBufferOperationMode()
//
// Sets the opertion mode of the Buffer: Bypass, FIFO, Stream, Trigger 
// 
// Parameter:
// operationMode - Determines the operation mode to set. 
//
bool QwDevKX13X::setBufferOperationMode(uint8_t operationMode)
{

  int retVal;
	uint8_t tempVal; 

  if( operationMode > 2 )
    return false; 

  retVal = readRegisterRegion(SFE_KX13X_BUF_CNTL2, &tempVal, 1);

  if( retVal  != 0 )
    return true;

	tempVal = tempVal | operationMode;

  retVal = writeRegisterByte(SFE_KX13X_BUF_CNTL2, tempVal);

  if( retVal != 0 )
    return false;

	return true;
}


//////////////////////////////////////////////////
// setBufferResolution()
//
// Sets the resoltuion of the data that is stored in the buffer: 8 or 16 bit. 
// 
// Parameter:
// sixteenBit - Determines whether the resolution is 16 (true) or 8 bit (false). 
//
bool QwDevKX13X::setBufferResolution(bool sixteenBit )
{
	int retVal;
	uint8_t tempVal;

	retVal = readRegisterRegion(SFE_KX13X_BUF_CNTL2, &tempVal, 1);

	if( retVal != 0 )
		return false;

	tempVal = tempVal | ((uint8_t)sixteenBit << 6);

	retVal = writeRegisterByte(SFE_KX13X_BUF_CNTL2, tempVal);

	if( retVal != 0 )
		return false;

	return true;
}



//////////////////////////////////////////////////
// enableBufferInt()
//
// Enables the buffer full interrupt bit. 
// 
// Parameter:
// enable - enable/disables the buffer full interrupt bit. 
//
bool QwDevKX13X::enableBufferInt(bool enable)
{
	int retVal;
	uint8_t tempVal;

	retVal = readRegisterRegion(SFE_KX13X_BUF_CNTL2, &tempVal, 1);

	if( retVal != 0 )
		return false;

	tempVal = tempVal | (enable << 5);

	retVal = writeRegisterByte(SFE_KX13X_BUF_CNTL2, tempVal);

	if( retVal != 0 )
		return false;

	return true;
}


//////////////////////////////////////////////////
// enableSampleBuffer()
//
// Enables use of the buffer.
// 
// Parameter:
// enable - enable/disables the buffer. 
//
bool QwDevKX13X::enableSampleBuffer(bool enable)
{
	int retVal;
	uint8_t tempVal;

	retVal = readRegisterRegion(SFE_KX13X_BUF_CNTL2, &tempVal, 1);

	if( retVal != 0 )
		return false;

	tempVal = tempVal | ((uint8_t)enable << 7);

	retVal = writeRegisterByte(SFE_KX13X_BUF_CNTL2, tempVal);

	if( retVal != 0 )
		return false;

	return true;
}



//////////////////////////////////////////////////
// getSampleLevel()
//
// Gets the number of samples in the Buffer. 
//
uint16_t QwDevKX13X::getSampleLevel()
{
	int retVal;
	uint8_t tempVal[2] = {0};
	uint16_t numSamples; 

	retVal = readRegisterRegion(SFE_KX13X_BUF_STATUS_1, tempVal, 2);

	if( retVal != 0 )
		return 0;

	numSamples = tempVal[0]; 
	numSamples = numSamples | ((tempVal[1] & 0x03) << 8); 

	return numSamples;
}


//////////////////////////////////////////////////
// clearBuffer()
//
// Clears the samples in the buffer.
//
bool QwDevKX13X::clearBuffer()
{
	int retVal;
	uint8_t clear = 1;

	retVal = writeRegisterByte(SFE_KX13X_BUF_CLEAR, clear); 

	if( retVal != 0 )
		return false;

	return true;
}

//////////////////////////////////////////////////
// runCommandTest()
//
// Runs the command test which verifies the circuitry connected to 
// the accelerometer.
//
bool QwDevKX13X::runCommandTest()
{
  
  uint8_t tempVal;
  int retVal;

  retVal = readRegisterRegion(SFE_KX13X_CNTL2, &tempVal, 1);

  if( retVal != 0 )
    return false;

	tempVal = tempVal | 0x40;

	// Going to assume that communication is working at this point.
  writeRegisterByte(SFE_KX13X_CNTL2, tempVal);

  readRegisterRegion(SFE_KX13X_COTR, &tempVal, 1);

  if( tempVal != 0xAA )
    return false;
	
  readRegisterRegion(SFE_KX13X_CNTL2, &tempVal, 1);

	if( tempVal != 0 )
		return false;
	
  readRegisterRegion(SFE_KX13X_COTR, &tempVal, 1);

  if( tempVal != 0x55 )
    return false;

	return true; 
}


//////////////////////////////////////////////////
// getRawAccelData()
//
// Retrieves the raw register values representing accelerometer data. 
// 
// Paramater:
// *rawAccelData - a pointer to the data struct that holds acceleromter X/Y/Z data. 
//
bool QwDevKX13X::getRawAccelData(rawOutputData *rawAccelData){

  
  int retVal;
  uint8_t tempVal;
  uint8_t tempRegData[6] = {0}; 

	// Check if buffer is enabled
  retVal = readRegisterRegion(SFE_KX13X_INC4, &tempVal, 1);

  if( retVal != 0 )
    return false;

  if( tempVal & 0x40 )// If Buffer is enabled, read there.
    retVal = readRegisterRegion(SFE_KX13X_BUF_READ, tempRegData, 6);
  else
    retVal = readRegisterRegion(SFE_KX13X_XOUT_L, tempRegData, 6);

  if( retVal != 0 )
    return false;

	rawAccelData->xData = tempRegData[XLSB]; 
	rawAccelData->xData |= (uint16_t)((tempRegData[XMSB]) << 8); 
	rawAccelData->yData = tempRegData[YLSB]; 
	rawAccelData->yData |= (uint16_t)((tempRegData[YMSB]) << 8); 
	rawAccelData->zData = tempRegData[ZLSB]; 
	rawAccelData->zData |= ((uint16_t)(tempRegData[ZMSB]) << 8); 

  return true;
}

//////////////////////////////////////////////////
// forceSleep()
//
// Forces the accelerometer into a sleep state.
// 
bool QwDevKX13X::forceSleep()
{
	int retVal;
	uint8_t tempVal;
	uint8_t forceSleep = 0x01; 

	retVal = readRegisterRegion(SFE_KX13X_CNTL5, &tempVal, 1); 

	if( retVal != 0 )
		return false;

	tempVal |= forceSleep; 

	retVal = writeRegisterByte(SFE_KX13X_CNTL5, tempVal);

	if( retVal != 0 )
		return false;

	return true; 
}

//////////////////////////////////////////////////
// forceWake()
//
// Forces the accelerometer into a sleep state.
// 
bool QwDevKX13X::forceWake()
{
	int retVal;
	uint8_t tempVal;
	uint8_t forceWake = 0x02; 

	retVal = readRegisterRegion(SFE_KX13X_CNTL5, &tempVal, 1); 

	if( retVal != 0 )
		return false;

	tempVal |= forceWake; 

	retVal = writeRegisterByte(SFE_KX13X_CNTL5, tempVal);

	if( retVal != 0 )
		return false;

	return true; 
}

//////////////////////////////////////////////////////////////////////////////////
// readRegisterRegion()
//
// Calls sfebus read function.
//
//  Parameter:    
//  reg- register to read from
//  data- array to store data in
//  length- Size of data in bytes (8 bits): 2 bytes = length of two
//  retval- -1 = error, 0 = success
//
int QwDevKX13X::readRegisterRegion(uint8_t reg, uint8_t *data, uint16_t len)
{
	return (int)_sfeBus->readRegisterRegion(_i2cAddress, reg, data, len);
}

//////////////////////////////////////////////////////////////////////////////////
// writeRegisterRegion()
//
// Calls sfebus write function.
//
//  Parameter:    
//  reg- register to read from
//  data- array to store data in
//  length- Size of data in bytes (8 bits): 2 bytes = length of two
//  retval- -1 = error, 0 = success
//
int QwDevKX13X::writeRegisterRegion(uint8_t reg, uint8_t *data, uint16_t len)
{
	return (int)_sfeBus->writeRegisterRegion(_i2cAddress, reg, data, len);
}

//////////////////////////////////////////////////////////////////////////////////
// writeRegisterByte()
//
// Calls sfebus write function.
//
//  Parameter:    
//  reg- register to read from
//  data- array to store data in
//  length- Size of data in bytes (8 bits): 2 bytes = length of two
//  retval- -1 = error, 0 = success
//
int QwDevKX13X::writeRegisterByte(uint8_t reg, uint8_t data)
{
	return (int)_sfeBus->writeRegisterByte(_i2cAddress, reg, data);
}


//***************************************** KX132 *********************************************************


//////////////////////////////////////////////////////////////////////////////////
// init()
//
// Ensures that communication is established with the accelerometer by pinging its 
// address and retrieving its device ID.
//
bool QwDevKX132::init(void)
{
  if( !_sfeBus->ping(_i2cAddress) )
		return false;

	if( getUniqueID() != KX132_WHO_AM_I )
		return false;

	return true; 
}


//////////////////////////////////////////////////////////////////////////////////
// getAccelData()
//
// Retrieves the raw accelerometer data and calls a conversion function to convert the raw values.
// 
// Paramater:
// *userData - a pointer to the user's data struct that will hold acceleromter data.
//
bool QwDevKX132::getAccelData(outputData *userData){
  
	bool retVal;

  retVal = getRawAccelData(&rawAccelData);

	if( !retVal )
		return false;

	retVal = convAccelData(userData, &rawAccelData);

	if( !retVal )
		return false;

	return true; 
}

//////////////////////////////////////////////////////////////////////////////////
// convAccelData()
//
// Converts raw acceleromter data with the current accelerometer's range settings.
// 
// Paramater:
// *userData - a pointer to the user's data struct that will hold acceleromter data.
// *rawAccelData - a pointer to the data struct that holds acceleromter X/Y/Z data. 
//
bool QwDevKX132::convAccelData(outputData *userAccel, rawOutputData *rawAccelData){

  uint8_t regVal;
  uint8_t range; 
  int retVal;

  retVal = readRegisterRegion(SFE_KX13X_CNTL1, &regVal, 1);

  if( retVal != 0 )
    return false; 

  range = (regVal & 0x18) >> 3;
  

  switch( range ) {
    case SFE_KX132_RANGE2G:
      userAccel->xData = (float)rawAccelData->xData * convRange2G;
      userAccel->yData = (float)rawAccelData->yData * convRange2G;
      userAccel->zData = (float)rawAccelData->zData * convRange2G;
      break;
    case SFE_KX132_RANGE4G:
      userAccel->xData = (float)rawAccelData->xData * convRange4G;
      userAccel->yData = (float)rawAccelData->yData * convRange4G;
      userAccel->zData = (float)rawAccelData->zData * convRange4G;
      break;
    case SFE_KX132_RANGE8G:
      userAccel->xData = (float)rawAccelData->xData * convRange8G;
      userAccel->yData = (float)rawAccelData->yData * convRange8G;
      userAccel->zData = (float)rawAccelData->zData * convRange8G;
      break;
    case SFE_KX132_RANGE16G:
      userAccel->xData = (float)rawAccelData->xData * convRange16G;
      userAccel->yData = (float)rawAccelData->yData * convRange16G;
      userAccel->zData = (float)rawAccelData->zData * convRange16G;
      break;
		default:
			return false;
  }

  return true;
}

//***************************************** KX134 ******************************************************

//////////////////////////////////////////////////////////////////////////////////
// init()
//
// Ensures that communication is established with the accelerometer by pinging its 
// address and retrieving its device ID.
//
bool QwDevKX134::init(void)
{
  if( !_sfeBus->ping(_i2cAddress) )
		return false;

	if( getUniqueID() != KX134_WHO_AM_I )
		return false;

	return true; 
}


//////////////////////////////////////////////////////////////////////////////////
// getAccelData()
//
// Retrieves the raw accelerometer data and calls a conversion function to convert the raw values.
// 
// Paramater:
// *userData - a pointer to the user's data struct that will hold acceleromter data.
//
bool QwDevKX134::getAccelData(outputData *userData)
{
  
	bool retVal;

  retVal = getRawAccelData(&rawAccelData);

	if( !retVal )
		return false;

	retVal = convAccelData(userData, &rawAccelData);

	if( !retVal )
		return false;

	return true; 
}

//////////////////////////////////////////////////////////////////////////////////
// convAccelData()
//
// Converts raw acceleromter data with the current accelerometer's range settings.
// 
// Paramater:
// *userData - a pointer to the user's data struct that will hold acceleromter data.
// *rawAccelData - a pointer to the data struct that holds acceleromter X/Y/Z data. 
//
bool QwDevKX134::convAccelData(outputData *userAccel, rawOutputData *rawAccelData)
{

  uint8_t regVal;
  uint8_t range; 
  int retVal;

  retVal = readRegisterRegion(SFE_KX13X_CNTL1, &regVal, 1);

  if( retVal != 0 )
    return false; 

  range = (regVal & 0x18) >> 3;
  

  switch( range ) {
    case SFE_KX134_RANGE8G:
      userAccel->xData = (float)rawAccelData->xData * convRange8G;
      userAccel->yData = (float)rawAccelData->yData * convRange8G;
      userAccel->zData = (float)rawAccelData->zData * convRange8G;
      break;                                               
    case SFE_KX134_RANGE16G:                                   
      userAccel->xData = (float)rawAccelData->xData * convRange16G;
      userAccel->yData = (float)rawAccelData->yData * convRange16G;
      userAccel->zData = (float)rawAccelData->zData * convRange16G;
      break;                                               
    case SFE_KX134_RANGE32G:                                   
      userAccel->xData = (float)rawAccelData->xData * convRange32G;
      userAccel->yData = (float)rawAccelData->yData * convRange32G;
      userAccel->zData = (float)rawAccelData->zData * convRange32G;
      break;                                               
    case SFE_KX134_RANGE64G:                                   
      userAccel->xData = (float)rawAccelData->xData * convRange64G;
      userAccel->yData = (float)rawAccelData->yData * convRange64G;
      userAccel->zData = (float)rawAccelData->zData * convRange64G;
      break;
		default:
			return false;

  }

  return true;
}


