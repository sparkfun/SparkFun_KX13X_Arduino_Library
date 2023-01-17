#include "SparkFun_Qwiic_KX13X.h"

uint8_t QwDevKX13X::getUniqueID()
{
  uint8_t tempVal;
  int retVal = readRegisterRegion(SFE_KX13X_WHO_AM_I, &tempVal, 1);

  if (retVal != 0)
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

  int retVal = 0;

  if (!enableAccel(true))
    return false;

  if (settings == DEFAULT_SETTINGS)
    retVal = writeRegisterByte(SFE_KX13X_CNTL1, DEFAULT_SETTINGS);

  if (settings == INT_SETTINGS)
  {
    enablePhysInterrupt();
    routeHardwareInterrupt(0x10);
    retVal = writeRegisterByte(SFE_KX13X_CNTL1, INT_SETTINGS);
  }

  if (settings == BUFFER_SETTINGS)
  {
    enablePhysInterrupt();
    routeHardwareInterrupt(0x40); // Buffer full interrupt
    enableSampleBuffer();         // Enable buffer
    setBufferOperationMode(0x00); // FIFO
    retVal = writeRegisterByte(SFE_KX13X_CNTL1, INT_SETTINGS);
  }

  if (retVal != 0)
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

  sfe_kx13x_cntl2_bitfield_t cntl2;
  cntl2.all = 0;
  cntl2.bits.srst = 1; // This is a long winded, but definitive way of setting the software reset bit

  int retVal;

  retVal = writeRegisterByte(SFE_KX13X_CNTL2, cntl2.all);

  // Logic is inverted here - if we reset using I2C the
  // accelerometer immediately shuts off which results
  // in a NACK.
  if (retVal != 0)
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

  if (retVal != 0)
    return false;

  sfe_kx13x_cntl1_bitfield_t cntl1;
  cntl1.all = tempVal;
  cntl1.bits.pc1 = enable; // This is a long winded but definitive way of setting/clearing the operating mode bit
  tempVal = cntl1.all;

  retVal = writeRegisterByte(SFE_KX13X_CNTL1, tempVal);

  if (retVal != 0)
    return false;

  return true;
}

//////////////////////////////////////////////////
// getOperatingMode()
//
// Retrieves the current operating mode - low/high power mode
//

int8_t QwDevKX13X::getOperatingMode()
{

  uint8_t tempVal;
  int retVal;

  retVal = readRegisterRegion(SFE_KX13X_CNTL1, &tempVal, 1);

  if (retVal != 0)
    return retVal;

  sfe_kx13x_cntl1_bitfield_t cntl1;
  cntl1.all = tempVal; // This is a long winded but definitive way of getting the operating mode bit

  return (cntl1.bits.pc1); // Return the operating mode bit
}

//////////////////////////////////////////////////
// setRange()
//
// Sets the operational g-range of the accelerometer.
//
// Parameter:
// range - sets the range of the accelerometer 2g - 32g depending
// on the version. 8g - 64g for the KX134.
//
bool QwDevKX13X::setRange(uint8_t range)
{

  uint8_t tempVal;
  int retVal;

  if (range > SFE_KX132_RANGE16G) // Same as SFE_KX134_RANGE64G
    return false;

  // Read - Modify - Write
  retVal = readRegisterRegion(SFE_KX13X_CNTL1, &tempVal, 1);

  if (retVal != 0)
    return false;

  sfe_kx13x_cntl1_bitfield_t cntl1;
  cntl1.all = tempVal;
  cntl1.bits.gsel =  range; // This is a long winded but definitive way of setting the range (g select)
  tempVal = cntl1.all;

  retVal = writeRegisterByte(SFE_KX13X_CNTL1, tempVal);

  if (retVal != 0)
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

  if (retVal != 0)
    return false;

  sfe_kx13x_cntl1_bitfield_t cntl1;
  cntl1.all = tempVal;
  cntl1.bits.drdye =  enable; // This is a long winded but definitive way of setting/clearing the data ready engine bit
  tempVal = cntl1.all;

  retVal = writeRegisterByte(SFE_KX13X_CNTL1, tempVal);

  if (retVal != 0)
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

  if (retVal != 0)
    return false;

  sfe_kx13x_cntl1_bitfield_t cntl1;
  cntl1.all = tempVal;
  cntl1.bits.tdte =  enable; // This is a long winded but definitive way of setting/clearing the tap engine bit
  tempVal = cntl1.all;

  retVal = writeRegisterByte(SFE_KX13X_CNTL1, tempVal);

  if (retVal != 0)
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

  if (retVal != 0)
    return false;

  sfe_kx13x_cntl1_bitfield_t cntl1;
  cntl1.all = tempVal;
  cntl1.bits.tpe =  enable; // This is a long winded but definitive way of setting/clearing the tilt engine bit
  tempVal = cntl1.all;

  retVal = writeRegisterByte(SFE_KX13X_CNTL1, tempVal);

  if (retVal != 0)
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

  if (retVal != 0)
    return false;

  sfe_kx13x_cntl4_bitfield_t cntl4;
  cntl4.all = tempVal;
  cntl4.bits.wufe =  enable; // This is a long winded but definitive way of setting/clearing the wake-up engine bit
  tempVal = cntl4.all;

  retVal = writeRegisterByte(SFE_KX13X_CNTL4, tempVal);

  if (retVal != 0)
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

  if (retVal != 0)
    return false;

  sfe_kx13x_cntl4_bitfield_t cntl4;
  cntl4.all = tempVal;
  cntl4.bits.btse =  enable; // This is a long winded but definitive way of setting/clearing the back-to-sleep engine bit
  tempVal = cntl4.all;

  retVal = writeRegisterByte(SFE_KX13X_CNTL4, tempVal);

  if (retVal != 0)
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

  if (rate > 15)
    return false;

  uint8_t tempVal;
  int retVal;

  retVal = readRegisterRegion(SFE_KX13X_ODCNTL, &tempVal, 1);

  if (retVal != 0)
    return false;

  sfe_kx13x_odcntl_bitfield_t odcntl;
  odcntl.all = tempVal;
  odcntl.bits.osa =  rate; // This is a long winded but definitive way of updating the ODR
  tempVal = odcntl.all;

  retVal = writeRegisterByte(SFE_KX13X_ODCNTL, tempVal);

  if (retVal != 0)
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

  if (rate > 7)
    return false;

  uint8_t tempVal;
  int retVal;

  retVal = readRegisterRegion(SFE_KX13X_CNTL3, &tempVal, 1);

  if (retVal != 0)
    return false;

  sfe_kx13x_cntl3_bitfield_t cntl3;
  cntl3.all = tempVal;
  cntl3.bits.otdt = rate; // This is a long winded but definitive way of updating the tap ODR
  tempVal = cntl3.all;

  retVal = writeRegisterByte(SFE_KX13X_CNTL3, tempVal);

  if (retVal != 0)
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

  if (rate > 3)
    return false;

  uint8_t tempVal;
  int retVal;

  retVal = readRegisterRegion(SFE_KX13X_CNTL3, &tempVal, 1);

  if (retVal != 0)
    return false;

  sfe_kx13x_cntl3_bitfield_t cntl3;
  cntl3.all = tempVal;
  cntl3.bits.otp = rate; // This is a long winded but definitive way of updating the tap ODR
  tempVal = cntl3.all;

  retVal = writeRegisterByte(SFE_KX13X_CNTL3, tempVal);

  if (retVal != 0)
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

  if (rate > 7)
    return false;

  uint8_t tempVal;
  int retVal;

  retVal = readRegisterRegion(SFE_KX13X_CNTL3, &tempVal, 1);

  if (retVal != 0)
    return false;

  sfe_kx13x_cntl3_bitfield_t cntl3;
  cntl3.all = tempVal;
  cntl3.bits.owuf = rate; // This is a long winded but definitive way of updating the wake-up ODR
  tempVal = cntl3.all;

  retVal = writeRegisterByte(SFE_KX13X_CNTL3, tempVal);

  if (retVal != 0)
    return false;

  return true;
}
//////////////////////////////////////////////////
// getOutputDataRate()
//
// Retrieves the output data rate of the accelerometer.
//
float QwDevKX13X::getOutputDataRate()
{
  int retVal;
  uint8_t tempVal;

  retVal = readRegisterRegion(SFE_KX13X_ODCNTL, &tempVal, 1);

  if (retVal != 0)
    return 0.0;

  sfe_kx13x_odcntl_bitfield_t odcntl;
  odcntl.all = tempVal; // This is a long winded but definitive way of getting the ODR

  return (0.781 * (pow(2, (float)odcntl.bits.osa)));
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
bool QwDevKX13X::configureInterruptPin(uint8_t pinVal)
{

  int retVal;

  retVal = writeRegisterByte(SFE_KX13X_INC1, pinVal);

  if (retVal != 0)
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

  if (pin > 2)
    return false;

  if (pin == 1)
  {
    retVal = readRegisterRegion(SFE_KX13X_INC1, &tempVal, 1);

    if (retVal != 0)
      return false;

    sfe_kx13x_inc1_bitfield_t inc1;
    inc1.all = tempVal;
    inc1.bits.ien1 = enable; // This is a long winded but definitive way of setting/clearing the enable bit
    tempVal = inc1.all;

    writeRegisterByte(SFE_KX13X_INC1, tempVal);
  }

  if (pin == 2)
  {
    retVal = readRegisterRegion(SFE_KX13X_INC5, &tempVal, 1);

    if (retVal != 0)
      return false;

    sfe_kx13x_inc5_bitfield_t inc5;
    inc5.all = tempVal;
    inc5.bits.ien2 = enable; // This is a long winded but definitive way of setting/clearing the enable bit
    tempVal = inc5.all;

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
bool QwDevKX13X::setPinMode(bool activeHigh, uint8_t pin)
{
  int retVal;
  uint8_t tempVal;

  if (pin > 2)
    return false;

  if (pin == 1)
  {
    retVal = readRegisterRegion(SFE_KX13X_INC1, &tempVal, 1);

    if (retVal != 0)
      return false;

    sfe_kx13x_inc1_bitfield_t inc1;
    inc1.all = tempVal;
    inc1.bits.iea1 = activeHigh; // This is a long winded but definitive way of setting/clearing the level bit
    tempVal = inc1.all;

    writeRegisterByte(SFE_KX13X_INC1, tempVal);
  }

  if (pin == 2)
  {
    retVal = readRegisterRegion(SFE_KX13X_INC5, &tempVal, 1);

    if (retVal != 0)
      return false;

    sfe_kx13x_inc5_bitfield_t inc5;
    inc5.all = tempVal;
    inc5.bits.iea2 = activeHigh; // This is a long winded but definitive way of setting/clearing the level bit
    tempVal = inc5.all;

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
// latch - False enables latch behavior, True enables pulse behavior (default)
//
bool QwDevKX13X::setLatchControl(bool pulsed, uint8_t pin)
{
  int retVal;
  uint8_t tempVal;

  if (pin > 2)
    return false;

  if (pin == 1)
  {
    retVal = readRegisterRegion(SFE_KX13X_INC1, &tempVal, 1);

    if (retVal != 0)
      return false;

    sfe_kx13x_inc1_bitfield_t inc1;
    inc1.all = tempVal;
    inc1.bits.iel1 = pulsed; // This is a long winded but definitive way of setting/clearing the latch bit
    tempVal = inc1.all;

    writeRegisterByte(SFE_KX13X_INC1, tempVal);
  }

  if (pin == 2)
  {
    retVal = readRegisterRegion(SFE_KX13X_INC5, &tempVal, 1);

    if (retVal != 0)
      return false;

    sfe_kx13x_inc5_bitfield_t inc5;
    inc5.all = tempVal;
    inc5.bits.iel2 = pulsed; // This is a long winded but definitive way of setting/clearing the latch bit
    tempVal = inc5.all;

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

  if ((width > 3) || (pin > 2))
    return false;

  if (pin == 1)
  {
    retVal = readRegisterRegion(SFE_KX13X_INC1, &tempVal, 1);

    if (retVal != 0)
      return false;

    sfe_kx13x_inc1_bitfield_t inc1;
    inc1.all = tempVal;
    inc1.bits.pw1 = width; // This is a long winded but definitive way of setting the pulse width
    tempVal = inc1.all;

    writeRegisterByte(SFE_KX13X_INC1, tempVal);
  }

  if (pin == 2)
  {
    retVal = readRegisterRegion(SFE_KX13X_INC5, &tempVal, 1);

    if (retVal != 0)
      return false;

    sfe_kx13x_inc5_bitfield_t inc5;
    inc5.all = tempVal;
    inc5.bits.pw2 = width; // This is a long winded but definitive way of setting the pulse width
    tempVal = inc5.all;

    writeRegisterByte(SFE_KX13X_INC5, tempVal);
  }

  return true;
}

//////////////////////////////////////////////////
// routeHardwareInterrupt()
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

  if (pin > 2)
    return false;

  if (pin == 1)
  {
    retVal = writeRegisterByte(SFE_KX13X_INC4, rdr);

    if (retVal != 0)
      return false;
  }

  if (pin == 2)
  {
    retVal = writeRegisterByte(SFE_KX13X_INC6, rdr);

    if (retVal != 0)
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

  if (retVal != 0)
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

  if (retVal != 0)
    return false;

  sfe_kx13x_tdtrc_bitfield_t tdtrc;
  tdtrc.all = tempVal;
  tdtrc.bits.stre = enable; // This is a long winded but definitive way of setting/clearing the enable bit
  tempVal = tdtrc.all;

  retVal = writeRegisterByte(SFE_KX13X_TDTRC, tempVal);

  if (retVal != 0)
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

  if (retVal != 0)
    return false;

  sfe_kx13x_tdtrc_bitfield_t tdtrc;
  tdtrc.all = tempVal;
  tdtrc.bits.dtre = enable; // This is a long winded but definitive way of setting/clearing the enable bit
  tempVal = tdtrc.all;

  retVal = writeRegisterByte(SFE_KX13X_TDTRC, tempVal);

  if (retVal != 0)
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

  if (retVal != 0)
    return false;

  sfe_kx13x_ins2_bitfield_t ins2;
  ins2.all = tempVal;

  return ins2.bits.drdy;
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

  if (retVal != 0)
    return false;

  sfe_kx13x_ins2_bitfield_t ins2;
  ins2.all = tempVal;

  return ins2.bits.ffs;
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

  if (retVal != 0)
    return false;

  sfe_kx13x_ins2_bitfield_t ins2;
  ins2.all = tempVal;

  return ins2.bits.bfi;
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

  if (retVal != 0)
    return false;

  sfe_kx13x_ins2_bitfield_t ins2;
  ins2.all = tempVal;

  return ins2.bits.wmi;
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

  if (retVal != 0)
    return false;

  sfe_kx13x_ins2_bitfield_t ins2;
  ins2.all = tempVal;

  return (ins2.bits.tdts == 0x01); // Single tap
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

  if (retVal != 0)
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

  if (retVal != 0)
    return false;

  sfe_kx13x_ins2_bitfield_t ins2;
  ins2.all = tempVal;

  return (ins2.bits.tdts == 0x03); // undefined tap event
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

  if (retVal != 0)
    return false;

  sfe_kx13x_ins2_bitfield_t ins2;
  ins2.all = tempVal;

  return (ins2.bits.tdts == 0x02); // Double tap
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

  if (retVal != 0)
    return false;

  sfe_kx13x_ins2_bitfield_t ins2;
  ins2.all = tempVal;

  return (ins2.bits.tps); // Tilt position status
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

  if ((threshold < 2) || (threshold > 171))
    return false;

  retVal = readRegisterRegion(SFE_KX13X_BUF_CNTL2, &tempVal, 1);

  if (retVal != 0)
    return false;

  sfe_kx13x_buf_cntl2_bitfield_t bufCntl2;
  bufCntl2.all = tempVal;

  // BRES – determines the resolution of the acceleration data samples collected by the sample buffer.
  // BRES = 0 – 8-bit samples are accumulated in the buffer
  // BRES = 1 – 16-bit samples are accumulated in the buffer

  if ((threshold > 86) && (bufCntl2.bits.bres == 1)) // 1 = 16bit resolution, max samples: 86
    threshold = 86;

  retVal = writeRegisterByte(SFE_KX13X_BUF_CNTL1, threshold);

  if (retVal != 0)
    return false;

  return true;
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

  if (operationMode > 2)
    return false;

  retVal = readRegisterRegion(SFE_KX13X_BUF_CNTL2, &tempVal, 1);

  if (retVal != 0)
    return true;

  sfe_kx13x_buf_cntl2_bitfield_t bufCntl2;
  bufCntl2.all = tempVal;
  bufCntl2.bits.bm = operationMode; // This is a long winded but definitive way of setting/clearing the operating mode
  tempVal = bufCntl2.all;

  retVal = writeRegisterByte(SFE_KX13X_BUF_CNTL2, tempVal);

  if (retVal != 0)
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
bool QwDevKX13X::setBufferResolution(bool sixteenBit)
{
  int retVal;
  uint8_t tempVal;

  retVal = readRegisterRegion(SFE_KX13X_BUF_CNTL2, &tempVal, 1);

  if (retVal != 0)
    return false;

  sfe_kx13x_buf_cntl2_bitfield_t bufCntl2;
  bufCntl2.all = tempVal;
  bufCntl2.bits.bres = sixteenBit; // This is a long winded but definitive way of setting/clearing the resolution bit
  tempVal = bufCntl2.all;

  retVal = writeRegisterByte(SFE_KX13X_BUF_CNTL2, tempVal);

  if (retVal != 0)
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

  if (retVal != 0)
    return false;

  sfe_kx13x_buf_cntl2_bitfield_t bufCntl2;
  bufCntl2.all = tempVal;
  bufCntl2.bits.bfie = enable; // This is a long winded but definitive way of setting/clearing the buffer interrupt enable bit
  tempVal = bufCntl2.all;

  retVal = writeRegisterByte(SFE_KX13X_BUF_CNTL2, tempVal);

  if (retVal != 0)
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

  if (retVal != 0)
    return false;

  sfe_kx13x_buf_cntl2_bitfield_t bufCntl2;
  bufCntl2.all = tempVal;
  bufCntl2.bits.bufe = enable; // This is a long winded but definitive way of setting/clearing the buffer enable bit
  tempVal = bufCntl2.all;

  retVal = writeRegisterByte(SFE_KX13X_BUF_CNTL2, tempVal);

  if (retVal != 0)
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

  if (retVal != 0)
    return 0;

  numSamples = tempVal[0];
  numSamples |= (((uint16_t)tempVal[1] & 0x03) << 8);

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

  if (retVal != 0)
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

  if (retVal != 0)
    return false;

  sfe_kx13x_cntl2_bitfield_t cntl2;
  cntl2.all = tempVal;
  cntl2.bits.cotc = 1; // This is a long winded, but definitive way of setting the COTC bit
  tempVal = cntl2.all;

  // Going to assume that communication is working at this point.
  writeRegisterByte(SFE_KX13X_CNTL2, tempVal);

  readRegisterRegion(SFE_KX13X_COTR, &tempVal, 1);

  if (tempVal != 0xAA)
    return false;

  readRegisterRegion(SFE_KX13X_CNTL2, &tempVal, 1);

  cntl2.all = tempVal;
  if (cntl2.bits.cotc != 0)
    return false;

  readRegisterRegion(SFE_KX13X_COTR, &tempVal, 1);

  if (tempVal != 0x55)
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
bool QwDevKX13X::getRawAccelData(rawOutputData *rawAccelData)
{

  int retVal;
  uint8_t tempVal;
  uint8_t tempRegData[6] = {0};

  retVal = readRegisterRegion(SFE_KX13X_BUF_CNTL2, &tempVal, 1); // bufCntl2.bits.bufe indicates if the buffer is enabled

  if (retVal != 0)
    return false;

  sfe_kx13x_buf_cntl2_bitfield_t bufCntl2;
  bufCntl2.all = tempVal;

  bool is16bit = true;

  if (bufCntl2.bits.bufe) // If Buffer is enabled, read there.
  {
    if (getSampleLevel() > 0) // Check the buffer contains data
    {
      if (bufCntl2.bits.bres) // If the buffer contains 16-bit samples
        retVal = readRegisterRegion(SFE_KX13X_BUF_READ, tempRegData, 6); // Read 3 * 16-bit
      else
      {
        retVal = readRegisterRegion(SFE_KX13X_BUF_READ, tempRegData, 3); // Read 3 * 8-bit
        is16bit = false;
      }
    }
    else
      // No buffer data to read!
      // We can either:
      return false;
      // Or, be kind and read the normal registers
      //retVal = readRegisterRegion(SFE_KX13X_XOUT_L, tempRegData, 6); // Read 3 * 16-bit
  }
  else
    retVal = readRegisterRegion(SFE_KX13X_XOUT_L, tempRegData, 6); // Read 3 * 16-bit

  if (retVal != 0)
    return false;

  if (is16bit) // Process buffer 8-bit samples
  {
    rawAccelData->xData = tempRegData[XLSB];
    rawAccelData->xData |= (uint16_t)tempRegData[XMSB] << 8;
    rawAccelData->yData = tempRegData[YLSB];
    rawAccelData->yData |= (uint16_t)tempRegData[YMSB] << 8;
    rawAccelData->zData = tempRegData[ZLSB];
    rawAccelData->zData |= (uint16_t)tempRegData[ZMSB] << 8;
  }
  else
  {
    rawAccelData->xData = 0;
    rawAccelData->xData |= (uint16_t)tempRegData[0] << 8; // Convert 8-bit signed to 16-bit signed
    rawAccelData->yData = 0;
    rawAccelData->yData |= (uint16_t)tempRegData[1] << 8;
    rawAccelData->zData = 0;
    rawAccelData->zData |= (uint16_t)tempRegData[2] << 8;
  }

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

  retVal = readRegisterRegion(SFE_KX13X_CNTL5, &tempVal, 1);

  if (retVal != 0)
    return false;

  sfe_kx13x_cntl5_bitfield_t cntl5;
  cntl5.all = tempVal;
  cntl5.bits.man_sleep = 1; // Set the manual sleep bit
  tempVal = cntl5.all;

  retVal = writeRegisterByte(SFE_KX13X_CNTL5, tempVal);

  if (retVal != 0)
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

  retVal = readRegisterRegion(SFE_KX13X_CNTL5, &tempVal, 1);

  if (retVal != 0)
    return false;

  sfe_kx13x_cntl5_bitfield_t cntl5;
  cntl5.all = tempVal;
  cntl5.bits.man_wake = 1; // Set the manual wake bit
  tempVal = cntl5.all;

  retVal = writeRegisterByte(SFE_KX13X_CNTL5, tempVal);

  if (retVal != 0)
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
  if (!_sfeBus->ping(_i2cAddress))
    return false;

  if (getUniqueID() != KX132_WHO_AM_I)
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
bool QwDevKX132::getAccelData(outputData *userData)
{

  bool retVal;

  retVal = getRawAccelData(&rawAccelData);

  if (!retVal)
    return false;

  retVal = convAccelData(userData, &rawAccelData);

  if (!retVal)
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
bool QwDevKX132::convAccelData(outputData *userAccel, rawOutputData *rawAccelData)
{
  uint8_t regVal;
  uint8_t range;
  int retVal;

  retVal = readRegisterRegion(SFE_KX13X_CNTL1, &regVal, 1);

  if (retVal != 0)
    return false;

  sfe_kx13x_cntl1_bitfield_t cntl1;
  cntl1.all = regVal;

  range = cntl1.bits.gsel;

  switch (range)
  {
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
  if (!_sfeBus->ping(_i2cAddress))
    return false;

  if (getUniqueID() != KX134_WHO_AM_I)
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

  if (!retVal)
    return false;

  retVal = convAccelData(userData, &rawAccelData);

  if (!retVal)
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

  if (retVal != 0)
    return false;

  sfe_kx13x_cntl1_bitfield_t cntl1;
  cntl1.all = regVal;

  range = cntl1.bits.gsel;

  switch (range)
  {
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
