/******************************************************************************
SparkFun_Qwiic_KX13X.h
Andy England @ SparkFun Electronics
Original Creation Date: 1/8/2021

This file prototypes the QwiicPIR class, implemented in SparkFun_Qwiic_Button.cpp.

Development environment specifics:
	IDE: Arduino 1.8.12
	Hardware Platform: Arduino Uno/SparkFun Redboard
	Qwiic Button Breakout Version: 1.0.0
    Qwiic Switch Breakout Version: 1.0.0

This code is Lemonadeware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef __SparkFun_Qwiic_KX13X_H__
#define __SparkFun_Qwiic_KX13X_H__


#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <SPI.h>
#include <Wire.h>

#define KX13X_DEFAULT_ADDRESS 0x1F

#define MAN_ID 0x00
#define PART_ID 0x01

//Accelerometer Data
#define XADP_L 0x02 //ADP is the output for the Automatic Data PathConfigured in CNTL5, ADP_CNTL1 and ADP_CNTL2
#define XADP_H 0x03
#define YADP_L 0x04
#define YADP_H 0x05
#define ZADP_L 0x06
#define ZADP_H 0x07
#define XOUT_L 0x08
#define XOUT_H 0x09
#define YOUT_L 0x0A
#define YOUT_H 0x0B
#define ZOUT_L 0x0C
#define ZOUT_H 0x0D

//0x0E-0x11 Kionix Reserved

#define COTR 0x12 //Command Test Response
#define COTR_DEFAULT 0x55 //Default value of COTR Register
#define COTR_SUCCESS 0xAA //Succesfull COTR Register setting value
#define WHO_AM_I 0x13
#define TSCP 0x14 //Current Tilt Position Register
#define TSPP 0x15 //Current Tilt Position Register
#define INS1 0x16 //Contains tap and double tap axis specific interrupts
#define INS2 0x17 //Defines which function causes an interrupt
#define INS3 0x18 //Reports axis and direction of motion that triggered the wakeup interrupt
#define STATUS_REG 0x19 //Reports the status of the interrupt
#define INT_REL 0x1A //Reading will clear INS1-3
#define CNTL1 0x1B
#define PC1 7 //1:high performance/low power mode 0: stand by
#define RES 6 //1:high performance mode 0: Low Power mode
#define CNTL2 0x1C
#define COTC 6 //1: Sets COTR to 0xAA to verify proper IC functionality 0: no action
#define CNTL3 0x1D
#define CNTL4 0x1E
#define CNTL5 0x1F
#define CNTL6 0x2A //I2C Autorelease
#define ODCNTL 0x21
#define INC1 0x22 //Controls settings for INT1
#define INC2 0x23 //Defines behavior for Wake-Up Function and Back To Sleep
#define INC3 0x24 //Defines which axes can cause a tap based interrupt
#define INC4 0x25 //Controls which function triggers INT1
#define INC5 0x26
#define INC6 0x27 //Controls which function triggers INT2
#define TILT_TIMER 0x29 //Defines how long the sensor must be in the new tilt state for the tilt state to change.

//Tap/Double Tap Control Registers 0x2A-0x31
#define TDTRC 0x2A //Tap/Double Tap Report Control
#define TDTC 0x2B //Minimum time between tap events to determine double tap
#define TTH 0x2C //Represents the top of the jerk range for a tap
#define TTH 0x2D //Represents the bottom of the jerk range for a tap
#define FTD 0x2E //Duration of the tap event itself
#define STD 0x2F //Sets the total allowable duration of tapping for a full double tap event
#define TLT 0x30 //Tap detection latency timer
#define TWS 0x31 //Window for any full tap event

//Free Fall Control Registers
#define FFTH 0x32 //Free Fall Threshold
#define FFC 0x33 //Free fall counter, the amount of counts in free fall
#define FFCNTL 0x34 //Main controls for free fall engine

//Tilt Angle Control Registers 
#define TILT_ANGLE_LL 0x37 //Low level threshold for tilt detection
#define TILT_ANGLE_HL 0x38 //High level threshold for tilt detection
#define HYST_SET 0x39 //Hysteresis between rotations for tilt detection.

#define LP_CNTL1 0x3A //Sets number of averaged samples, affects power consumption as well as noise
#define LP_CNTL2 0x3B //Reduces the power consumption even further by doing digital shutdown.

//0x3C-0x48 Reserved

#define WUFTH 0x49 //Wake up function threshold <7:0>
#define BTSWUFTH 0x4A //Back to sleep function <10:8> and wake up function threshold <10:8>
#define BTSTH 0x4B //Back to sleep function threshold <7:0>
#define BTSC 0x4C //Back to sleep debounce counter
#define WUFC 0x4D //Debounce counter for the Qake-Up function engine
#define SELF_TEST 0x5D //The final step to enabling the self test function

//Output Buffer Registers (0x5E-0x63)
#define BUF_CNTL1 0x5E //Determines the number of samples that will trigger a watermark interupt
#define BUF_CNTL2 0x5F //Controls activation, resolution buffer full interrupt of the sample buffer
#define BUF_STATUS_1 0x60 //Sample Level <7:0>
#define BUF_STATUS_2 0x61 //The status of th buffer's trigger function <7> and Sample level <1:0>
#define BUF_CLEAR 0x62 //Clears buffer status and buffer
#define BUF_READ 0x63 //Reads data from the buffer according to setting in BUF_CNTL2

//Advanced Data Path Control Registers (0x64-0x76)
#define ADP_CNTL1 0x64 //Number of samples used to calculate RMS output as well as the output data rate
#define ADP_CNTL2 0x65 //Vwake up/back to sleep engine inputs
#define ADP_CNTL3 0x66 //Filter settings
#define ADP_CNTL4 0x67 //Filter settings
#define ADP_CNTL5 0x68 //Filter settings
#define ADP_CNTL6 0x69 //Filter settings
#define ADP_CNTL7 0x6A //Filter settings
#define ADP_CNTL8 0x6B //Filter settings
#define ADP_CNTL9 0x6C //Filter settings
#define ADP_CNTL10 0x6D //Filter settings
#define ADP_CNTL11 0x6E //Filter settings
#define ADP_CNTL12 0x6F //Filter settings
#define ADP_CNTL13 0x70 //Filter settings
#define ADP_CNTL14 0x71 //Filter settings
#define ADP_CNTL15 0x72 //Filter settings
#define ADP_CNTL16 0x73 //Filter settings
#define ADP_CNTL17 0x74 //Filter settings
#define ADP_CNTL18 0x75 //Filter settings
#define ADP_CNTL19 0x76 //Filter settings

#define XLSB 0
#define XMSB 1
#define YLSB 2
#define YMSB 3
#define ZLSB 4
#define ZMSB 5

class QwiicKX13X
{
	public:
		bool begin(uint8_t deviceAddress = KX13X_DEFAULT_ADDRESS, TwoWire &wirePort = Wire);
		bool beginSPI();
		bool initialize();
		bool runCommandTest();

		bool waitForI2C();
		bool waitForSPI();

		bool getReadings();
		int16_t getAccelX();
		int16_t getAccelY();
		int16_t getAccelZ();

		bool readBit(uint8_t regAddr, uint8_t bitAddr);
		bool writeBit(uint8_t regAddr, uint8_t bitAddr, bool bitToWrite);
		uint8_t readRegister(uint8_t addr);
		bool writeRegister(uint8_t startingRegister, uint8_t data);

		bool getData(uint8_t startingRegister, uint8_t * dataBuffer, uint8_t bytesToGet = 1);
	private:
		TwoWire *_i2cPort;		//The generic connection to user's chosen I2C hardware
		uint8_t _deviceAddress; //Keeps track of I2C address. setI2CAddress changes this.	
		SPIClass *_spiPort;			 //The generic connection to user's chosen SPI hardware
		unsigned long _spiPortSpeed; //Optional user defined port speed

		uint8_t _accelData[6] = {0};
};
#endif /* QWIIC_KX13X */