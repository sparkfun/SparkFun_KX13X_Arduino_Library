/******************************************************************************
SparkFun_Qwiic_KX13X.h
Andy England @ SparkFun Electronics
Elias Santistevan @ SparkFun Electronics
Original Creation Date: March 2021

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
#define KX13X_ALT_ADDRESS 0x1E

#define KX132_WHO_AM_I  0x3D
#define KX134_WHO_AM_I  0x46
#define TOTAL_ACCEL_DATA 48 //bytes
#define MAX_BUFFER_LENGTH 32 //bytes

struct outputData { 
  int16_t xData;
  int16_t yData;
  int16_t zData;
};

//Accelerometer Data
enum SHARED_REGISTERS {

  KX13X_MAN_ID = 0x00,//      Retuns "KION" in ASCII
  KX13X_PART_ID,//            Who am I + Silicon specific ID
  KX13X_XADP_L,//     ------- X, Y, and Z - High and Low bytes -----
  KX13X_XADP_H,
  KX13X_YADP_L,
  KX13X_YADP_H,
  KX13X_ZADP_L,
  KX13X_ZADP_H,
  KX13X_XOUT_L,
  KX13X_XOUT_H,
  KX13X_YOUT_L,
  KX13X_YOUT_H,
  KX13X_ZOUT_L,
  KX13X_ZOUT_H, //     --------------^^--------------------------
  //                          0x0E - 0x11 Reserved
  KX13X_COTR = 0x12,//        Command Test Register
  KX13X_WHO_AM_I, //          Who am I: 0x3D-KX132, 0x46-KX134
  KXI3X_TSCP,//        -------Tilt Register---------------------- 
  KXI3X_TSPP,//        -----------^^-----------------------------
  KXI3X_INS1 //        -------Interrupt Registers ---------------
  KXI3X_INS2,
  KXI3X_INS3,
  KXI3X_STATUS_REG, 
  KXI3X_INT_REL, //    ------------^^----------------------------
  KXI3X_CNTL1,//       --------Control Registers----------------- 
  KXI3X_CNTL2,
  KXI3X_CNTL3,
  KXI3X_CNTL4,
  KXI3X_CNTL5,
  KXI3X_CNTL6,//        -------------^^---------------------------
  KXI3X_ODCNTL,
  KXI3X_INC1,//Controls settings for INT1
  KXI3X_INC2,//Defines behavior for Wake-Up Function and Back To Sleep
  KXI3X_INC3,//Defines which axes can cause a tap based interrupt
  KXI3X_INC4,//Controls which function triggers INT1
  KXI3X_INC5,
  KXI3X_INC6,//Controls which function triggers INT2
  // 0x28 Reserved
  KXI3X_TILT_TIMER =  0x29, 
  KX13X_TDTRC, // Tap Control Regs ----- 
  KX13X_TDTC,
  KX13X_TDTC,
  KX13X_TTH,
  KX13X_TTL,
  KX13X_FTD,
  KX13X_STD,
  KX13X_TLT,
  KX13X_TWS,
  KX13X_FFTH,
  KX13X_FFC,
  KX13X_FFCNTL,
  // 0x35 - 0x36 Reserved
  KX13X_TILT_ANGLE_LL = 0x37,
  KX13X_TILT_ANGLE_HL,
  KX13X_HYST_SET,
  KX13X_LP_CNTL1,
  KX13X_LP_CNTL2,
  // 0x3C - 0x48 Reserved
  KX13X_WUFTH,
  KX13X_BTSWUFTH,
  KX13X_BTSTH,
  KX13X_BTSC,
  KX13X_WUFC,
  // 0x4E - 0x5C Reserved
  KX13X_SELF_TEST = 0x5D,
  KX13X_BUF_CNTL1,
  KX13X_BUF_CNTL2,
  KX13X_BUF_STATUS_1,
  KX13X_BUF_STATUS_2,
  KX13X_BUF_CLEAR,
  KX13X_BUF_READ,
  KX13X_ADP_CNTL1,
  KX13X_ADP_CNTL2,
  KX13X_ADP_CNTL3,
  KX13X_ADP_CNTL4,
  KX13X_ADP_CNTL5,
  KX13X_ADP_CNTL6,
  KX13X_ADP_CNTL7,
  KX13X_ADP_CNTL8,
  KX13X_ADP_CNTL9,
  KX13X_ADP_CNTL10,
  KX13X_ADP_CNTL11,
  KX13X_ADP_CNTL12,
  KX13X_ADP_CNTL13,
  KX13X_ADP_CNTL14,
  KX13X_ADP_CNTL15,
  KX13X_ADP_CNTL16,
  KX13X_ADP_CNTL17,
  KX13X_ADP_CNTL18,
  KX13X_ADP_CNTL19
  //Reserved 0x77 - 0x7F
};

#define  COTR_SUCCESS 0xAA //Succesfull COTR Register setting value
#define  PC1 7 //1:high performance/low power mode 0: stand by
#define  COTC 6 //1: Sets COTR to 0xAA to verify proper IC functionality 0: no action
#define  RES 6 //1:high performance mode 0: Low Power mode
#define  IEN1 5 //1:physical interrupt pin is enabled 0: disabled
#define  IEA1 4 //1:active HIGH 0: active LOW
#define  IEL1 3 //1:pulsed interrupt 0:latched interrupt until cleared by reading INT_REL
#define  STPOL 1 //1: positive self test polarity 0: negative
#define  SPI3E 0 //1: SPI Enabled 0: SPI Disabled

#define XLSB 0
#define XMSB 1
#define YLSB 2
#define YMSB 3
#define ZLSB 4
#define ZMSB 5

#define SPI_READ 0x01 // OR'ed at most sig BIT with register address
#define SPI_WRITE 0x00 // OR'ed at most sig BIT with register address

typedef enum {

  KX13X_SUCCESS = 0x00;
  KX13X_GENERAL_ERROR;
  KX13X_I2C_ERROR;

} KX13X_STATUS_t;

class QwiicKX13xCore
{
	public:

		bool beginCore(uint8_t, TwoWire &wirePort);
		bool beginSPICore(uint8_t, uint32_t, SPIClass &spiPort);
		uint8_t initialize();
		bool runCommandTest();

		bool waitForI2C();
		bool waitForSPI();

    outputData getAccelData();

		bool readBit(uint8_t, uint8_t);
		bool writeBit(uint8_t, uint8_t, bool);

		KX13X_STATUS_t readRegister(uint8_t*, uint8_t);
		KX13X_STATUS_t writeRegister(uint8_t, uint8_t);
		KX13X_STATUS_t readMultipleRegisters(uint8_t, uint8_t* , uint8_t);
    KX13X_STATUS_t overBufLenI2CRead(uint8_t, uint8_t* , uint8_t);

    // CPOL and CPHA are demonstrated on pg 25 of Specification Data sheet  
    // CPOL = 0, CPHA = 0 SPI_MODE0
    SPISettings kxSPISettings;

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

    bool begin(uint8_t kxAddress = KX13X_DEFAULT_ADDRESS, TwoWire &i2cPort = Wire);
    bool beginSPI(uint8_t, uint32_t spiPortSpeed = 10000000, SPIClass &spiPort = SPI);
//    bool convAccelData(outputData); 

class QwiicKX134 : public QwiicKX13xCore
{
  public: 

    bool begin(uint8_t kxAddress = KX13X_DEFAULT_ADDRESS, TwoWire &i2cPort = Wire);
    bool beginSPI(uint8_t, uint32_t spiPortSpeed = 10000000, SPIClass &spiPort = SPI);
    //bool convAccelData(outputData); 
};
    
#endif /* QWIIC_KX13X */
