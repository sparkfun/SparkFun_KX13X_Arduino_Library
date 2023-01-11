/*
  example3-Buffer 

  This example shows both how to setup the buffer but also how to route the buffer's 
	interrupt to a physical interrupt pin. 

  Written by Elias Santistevan @ SparkFun Electronics, October 2022

	Products:

	SparkFun Triple Axis Accelerometer Breakout - KX132: 
		https://www.sparkfun.com/products/17871

	SparkFun Triple Axis Accelerometer Breakout - KX134: 
		https://www.sparkfun.com/products/17589


  Repository:

		https://github.com/sparkfun/SparkFun_KX13X_Arduino_Library

  SparkFun code, firmware, and software is released under the MIT 
	License	(http://opensource.org/licenses/MIT).
*/

#include <Wire.h>
#include "SparkFun_Qwiic_KX13X.h"

SparkFun_KX132 kxAccel; 
// SparkFun_KX134 kxAccel; // For the KX134, uncomment this and comment line above

outputData myData;     //  Struct for the accelerometer's data
byte dataReadyPin = 2; //  Change to fit your project.

void setup() 
{
	
	Wire.begin();

	Serial.begin(115200);
  Serial.println("Welcome.");

	// Wait for the Serial monitor to be opened. 
  while(!Serial)
    delay(50);


  if( !kxAccel.begin() )
	{
    Serial.println("Could not communicate with the the KX13X. Freezing.");
    while(1);
	}

	Serial.println("Ready.");

	// Reset the chip so that old settings don't apply to new setups.
	if( kxAccel.softwareReset() )
		Serial.println("Reset.");

	// Many settings for KX13X can only be                    														
	// applied when the accelerometer is powered down.                  														
	// However there are many that can be changed "on-the-fly"
	// check datasheet for more info, or the comments in the
	// "...regs.h" file which specify which can be changed when.
	kxAccel.enableAccel(false); 

	kxAccel.enableBufferInt();							//  Enables the Buffer interrupt
	kxAccel.enablePhysInterrupt();          //  Enables interrupt pin 1
  kxAccel.routeHardwareInterrupt(0x40);   //  Routes the data ready bit to pin 1

  kxAccel.enableSampleBuffer();				  // Enable buffer.
  kxAccel.setBufferOperationMode(0x00); // Enable the buffer to be FIFO.

	// Additional Buffer Settings

	//uint8_t numSamples = 140;
	//kxAccel.setBufferThreshold(numSamples); //  Set the number of sample that can be stored in the buffer

	//kxAccel.setBufferResolution();          //  Change sample resolution to 16 bit, 8 bit by default.
																						//  This will change how many samples can be held in buffer.

	//kxAccel.clearBuffer();                   //  Clear the buffer
	//kxAccel.getSampleLevel();                //  Get the number of samples in the buffer. This number
																						 //  Changes depending on the resolution, see datasheet for more info.

  kxAccel.setRange(SFE_KX132_RANGE16G);         // 16g Range
  //kxAccel.setRange(SFE_KX134_RANGE16G);        // 16g for the KX134

	//kxAccel.setOutputDataRate(); //  Default is 50Hz
	kxAccel.enableAccel();          


}

void loop() 
{

  if( digitalRead(dataReadyPin) == HIGH ) // Check for data ready pin
	{ 
    kxAccel.getAccelData(&myData);
    Serial.print("X: ");
    Serial.print(myData.xData, 4);
    Serial.print(" Y: ");
    Serial.print(myData.yData, 4);
    Serial.print(" Z: ");
    Serial.print(myData.zData, 4);
		Serial.println();
    
  }

  delay(20); // Delay should be 1/ODR (Output Data Rate), default is 50Hz
}
