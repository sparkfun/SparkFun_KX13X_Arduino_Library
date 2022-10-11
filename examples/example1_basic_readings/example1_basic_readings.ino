/*
  example1-BasicReadings

  This example shows the basic settings and functions for retrieving accelerometer
	data.
	Please refer to the header file for more possible settings, found here:
	..\SparkFun_KX13X_Arduino_Library\src\sfe_kx13x_defs.h

  Written by Elias Santistevan @ SparkFun Electronics, October 2022

	Product:

		https://www.sparkfun.com/products/17871

  Repository:

		https://github.com/sparkfun/SparkFun_KX13X_Arduino_Library

  SparkFun code, firmware, and software is released under the MIT 
	License	(http://opensource.org/licenses/MIT).
*/

#include <Wire.h>
#include "SparkFun_KX13X.h"

SparkFun_KX132 kxAccel; 

outputData myData;   // Struct for the accelerometer's data

void setup() 
{
	
	Wire.begin();

	Serial.begin(115200);
  Serial.println("Welcome.");

	// Wait for the Serial monitor to be opened. 
  while(!Serial)
    delay(50);


  if( !kxAccel.begin(chipSelect) )
	{
    Serial.println("Could not communicate with the the KX13X. Freezing.");
    while(1);
	}

	Serial.println("Ready.");

	// Many settings for KX13X can only be                    														
	// applied when the accelerometer is powered down.                  														
	// However there are many that can be changed "on-the-fly"
	// check datasheet for more info, or the comments in the
	// "...regs.h" file which specify which can be changed when.
	kxAccel.enableAccel(false); 

  kxAccel.setRange(0x18);         // 16g Range
	kxAccel.enableDataEngine();     // Enables the bit that indicates data is ready.
	// kxAccel.setOutputDataRate(); // Default is 400Hz
	kxAccel.enableAccel();          


}

void loop() 
{
	// Check if data is ready.
	if( kxAccel.dataReady() )
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
  delay(20); // Delay should be 1/ODR (Output Data Rate), default is 1/50ODR

}
