/*
  example2-Interrupts

  This example shows how to route the data ready bit to either physical interrupt pin one and pin two.

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
#include <SPI.h>
#include "SparkFun_KX13X.h"

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

	kxAccel.enableDataEngine();             //  Enables the bit that indicates data is ready.
	kxAccel.enablePhysInterrupt();          //  Enables interrupt pin 1
  kxAccel.routeHardwareInterrupt(0x10);   //  Routes the data ready bit to pin 1

	// Routing Data Ready pin to interrupt pin 2.
	//kxAccel.enablePhysInterrupt(true, 2);    //  Enables interrupt pin 2
	//kxAccel.routeHardwareInterrupt(0x10, 2); //  Routes the data ready bit to pin 2

  // This will change the interrupt behavior to latch instead of pulse
	// In this case you'll need to release directly with clearInterrupt();.
  //kxAccel.setLatchControl(); 

	//kxAccel.setPinMode();		 // Change interrupt to active HIGH
	//kxAccel.setPulseWidth(); // Change the length of a pulsed (non latched) interrupt
							
  kxAccel.setRange(0x18);         // 16g Range
	//kxAccel.setOutputDataRate();   // Default is 400Hz
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
    
	 // If interrupt behavior has been changed to latch, use the 
	 // following function to clear it after reading data.
	 //kxAccel.clearInterrupt();
  }

  delay(20); // Delay should be 1/ODR (Output Data Rate), default is 50Hz
}
