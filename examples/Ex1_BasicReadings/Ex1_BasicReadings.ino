/*********************************************************
Author: Elias Santistevan @ SparkFun Electronics
Date: 5/2021
Library repository can be found here:
https://github.com/sparkfun/SparkFun_KX13X_Arduino_Library

Basic example for reading back accelerometer values using I2C. 

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.
*******************************************************/

#include <Wire.h>
#include "SparkFun_KX13X.h"

SparkFun_KX132_SPI kxAccel; 

outputData myData; // This will hold the accelerometer's output. 
byte chipSelect = 1;

void setup() {
	
	pinMode(chipSelect, OUTPUT);
	digitalWrite(chipSelect, HIGH);

	SPI.begin();
	Serial.begin(115200);
  Serial.println("Welcome.");

//  while(!Serial)
//    delay(50);
//

  if( !kxAccel.begin(chipSelect) )
	{
    Serial.println("Could not communicate with the the KX13X. Freezing.");
    while(1);
	}

	kxAccel.enableAccel(false); // The following settings will only be applied when the
															// KX13X is off. 
	
	Serial.println("Ready.");
    
  kxAccel.setRange(0x18); // 16g Range
	kxAccel.enableDataEngine(); // Enables the bit that indicates data is ready.
	// kxAccel.setOutputDataRate(); //Keeping default of 400Hz
	kxAccel.enableAccel();


}

void loop() {


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
