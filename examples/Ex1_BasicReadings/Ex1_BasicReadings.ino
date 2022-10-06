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

SparkFun_KX132 kxAccel; 

outputData myData; // This will hold the accelerometer's output. 

void setup() {

	Wire.begin();
	Serial.begin(115200);
  Serial.println("Welcome.");

  while(!Serial)
    delay(50);


  if( !kxAccel.begin() )
	{
    Serial.println("Could not communicate with the the KX13X. Freezing.");
    while(1);
	}

	Serial.println("Ready.");
    
  kxAccel.setRange(KX134_RANGE32G); // For a larger range uncomment

}

void loop() {

  kxAccel.getAccelData(&myData); 
  Serial.print("X: ");
  Serial.print(myData.xData, 4);
  Serial.print("g ");
  Serial.print(" Y: ");
  Serial.print(myData.yData, 4);
  Serial.print("g ");
  Serial.print(" Z: ");
  Serial.print(myData.zData, 4);
  Serial.println("g ");

  delay(20); // Delay should be 1/ODR (Output Data Rate), default is 50Hz

}
