#include <Wire.h>
#include "SparkFun_Qwiic_KX13X.h"

QwiicKX132 accel;
outputData myData;

void setup() {

  while(!Serial){delay(50);}
  Serial.begin(115200);
  Serial.println("Welcome.");

  Wire.begin();
  if( !accel.begin() )
    Serial.println("Could not communicate with the the KX13X.");
  else
    Serial.println("Ready.");
    


  if( !accel.initialize(DEFAULT_SETTINGS)){
    Serial.println("Could not initialize the chip.");
    while(1);
  }
  else
    Serial.println("Initialized...");

  if( !accel.setRange(3) ) {
    Serial.println("Could not set range.");
    while(1);
  }

}

void loop() {

  myData = accel.getAccelData();
  Serial.print("X: ");
  Serial.print(myData.xData, 4);
  Serial.print("g ");
  Serial.print(" Y: ");
  Serial.print(myData.zData, 4);
  Serial.print("g ");
  Serial.print(" Z: ");
  Serial.print(myData.zData, 4);
  Serial.println("g ");

  delay(20);

}
