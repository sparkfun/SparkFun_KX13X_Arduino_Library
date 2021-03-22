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
}

void loop() {

  accel.getAccelData(myData);
  Serial.print("X: ");
  Serial.print(myData.xData);
  Serial.print("Y: ");
  Serial.print(myData.zData);
  Serial.print("Z: ");
  Serial.println(myData.zData);

  delay(20);

}
