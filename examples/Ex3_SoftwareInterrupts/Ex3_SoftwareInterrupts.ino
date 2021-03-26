#include <Wire.h>
#include "SparkFun_Qwiic_KX13X.h"

QwiicKX132 kxAccel;
outputData myData; // This will hold the accelerometer's output. 

void setup() {

  while(!Serial){
    delay(50);
  }

  Serial.begin(115200);
  Serial.println("Welcome.");

  Wire.begin();
  if( !kxAccel.begin() ){
    Serial.println("Could not communicate with the the KX13X. Freezing.");
    while(1);
  }
  else
    Serial.println("Ready.");
    


  if( !kxAccel.initialize(SOFT_INT_SETTINGS)){
    Serial.println("Could not initialize the chip. Freezing.");
    while(1);
  }
  else
    Serial.println("Initialized...");

  // kxAccel.setRange(KX132_RANGE16G);

}

void loop() {
  

  if( kxAccel.dataTrigger() ) {
    Serial.println("Data Ready.");
    // Interrupt is cleared on reading registers.
    myData = kxAccel.getAccelData();
    Serial.print("X: ");
    Serial.print(myData.xData, 4);
    Serial.print("g ");
    Serial.print(" Y: ");
    Serial.print(myData.zData, 4);
    Serial.print("g ");
    Serial.print(" Z: ");
    Serial.print(myData.zData, 4);
    Serial.println("g ");
  }
  
    
  
  delay(20); // Delay should be 1/ODR (Output Data Rate), default is 50Hz
}
