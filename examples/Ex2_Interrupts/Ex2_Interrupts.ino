#include <Wire.h>
#include "SparkFun_Qwiic_KX13X.h"

QwiicKX132 kxAccel;
outputData myData; // This will hold the accelerometer's output. 
int dataReadyPin = D1;

void setup() {

  pinMode(dataReadyPin, INPUT);

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
    


  if( !kxAccel.initialize(INT_SETTINGS)){
    Serial.println("Could not initialize the chip. Freezing.");
    while(1);
  }
  else
    Serial.println("Initialized...");

  // kxAccel.setRange(KX132_RANGE16G);

}

void loop() {

  if( digitalRead(dataReadyPin) == HIGH ){ // Wait for new data to be ready.

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
    
     //kxAccel.clearInterrupt();// Because the data is being read in "burst"
     //mode, meaning that all the acceleration data is being read at once, we don't
     //need to clear the interrupt.
  }
  delay(20); // Delay should be 1/ODR (Output Data Rate), default is 50Hz
}
