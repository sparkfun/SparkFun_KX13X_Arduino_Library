#include <Wire.h>
#include "SparkFun_Qwiic_KX13X.h"

QwiicKX132 kxAccel;
outputData myData; // This will hold the accelerometer's output. 
int dataReadyPin = D1;
int timeThen;
int timeNow; 

void setup() {

  pinMode(dataReadyPin, OUTPUT);
  digitalWrite(dataReadyPin, HIGH);

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
    Serial.println("Could not initialize the chip.");
    while(1);
  }
  else
    Serial.println("Initialized...");

  // kxAccel.setRange(KX132_RANGE16G);
  timeThen = millis();

}

void loop() {

  timeNow = millis();

  if(timeNow - timeThen > 1000){
    timeThen = timeNow;
    digitalWrite(dataReadyPin, LOW);
    Serial.println("click.");
  }

  myData = kxAccel.getAccelData();
  //digitalWrite(dataReadyPin, HIGH);
  //Serial.print("X: ");
  //Serial.print(myData.xData, 4);
  //Serial.print("g ");
  //Serial.print(" Y: ");
  //Serial.print(myData.zData, 4);
  //Serial.print("g ");
  //Serial.print(" Z: ");
  //Serial.print(myData.zData, 4);
  //Serial.println("g ");
  
  // kxAccel.clearInterrupt();

  delay(20); // Delay should be 1/ODR (Output Data Rate), default is 50Hz
}
