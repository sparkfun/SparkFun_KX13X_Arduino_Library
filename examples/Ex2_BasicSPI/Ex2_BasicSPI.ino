#include <SparkFun_Qwiic_KX13X.h>
#include <SPI.h>
QwiicKX13X accel;
#define CS_PIN 10
#define SPI_SPEED 2000000 //Max is 10 MHz

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Example 2 - Basic Accelerometer Readings over SPI");
  //Wire.begin();
  /*if (accel.begin() == false)
  {
    Serial.println("Device did not ack, hanging");
    while (1);
  }*/
  accel.beginSPI(CS_PIN, SPI_SPEED);
  Serial.println(accel.initialize());
  /*if (accel.initialize() == true)
  {
    Serial.println("SPI Success");
  }
  else
  {
    Serial.println("SPI fail, check wiring");
  }*/
}

void loop() {
  accel.getReadings();
  int16_t xVal = accel.getAccelX();
  int16_t yVal = accel.getAccelY();
  int16_t zVal = accel.getAccelZ();
  Serial.print(xVal);
  Serial.print(", ");
  Serial.print(yVal);
  Serial.print(", ");
  Serial.println(zVal);
}