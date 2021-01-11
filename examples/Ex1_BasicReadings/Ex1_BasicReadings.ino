#include <SparkFun_Qwiic_KX13X.h>

QwiicKX13X accel;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println(Example 1 - Basic Accelerometer Readings);
  Wire.begin();
  if (accel.begin() == false)
  {
    Serial.println("Device did not ack, hanging");
    while(1);
  }
  accel.initialize();
}

void loop() {
  // put your main code here, to run repeatedly:

}
