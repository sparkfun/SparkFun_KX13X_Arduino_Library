#include <SparkFun_Qwiic_KX13X.h>

QwiicKX13X accel;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Example 1 - Basic Accelerometer Readings");
  Wire.begin();
  if (accel.begin() == false)
  {
    Serial.println("Device did not ack, hanging");
    while (1);
  }
  accel.initialize();
}

void loop() {
  accel.getReadings();
  uint16_t xVal = accel.getAccelX();
  uint16_t yVal = accel.getAccelY();
  uint16_t zVal = accel.getAccelZ();
  Serial.println(xVal);
}
