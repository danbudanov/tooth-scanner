#include <Wire.h>
#include "Accelerometer_ADXL345/ADXL345.h"

ADXL345 adxl;
void setup() {
    Serial.begin(9600);
    adxl.powerOn();
}

void loop() {
  // put your main code here, to run repeatedly:
    int x, y, z;
    adxl.readXYZ(&x, &y, &z);
    Serial.print(x);
    Serial.print(y);
    Serial.print(z);

    double xyz[3];

}
