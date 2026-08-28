#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void gyro_Init() {
  while (!Serial) delay(10);

  Wire.begin(21, 22);    // ESP32 (SDA: 21, SCL: 22)
  Wire.setClock(100000);

  if (!bno.begin()) {
    Serial.println("エラー");
    while (1);
}

void gyro(){
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  Serial.print(euler.x()); // x
  Serial.print(",");
  Serial.print(euler.y()); // y
  Serial.print(",");
  Serial.println(euler.z()); // z

  delay(20);
}