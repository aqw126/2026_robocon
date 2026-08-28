#include "robot_types.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

constexpr int BNO_YAW_SIGN = 1;  // CCWでheadingが増えなければ-1
constexpr float BNO_YAW_OFFSET_RAD = 0.0f;
constexpr int BNO_SDA_PIN = 21, BNO_SCL_PIN = 22;
Adafruit_BNO055 bno(55, 0x28);
float imuHeadingRad = 0.0f;
bool bnoAvailable = false;
static float gyroReferenceRad = 0.0f;

float gyro_NormalizeAngle(float angle) { while (angle > kPi) angle -= 2.0f * kPi; while (angle <= -kPi) angle += 2.0f * kPi; return angle; }
void gyro_Init() {
  Wire.begin(BNO_SDA_PIN, BNO_SCL_PIN); Wire.setClock(100000); bnoAvailable = bno.begin();
  if (!bnoAvailable) { Serial.println("BNO055 が見つかりません。方位は0固定です。"); return; }
  bno.setExtCrystalUse(true); delay(50);
  const imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyroReferenceRad = BNO_YAW_SIGN * radians(euler.x()) + BNO_YAW_OFFSET_RAD;
}
void gyro_Update() {
  if (!bnoAvailable) return;
  const imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imuHeadingRad = gyro_NormalizeAngle(BNO_YAW_SIGN * radians(euler.x()) + BNO_YAW_OFFSET_RAD - gyroReferenceRad);
}
void gyro_ResetHeading() {
  if (!bnoAvailable) return;
  const imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyroReferenceRad = BNO_YAW_SIGN * radians(euler.x()) + BNO_YAW_OFFSET_RAD; imuHeadingRad = 0.0f;
}
