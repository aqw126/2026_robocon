#include "robot_types.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// 実機確認結果: 生のHeadingは反時計回りで増加し、時計回りで減少する。
// コード内部も反時計回りを正とするため符号はそのまま使用する。
constexpr int BNO_YAW_SIGN = 1;
constexpr float BNO_YAW_OFFSET_RAD = 0.0f;
// ta1.inoでBNO055との通信を確認できた実配線。
constexpr int BNO_SDA_PIN = 38, BNO_SCL_PIN = 48;
Adafruit_BNO055 bno(55, 0x28);
float imuHeadingRad = 0.0f;
bool bnoAvailable = false;
static float gyroReferenceRad = 0.0f;
static float gyroHeadingAtReferenceRad = 0.0f;

float gyro_NormalizeAngle(float angle) { while (angle > kPi) angle -= 2.0f * kPi; while (angle <= -kPi) angle += 2.0f * kPi; return angle; }
void gyro_Init() {
  Wire.begin(BNO_SDA_PIN, BNO_SCL_PIN); Wire.setClock(100000); bnoAvailable = bno.begin();
  if (!bnoAvailable) { Serial.println("BNO055 が見つかりません。方位は0固定です。"); return; }
  bno.setExtCrystalUse(true); delay(50);
  const imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyroReferenceRad = BNO_YAW_SIGN * radians(euler.x()) + BNO_YAW_OFFSET_RAD;
  gyroHeadingAtReferenceRad = 0.0f;
  imuHeadingRad = 0.0f;
}
void gyro_Update() {
  if (!bnoAvailable) return;
  const imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imuHeadingRad = gyro_NormalizeAngle(
    BNO_YAW_SIGN * radians(euler.x()) + BNO_YAW_OFFSET_RAD
    - gyroReferenceRad + gyroHeadingAtReferenceRad
  );
}
void gyro_ResetHeading(float heading_rad) {
  gyroHeadingAtReferenceRad = gyro_NormalizeAngle(heading_rad);
  imuHeadingRad = gyroHeadingAtReferenceRad;
  if (!bnoAvailable) return;
  const imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyroReferenceRad = BNO_YAW_SIGN * radians(euler.x()) + BNO_YAW_OFFSET_RAD;
}
