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

bool gyro_ReadRawHeading(float &headingRad) {
  if (!bnoAvailable) return false;
  sensors_event_t event;
  if (!bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER)) return false;
  if (isnan(event.orientation.x) || isinf(event.orientation.x)) return false;
  headingRad = BNO_YAW_SIGN * radians(event.orientation.x) + BNO_YAW_OFFSET_RAD;
  return true;
}

void gyro_Init() {
  Wire.begin(BNO_SDA_PIN, BNO_SCL_PIN);
  Wire.setClock(100000);
  // アーム始動時にBNO055の電源が乱れても、I2C待ちで制御全体を止めない。
  Wire.setTimeOut(20);
  bnoAvailable = bno.begin();
  if (!bnoAvailable) { Serial.println("BNO055 が見つかりません。方位は0固定です。"); return; }
  bno.setExtCrystalUse(true); delay(50);
  float rawHeadingRad;
  if (!gyro_ReadRawHeading(rawHeadingRad)) {
    Serial.println("BNO055の初期方位を読み取れません。");
    bnoAvailable = false;
    return;
  }
  gyroReferenceRad = rawHeadingRad;
  gyroHeadingAtReferenceRad = 0.0f;
  imuHeadingRad = 0.0f;
}
void gyro_Update() {
  if (!bnoAvailable) return;
  float rawHeadingRad;
  if (!gyro_ReadRawHeading(rawHeadingRad)) return;
  imuHeadingRad = gyro_NormalizeAngle(
    rawHeadingRad - gyroReferenceRad + gyroHeadingAtReferenceRad
  );
}
void gyro_ResetHeading(float heading_rad) {
  gyroHeadingAtReferenceRad = gyro_NormalizeAngle(heading_rad);
  imuHeadingRad = gyroHeadingAtReferenceRad;
  if (!bnoAvailable) return;
  float rawHeadingRad;
  if (gyro_ReadRawHeading(rawHeadingRad)) gyroReferenceRad = rawHeadingRad;
}
