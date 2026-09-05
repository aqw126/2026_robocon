#include "robot_types.h"

// ジャイロ無効版。
// BNO055とは通信せず、imuHeadingRadにはミッションが管理する想定角度だけを保持する。
float imuHeadingRad = 0.0f;
bool bnoAvailable = false;

void gyro_Init() {
  bnoAvailable = false;
  imuHeadingRad = 0.0f;
  Serial.println("BNO055 disabled; timed turns are used");
}

void gyro_Update() {}

void gyro_ResetHeading(float heading_rad) {
  while (heading_rad > kPi) heading_rad -= 2.0f * kPi;
  while (heading_rad <= -kPi) heading_rad += 2.0f * kPi;
  imuHeadingRad = heading_rad;
}
