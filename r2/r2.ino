#include "robot_types.h"

constexpr uint32_t PRINT_INTERVAL_MS = 100;
constexpr uint32_t START_WAIT_MS = 3000;

void printRobotStatus() {
  static uint32_t previousPrintMs = 0;
  const uint32_t nowMs = millis();
  if (nowMs - previousPrintMs < PRINT_INTERVAL_MS) return;
  previousPrintMs = nowMs;

  Serial.print("pose,");
  Serial.print(robotPose.x_m, 3);
  Serial.print(',');
  Serial.print(robotPose.y_m, 3);
  Serial.print(',');
  Serial.print(robotPose.heading_rad, 3);
  Serial.print(",tracking,");
  Serial.print(trackingEncoderSpeedMps[0], 3);
  Serial.print(',');
  Serial.println(trackingEncoderSpeedMps[1], 3);
}

// ESP32-S3 / Arduino IDE: 三輪オムニ（右下=π/3, 左下=2π/3, 正面=π）
void setup() {
  Serial.begin(115200);
  delay(200);

  omni_Init();
  encoder_Init();
  gyro_Init();
  odometry_Init();

  // 実機で成功したR2e.inoと同じく、電源投入後3秒待ってから自動開始する。
  Serial.println("Automatic mission starts in 3 seconds");
  delay(START_WAIT_MS);
  mission_Init();

  Serial.println("3-wheel omni / 2-tracking-encoder controller ready");
}

void loop() {
  gyro_Update();

  // 100Hzで2測定輪オドメトリ、位置制御、駆動出力を順に更新する。
  if (encoder_Update()) {
    odometry_Update();
    mission_Update();
    omni_UpdateOutput(encoderDtS);
    printRobotStatus();
  }
}
