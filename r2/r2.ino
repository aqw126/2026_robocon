#include "robot_types.h"

constexpr uint32_t PRINT_INTERVAL_MS = 100;

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
  Serial.print(",wheel,");
  Serial.print(wheelSpeedMps[0], 3);
  Serial.print(',');
  Serial.print(wheelSpeedMps[1], 3);
  Serial.print(',');
  Serial.println(wheelSpeedMps[2], 3);
}

// ESP32-S3 / Arduino IDE: 三輪オムニ（右下=π/3, 左下=2π/3, 正面=π）
void setup() {
  Serial.begin(115200);
  delay(200);

  omni_Init();
  encoder_Init();
  gyro_Init();
  odometry_Init();
  mission_Init();

  Serial.println("3-wheel omni controller ready");
}

void loop() {
  gyro_Update();

  // 100Hzでオドメトリ、状態遷移、車輪PIDを順に更新する。
  if (encoder_Update()) {
    odometry_Update();
    mission_Update();
    omni_UpdatePid(encoderDtS);
    printRobotStatus();
  }
}
