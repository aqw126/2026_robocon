#include "robot_types.h"

constexpr uint32_t PRINT_INTERVAL_MS = 100;
constexpr uint32_t CONTROL_INTERVAL_US = 10000;
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
  Serial.print(",command,");
  Serial.print(targetBodyVelocity.vx, 3);
  Serial.print(',');
  Serial.print(targetBodyVelocity.vy, 3);
  Serial.print(',');
  Serial.println(targetBodyVelocity.w, 3);
}

// ESP32-S3 / Arduino IDE: 三輪オムニ（右下=π/3, 左下=2π/3, 正面=π）
void setup() {
  Serial.begin(115200);
  delay(200);

  omni_Init();
  gyro_Init();
  odometry_Init();

  // 実機で成功したR2e.inoと同じく、電源投入後3秒待ってから自動開始する。
  Serial.println("Automatic mission starts in 3 seconds");
  delay(START_WAIT_MS);
  mission_Init();

  Serial.println("WARNING: no encoder; X/Y are PWM-model estimates only");
  Serial.println("3-wheel omni / BNO055-only controller ready");
}

void loop() {
  gyro_Update();

  static uint32_t previousControlUs = 0;
  const uint32_t nowUs = micros();
  if (previousControlUs != 0 && nowUs - previousControlUs < CONTROL_INTERVAL_US) return;

  const float dtS = previousControlUs == 0
    ? CONTROL_INTERVAL_US * 1.0e-6f
    : (nowUs - previousControlUs) * 1.0e-6f;
  previousControlUs = nowUs;

  // 並進位置は前周期に実際に出したPWMからの推定で、実測値ではない。
  odometry_Update(dtS);
  mission_Update();
  omni_UpdateOutput(dtS);
  printRobotStatus();
}
