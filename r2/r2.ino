#include "robot_types.h"

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
  tof_Update();

  // 100Hzでオドメトリ、状態遷移、車輪PIDを順に更新する。
  if (encoder_Update()) {
    odometry_Update();
    mission_Update();
    omni_UpdatePid(encoderDtS);
  }
}
