// ESP32-S3 / Arduino IDE: 三輪オムニ（右下=π/3, 左下=2π/3, 正面=π）
void setup() {
  Serial.begin(115200); delay(200);
  omni_Init(); encoder_Init(); gyro_Init(); odometry_Init();
  omni_SetBodyVelocity(0.0f, 0.0f, 0.0f); // 起動直後は安全のため停止
  Serial.println("3-wheel omni controller ready");
}
void loop() {
  gyro_Update();
  if (encoder_Update()) { odometry_Update(); omni_UpdatePid(encoderDtS); }
  // 使用例: omni_SetBodyVelocity(0.30f, 0.00f, 0.00f); // 前進0.30 m/s
  // 使用例: omni_SetFieldVelocity(0.30f, 0.00f, 0.00f, robotPose.heading_rad);
}
