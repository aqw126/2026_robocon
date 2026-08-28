// エンコーダーの並進量を、BNO055の方位でフィールド座標へ積分する。
Pose2D robotPose;
BodyTwist robotBodyVelocity = {0.0f, 0.0f, 0.0f};
void odometry_Init() { robotPose = Pose2D(); }
void odometry_Reset(float x_m = 0.0f, float y_m = 0.0f) { robotPose.x_m = x_m; robotPose.y_m = y_m; robotPose.heading_rad = imuHeadingRad; }
void odometry_Update() {
  WheelSpeeds wheelDelta, wheelVelocity;
  for (uint8_t i = 0; i < config::wheel_count; ++i) { wheelDelta.v_wheel[i] = wheelDeltaDistanceM[i]; wheelVelocity.v_wheel[i] = wheelSpeedMps[i]; }
  BodyTwist bodyDelta; if (!omni_Forward(wheelDelta, bodyDelta)) return;
  omni_Forward(wheelVelocity, robotBodyVelocity);
  const float c = cosf(imuHeadingRad), s = sinf(imuHeadingRad);
  robotPose.x_m += c * bodyDelta.vx - s * bodyDelta.vy;
  robotPose.y_m += s * bodyDelta.vx + c * bodyDelta.vy;
  robotPose.heading_rad = imuHeadingRad;
}
