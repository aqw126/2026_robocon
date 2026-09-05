#include "robot_types.h"

// エンコーダーなし版。
// 姿勢はBNO055で測定するが、X/Yは実出力PWMから推定した速度の積分値にすぎない。
// 車輪の滑り、モーター停止、壁への接触は検出できない。
Pose2D robotPose;
BodyTwist robotBodyVelocity = {0.0f, 0.0f, 0.0f};
static float previousOdometryHeadingRad = 0.0f;

// PWMから計算した理論速度に対する、実速度の暫定倍率。
// 倉庫Cへ着く前に推定位置が到着したため、理論値より遅い0.70から始める。
// まだ手前で動作が切り替わる場合は小さく、通り過ぎる場合は大きくする。
constexpr float DRIVE_SPEED_ESTIMATE_SCALE = 0.70f;

float odometry_NormalizeAngle(float angle) {
  while (angle > kPi) angle -= 2.0f * kPi;
  while (angle <= -kPi) angle += 2.0f * kPi;
  return angle;
}

void odometry_Init() {
  robotPose = Pose2D();
  previousOdometryHeadingRad = imuHeadingRad;
}

void odometry_Reset(float x_m, float y_m) {
  robotPose.x_m = x_m;
  robotPose.y_m = y_m;
  robotPose.heading_rad = imuHeadingRad;
  previousOdometryHeadingRad = imuHeadingRad;
  robotBodyVelocity = {0.0f, 0.0f, 0.0f};
}

void odometry_Update(float dt_s) {
  if (dt_s <= 0.0f) return;

  const float dTheta = odometry_NormalizeAngle(
    imuHeadingRad - previousOdometryHeadingRad
  );
  const float midHeading = previousOdometryHeadingRad + 0.5f * dTheta;

  // 目標速度ではなく、加速制限・PWM上限を通過した実際の出力値から速度を推定する。
  WheelSpeeds estimatedWheelSpeed;
  for (uint8_t i = 0; i < config::wheel_count; ++i) {
    estimatedWheelSpeed.v_wheel[i] =
      appliedMotorPwm[i] / config::drive_pwm_per_wheel_mps
      * DRIVE_SPEED_ESTIMATE_SCALE;
  }

  BodyTwist estimatedBodyVelocity;
  if (!omni_Forward(estimatedWheelSpeed, estimatedBodyVelocity)) return;

  const float dxBody = estimatedBodyVelocity.vx * dt_s;
  const float dyBody = estimatedBodyVelocity.vy * dt_s;
  const float c = cosf(midHeading);
  const float s = sinf(midHeading);
  robotPose.x_m += c * dxBody - s * dyBody;
  robotPose.y_m += s * dxBody + c * dyBody;
  robotPose.heading_rad = imuHeadingRad;

  robotBodyVelocity.vx = estimatedBodyVelocity.vx;
  robotBodyVelocity.vy = estimatedBodyVelocity.vy;
  robotBodyVelocity.w = dTheta / dt_s;
  previousOdometryHeadingRad = imuHeadingRad;
}
