#include "robot_types.h"

// エンコーダーなし版。
// 姿勢はBNO055で測定するが、X/Yは指令速度の時間積分による推定値にすぎない。
// 車輪の滑り、モーター停止、壁への接触は検出できない。
Pose2D robotPose;
BodyTwist robotBodyVelocity = {0.0f, 0.0f, 0.0f};
static float previousOdometryHeadingRad = 0.0f;

// 指令値と実速度の差を実走試験で補正するための暫定倍率。
constexpr float COMMAND_TRANSLATION_SCALE = 1.0f;

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

  const float dxBody = targetBodyVelocity.vx * dt_s * COMMAND_TRANSLATION_SCALE;
  const float dyBody = targetBodyVelocity.vy * dt_s * COMMAND_TRANSLATION_SCALE;
  const float c = cosf(midHeading);
  const float s = sinf(midHeading);
  robotPose.x_m += c * dxBody - s * dyBody;
  robotPose.y_m += s * dxBody + c * dyBody;
  robotPose.heading_rad = imuHeadingRad;

  robotBodyVelocity.vx = targetBodyVelocity.vx * COMMAND_TRANSLATION_SCALE;
  robotBodyVelocity.vy = targetBodyVelocity.vy * COMMAND_TRANSLATION_SCALE;
  robotBodyVelocity.w = dTheta / dt_s;
  previousOdometryHeadingRad = imuHeadingRad;
}
