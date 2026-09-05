#include "robot_types.h"

// 2個の測定輪の移動量から、BNO055で得た回転成分を除去して並進量を求める。
Pose2D robotPose;
BodyTwist robotBodyVelocity = {0.0f, 0.0f, 0.0f};
static float previousOdometryHeadingRad = 0.0f;

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

void odometry_Update() {
  const float dTheta = odometry_NormalizeAngle(imuHeadingRad - previousOdometryHeadingRad);

  float directionX[config::tracking_encoder_count];
  float directionY[config::tracking_encoder_count];
  float translationDistance[config::tracking_encoder_count];
  for (uint8_t i = 0; i < config::tracking_encoder_count; ++i) {
    directionX[i] = cosf(config::tracking_direction_rad[i]);
    directionY[i] = sinf(config::tracking_direction_rad[i]);

    // CCW回転時の測定輪接地点変位(-y*dTheta, x*dTheta)を測定方向へ射影する。
    const float rotationLeverM =
      -directionX[i] * config::tracking_position_y_m[i]
      + directionY[i] * config::tracking_position_x_m[i];
    translationDistance[i] =
      trackingEncoderDeltaDistanceM[i] - rotationLeverM * dTheta;
  }

  const float determinant =
    directionX[0] * directionY[1] - directionY[0] * directionX[1];
  if (fabsf(determinant) < 1.0e-6f) return;

  const float dxBody =
    (directionY[1] * translationDistance[0]
     - directionY[0] * translationDistance[1]) / determinant;
  const float dyBody =
    (-directionX[1] * translationDistance[0]
     + directionX[0] * translationDistance[1]) / determinant;

  const float midHeading = previousOdometryHeadingRad + 0.5f * dTheta;
  const float c = cosf(midHeading);
  const float s = sinf(midHeading);
  robotPose.x_m += c * dxBody - s * dyBody;
  robotPose.y_m += s * dxBody + c * dyBody;
  robotPose.heading_rad = imuHeadingRad;

  if (encoderDtS > 0.0f) {
    robotBodyVelocity.vx = dxBody / encoderDtS;
    robotBodyVelocity.vy = dyBody / encoderDtS;
    robotBodyVelocity.w = dTheta / encoderDtS;
  }
  previousOdometryHeadingRad = imuHeadingRad;
}
