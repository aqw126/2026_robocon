#pragma once

#include <Arduino.h>
#include <math.h>

// 座標系: x=前方, y=左方, 角速度w=反時計回り。単位: m, s, rad
// 車輪順: 0=右下(π/3), 1=左下(2π/3), 2=正面(π)
constexpr float kPi = 3.14159265358979323846f;

namespace config {
constexpr uint8_t wheel_count = 3;
constexpr float wheel_radius_m = 0.050f;       // 実測値に変更
constexpr float mount_radius_m = 0.150f;       // 機体中心から車輪まで
// 機体前方を0rad、左方向を+π/2、反時計回りを正とする。
// 車輪順: 0=M1右後、1=M2左後、2=M3正面
constexpr float mount_offset[wheel_count] = {
  -2.0f * kPi / 3.0f,  // M1 右後: -120°
   2.0f * kPi / 3.0f,  // M2 左後: +120°
   0.0f                 // M3 正面:    0°
};
constexpr float max_wheel_speed_mps = 1.0f;
}

struct BodyTwist { float vx, vy, w; };
struct WheelSpeeds { float v_wheel[config::wheel_count] = {}; };
struct Pose2D { float x_m = 0.0f, y_m = 0.0f, heading_rad = 0.0f; };

// タブ間で共有する状態
extern float wheelSpeedMps[config::wheel_count];
extern float wheelDeltaDistanceM[config::wheel_count];
extern float encoderDtS;
extern float imuHeadingRad;
extern Pose2D robotPose;
extern BodyTwist robotBodyVelocity;

// タブ間で呼び出す関数
void omni_Init();
void omni_SetBodyVelocity(float vx_mps, float vy_mps, float w_radps);
void omni_SetFieldVelocity(float vx_mps, float vy_mps, float w_radps, float heading_rad);
void omni_UpdatePid(float dt_s);
bool omni_Forward(const WheelSpeeds &wheel, BodyTwist &body);

void encoder_Init();
bool encoder_Update();
void gyro_Init();
void gyro_Update();
void gyro_ResetHeading();
void odometry_Init();
void odometry_Update();
void odometry_Reset(float x_m, float y_m);
void mission_Init();
void mission_Update();
