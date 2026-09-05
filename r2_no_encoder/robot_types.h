#pragma once

#include <Arduino.h>
#include <math.h>

// 座標系: x=前方, y=左方, 角速度w=反時計回り。単位: m, s, rad
// 車輪順: 0=左前、1=後ろ、2=右前
constexpr float kPi = 3.14159265358979323846f;

namespace config {
constexpr uint8_t drive_wheel_count = 3;
constexpr uint8_t wheel_count = drive_wheel_count;

// 駆動輪は機体中心から177.5mm。
constexpr float mount_radius_m = 0.1775f;
// 機体前方を0rad、左方向を+π/2、反時計回りを正とする。
// 車輪順: 0=左前、1=後ろ、2=右前
constexpr float mount_offset[wheel_count] = {
   kPi / 3.0f,   // 左前: +60°
   kPi,          // 後ろ: 180°（車輪の転がり方向は真横）
  -kPi / 3.0f    // 右前: -60°
};

constexpr float max_wheel_speed_mps = 1.0f;
}

struct BodyTwist { float vx, vy, w; };
struct WheelSpeeds { float v_wheel[config::wheel_count] = {}; };
struct Pose2D { float x_m = 0.0f, y_m = 0.0f, heading_rad = 0.0f; };

// タブ間で共有する状態
extern float imuHeadingRad;
extern Pose2D robotPose;
extern BodyTwist robotBodyVelocity;
extern BodyTwist targetBodyVelocity;

// タブ間で呼び出す関数
void omni_Init();
void omni_SetBodyVelocity(float vx_mps, float vy_mps, float w_radps);
void omni_SetFieldVelocity(float vx_mps, float vy_mps, float w_radps, float heading_rad);
void omni_UpdateOutput(float dt_s);
bool omni_Forward(const WheelSpeeds &wheel, BodyTwist &body);

void gyro_Init();
void gyro_Update();
void gyro_ResetHeading(float heading_rad);
void odometry_Init();
void odometry_Update(float dt_s);
void odometry_Reset(float x_m, float y_m);
void mission_Init();
void mission_Update();
