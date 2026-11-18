#pragma once

#include <Arduino.h>
#include <math.h>

// 座標系: x=前方, y=左方, 角速度w=反時計回り。単位: m, s, rad
// 車輪順: 0=左前、1=後ろ、2=右前
// r2/robot_types.h からBluetooth操作に必要な部分だけを抜き出したもの。
constexpr float kPi = 3.14159265358979323846f;

namespace config {
constexpr uint8_t drive_wheel_count = 3;
constexpr uint8_t wheel_count = drive_wheel_count;

constexpr float mount_radius_m = 0.1775f;
constexpr float mount_offset[wheel_count] = {
   kPi / 3.0f,   // 左前: +60°
   kPi,          // 後ろ: 180°（車輪の転がり方向は真横）
  -kPi / 3.0f    // 右前: -60°
};
constexpr float max_wheel_speed_mps = 1.0f;
}

struct BodyTwist { float vx, vy, w; };
struct WheelSpeeds { float v_wheel[config::wheel_count] = {}; };

// タブ間で呼び出す関数
void omni_Init();
void omni_SetBodyVelocity(float vx_mps, float vy_mps, float w_radps);
void omni_UpdateOutput(float dt_s);
bool omni_Forward(const WheelSpeeds &wheel, BodyTwist &body);

void mechanism_Init();
void mechanism_ArmUp();
void mechanism_ArmDown();
void mechanism_ArmStop();

void bluetooth_Init();
// 受信済みの最新コマンドを取り出す。無ければ0。
char bluetooth_TakeCommand();
// 切断されたら true を一度だけ返す。
bool bluetooth_TakeDisconnected();
