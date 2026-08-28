// 
// この資料の前提:
//   N(wheel_count)個のホイールが、N角正多角形を形成するように並んでいる
//   ホイールの取付半径は mount_radius [mm]
//   正面方向から α_i (mount_offset[i]) [rad] ずれている
//   
// やりたいこと:
//   機体をどう動かしたいかを Twist (vx, vy, w) で表した。
//   これをホイールごとの速度に分解する。
// 
// ここに書いてあること:
//   - config               前提となる定数群
//   - BodyTwist            型の定義
//   - WheelSpeeds          型の定義
//   - inverse(BodyTwist)   逆運動学変換関数の定義
//   - simple_ik(bodyTwist) 裏技的な変換関数の定義 (4輪かつ取り付けがX配置の場合のみ)
// 
#include <stdint.h>
#include <math.h>
constexpr float kPi = 3.14159265358979323846f; // π


// ---- 機体設定 ----


namespace config {
constexpr uint8_t wheel_count = 3;                // 機体のホイール数
constexpr float mount_radius = f;            // 機体中心からホイールまでの距離 [mm]
constexpr float mount_offset[wheel_count] = {     // ホイールの取付角
    kPi * 1.0f / 2.0f,
    kPi * 1.0f / 3.0f,
    kPi * 2.0f / 3.0f,
};
}

// モーターのピン設定
const int IN1 = 4;
const int SD1 = 5;

const int IN2 = 6;
const int SD2 = 7;

const int IN3 = 15;
const int SD3 = 16;

// ---- 使う構造体 2つ (Twist, WheelSpeeds) ----


// 機体全体の動きを表す
// vx, vy: [mm/s] 速度ベクトル, 機体の正面+x, 左+y
// w:     [rad/s] 角速度, CCW
struct BodyTwist {
    float vx, vy;
    float w;
};

// ホイールそれぞれの速さ
struct WheelSpeeds {
    float v_wheel[config::wheel_count] = {};
};


// ---- 逆運動学関数 ----
void omni_Init(){
  pinMode(IN1, OUTPUT);
  pinMode(SD1, OUTPUT);

  pinMode(IN2, OUTPUT);
  pinMode(SD2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(SD3, OUTPUT);
}

// 機体の実現したい動き方をホイール単位に分解する
WheelSpeeds inverse(BodyTwist t) {
    WheelSpeeds ws;
    for (int i=0; i<config::wheel_count; i++) {
        float alpha = config::mount_offset[i];

        // 並進ぶん: 機体の速度ベクトルを、このホイールが転がる向きに射影する
        // 旋回ぶん: 半径 R の円周上の速さ = R * w
        ws.v_wheel[i] = - sinf(alpha) * t.vx
                        + cosf(alpha) * t.vy
                        + config::mount_radius * t.w;
    }
    return ws;
}

