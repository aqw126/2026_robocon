#include "robot_types.h"
#include "mission_support.h"

//ToFはつけないのでこのフォルダは無視
// ============================================================
// ToF入力と位置補正の受け口
// センサー型番未確定のため、初期値は無効。VL53L0X等を決めたら
// tof_Update() 内だけをライブラリに合わせて実装する。
// ============================================================
constexpr bool TOF_SENSOR_ENABLED = false;
constexpr uint32_t TOF_UPDATE_INTERVAL_MS = 50;
constexpr float TOF_MIN_VALID_M = 0.03f;
constexpr float TOF_MAX_VALID_M = 2.00f;

ToFReading frontTof;

void tof_Init() {
  if (!TOF_SENSOR_ENABLED) {
    Serial.println("ToF: disabled (sensor type/pins not configured)");
    return;
  }

  // TODO: 使用するToFのライブラリをここで初期化する。
  // 例: VL53L0X/VL53L1Xのbegin()、I2Cアドレス、XSHUTピンの設定。
}

void tof_Update() {
  static uint32_t lastUpdateMs = 0;
  if (!TOF_SENSOR_ENABLED || millis() - lastUpdateMs < TOF_UPDATE_INTERVAL_MS) return;
  lastUpdateMs = millis();

  // TODO: ライブラリから距離[m]を読み、範囲確認後にfrontTofへ保存する。
  // const float measured_m = ...;
  // frontTof.valid = measured_m >= TOF_MIN_VALID_M && measured_m <= TOF_MAX_VALID_M;
  // if (frontTof.valid) frontTof.distance_m = measured_m;
  frontTof.valid = false;
}

bool tof_GetFrontDistance(ToFReading &reading) {
  reading = frontTof;
  return reading.valid;
}
