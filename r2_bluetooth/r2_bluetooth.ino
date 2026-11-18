#include "robot_types.h"

// ============================================================
// R2 Bluetooth手動操作版（エンコーダー・ジャイロ・オドメトリ不使用）
// 駆動は r2/omni.ino の逆運動学とスルーレート制限をそのまま使う。
//
// コマンド（1文字、R1_newと同じ割り当て + 左右回転）
//   f=前進  b=後退  l=左移動  r=右移動
//   o=左回転(反時計回り)  p=右回転(時計回り)
//   x=アーム上昇  y=アーム下降
//   s=全停止（走行＋アーム）
// ============================================================
constexpr float MOVE_SPEED_MPS = 0.25f;     // 並進速度。0.25m/s = MOTOR_PWM上限
constexpr float TURN_RATE_RADPS = 1.0f;     // 回転速度
constexpr uint32_t OUTPUT_INTERVAL_MS = 10;  // r2と同じ100Hzで出力更新

void applyCommand(char cmd) {
  switch (cmd) {
    case 'f': omni_SetBodyVelocity( MOVE_SPEED_MPS, 0.0f, 0.0f); break;
    case 'b': omni_SetBodyVelocity(-MOVE_SPEED_MPS, 0.0f, 0.0f); break;
    case 'l': omni_SetBodyVelocity(0.0f,  MOVE_SPEED_MPS, 0.0f); break;
    case 'r': omni_SetBodyVelocity(0.0f, -MOVE_SPEED_MPS, 0.0f); break;
    case 'o': omni_SetBodyVelocity(0.0f, 0.0f,  TURN_RATE_RADPS); break;
    case 'p': omni_SetBodyVelocity(0.0f, 0.0f, -TURN_RATE_RADPS); break;
    case 'x': mechanism_ArmUp(); break;
    case 'y': mechanism_ArmDown(); break;
    case 's':
      omni_SetBodyVelocity(0.0f, 0.0f, 0.0f);
      mechanism_ArmStop();
      break;
    default: return;
  }
  Serial.print("RX: ");
  Serial.println(cmd);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  omni_Init();
  mechanism_Init();
  bluetooth_Init();

  Serial.println("R2 Bluetooth controller ready");
}

void loop() {
  // 切断されたら暴走しないよう全停止する。
  if (bluetooth_TakeDisconnected()) applyCommand('s');

  const char cmd = bluetooth_TakeCommand();
  if (cmd != 0) applyCommand(cmd);

  static uint32_t previousMs = 0;
  const uint32_t nowMs = millis();
  if (nowMs - previousMs >= OUTPUT_INTERVAL_MS) {
    omni_UpdateOutput((nowMs - previousMs) * 0.001f);
    previousMs = nowMs;
  }
}
