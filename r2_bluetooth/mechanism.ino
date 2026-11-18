#include "robot_types.h"

// ============================================================
// アーム昇降。ピンは現在の配線に合わせて変更済み。
// r2では時間制御の片方向だけだったため、Bluetooth操作用に逆方向を追加した。
// ============================================================
constexpr int ARM_IN_PIN = 17;
constexpr int ARM_PWM_PIN = 18;
constexpr int ARM_PWM_DUTY = 255;

void mechanism_Init() {
  pinMode(ARM_IN_PIN, OUTPUT);
  pinMode(ARM_PWM_PIN, OUTPUT);
  mechanism_ArmStop();
}

// 実機確認: IN=HIGH（PWM反転）側がアーム上昇。duty 255 -> 出力0 = 最大速度。
void mechanism_ArmUp() {
  digitalWrite(ARM_IN_PIN, HIGH);
  analogWrite(ARM_PWM_PIN, 255 - ARM_PWM_DUTY);
}

// 実機確認: IN=LOW, PWM=duty 側がアーム下降（r2/mechanism.ino の動作方向）。
void mechanism_ArmDown() {
  digitalWrite(ARM_IN_PIN, LOW);
  analogWrite(ARM_PWM_PIN, ARM_PWM_DUTY);
}

void mechanism_ArmStop() {
  analogWrite(ARM_PWM_PIN, 0);
  digitalWrite(ARM_IN_PIN, LOW);
}
