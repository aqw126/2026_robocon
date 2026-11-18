#include "robot_types.h"

// ============================================================
// アーム昇降。上昇方向は r2/mechanism.ino と同じ。ピンは現在の配線に合わせて変更済み。
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

// r2の「アームを動かす」方向（IN=LOW, PWM=duty）を上昇とする。
void mechanism_ArmUp() {
  digitalWrite(ARM_IN_PIN, LOW);
  analogWrite(ARM_PWM_PIN, ARM_PWM_DUTY);
}

// 逆方向: IN=HIGH、PWMは反転（duty 255 -> 出力0 = 最大速度）。
void mechanism_ArmDown() {
  digitalWrite(ARM_IN_PIN, HIGH);
  analogWrite(ARM_PWM_PIN, 255 - ARM_PWM_DUTY);
}

void mechanism_ArmStop() {
  analogWrite(ARM_PWM_PIN, 0);
  digitalWrite(ARM_IN_PIN, LOW);
}
