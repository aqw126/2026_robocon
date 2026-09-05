#include "mission_support.h"

// ============================================================
// アーム機構のAPI
// sketch_sep03cR (1).inoのloop() 1回分に相当する100msだけモーターを回し、
// その後はPWMを0にして停止する。
// ============================================================
constexpr int ARM_IN_PIN = 5;
constexpr int ARM_PWM_PIN = 4;
constexpr int ARM_SENSOR_PIN = 14;
constexpr int ARM_PWM_DUTY = 200;
constexpr uint32_t GET_CAN_DURATION_MS = 100;
constexpr uint32_t WATER_DURATION_MS = 1500;

enum MechanismAction { MECHANISM_IDLE, MECHANISM_GET_CAN, MECHANISM_WATER };
MechanismAction mechanismAction = MECHANISM_IDLE;
uint32_t mechanismActionStartMs = 0;

void mechanism_WriteArm(bool active) {
  if (active) {
    // 添付コードと同じ回転方向。
    digitalWrite(ARM_IN_PIN, LOW);
    analogWrite(ARM_PWM_PIN, ARM_PWM_DUTY);
  } else {
    analogWrite(ARM_PWM_PIN, 0);
    digitalWrite(ARM_IN_PIN, LOW);
  }
}

void mechanism_Init() {
  pinMode(ARM_IN_PIN, OUTPUT);
  pinMode(ARM_PWM_PIN, OUTPUT);
  pinMode(ARM_SENSOR_PIN, INPUT_PULLUP);
  mechanism_WriteArm(false);
}

void mechanism_Start(MechanismAction action) {
  mechanismAction = action;
  mechanismActionStartMs = millis();
  // 現在モーターを使うのは、初期動作中のアーム動作だけ。
  mechanism_WriteArm(action == MECHANISM_GET_CAN);
}

void mechanism_StartGetCan() { mechanism_Start(MECHANISM_GET_CAN); }
void mechanism_StartWater() { mechanism_Start(MECHANISM_WATER); }

bool mechanism_IsActionFinished() {
  if (mechanismAction == MECHANISM_IDLE) return true;
  const uint32_t duration = mechanismAction == MECHANISM_GET_CAN ? GET_CAN_DURATION_MS : WATER_DURATION_MS;
  if (millis() - mechanismActionStartMs < duration) return false;
  mechanism_Stop();
  return true;
}

void mechanism_Stop() {
  mechanism_WriteArm(false);
  mechanismAction = MECHANISM_IDLE;
}
