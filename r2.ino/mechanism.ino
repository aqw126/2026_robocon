#include "mission_support.h"

// ============================================================
// じょうろ取得・散水機構のAPI
// 実機のアクチュエータが未確定のため、初期値は出力無効。
// ピンとON/OFF極性を決めれば、上位のmission.inoは変更不要。
// ============================================================
constexpr int MECHANISM_OUTPUT_PIN = -1;  // 例: 27。未接続時は -1
constexpr bool MECHANISM_ACTIVE_HIGH = true;
constexpr uint32_t GET_CAN_DURATION_MS = 800;
constexpr uint32_t WATER_DURATION_MS = 1500;

enum MechanismAction { MECHANISM_IDLE, MECHANISM_GET_CAN, MECHANISM_WATER };
MechanismAction mechanismAction = MECHANISM_IDLE;
uint32_t mechanismActionStartMs = 0;

void mechanism_Write(bool active) {
  if (MECHANISM_OUTPUT_PIN < 0) return;
  const bool level = MECHANISM_ACTIVE_HIGH ? active : !active;
  digitalWrite(MECHANISM_OUTPUT_PIN, level ? HIGH : LOW);
}

void mechanism_Init() {
  if (MECHANISM_OUTPUT_PIN >= 0) {
    pinMode(MECHANISM_OUTPUT_PIN, OUTPUT);
    mechanism_Write(false);
  }
}

void mechanism_Start(MechanismAction action) {
  mechanismAction = action;
  mechanismActionStartMs = millis();
  mechanism_Write(true);
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
  mechanism_Write(false);
  mechanismAction = MECHANISM_IDLE;
}
