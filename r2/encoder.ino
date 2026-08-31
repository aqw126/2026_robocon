#include "robot_types.h"

// エンコーダー1は実配線。エンコーダー2・3はピン情報待ちの仮値。
// ENCODER_COUNTS_PER_WHEEL_REV はA相立上りで得られる車輪1回転当たりの実カウント数。
constexpr int ENC_A[config::wheel_count] = {2, 9, 11};
constexpr int ENC_B[config::wheel_count] = {1, 10, 12};
constexpr int ENCODER_DIRECTION_SIGN[config::wheel_count] = {1, 1, 1};
constexpr float ENCODER_COUNTS_PER_WHEEL_REV = 1440.0f;
constexpr uint32_t ENCODER_UPDATE_INTERVAL_US = 10000;

volatile int32_t encoderCount[config::wheel_count] = {0, 0, 0};
int32_t previousEncoderCount[config::wheel_count] = {0, 0, 0};
float wheelDeltaDistanceM[config::wheel_count] = {0.0f, 0.0f, 0.0f};
float wheelSpeedMps[config::wheel_count] = {0.0f, 0.0f, 0.0f};
float totalDistanceM[config::wheel_count] = {0.0f, 0.0f, 0.0f};
float encoderDtS = 0.0f;

void IRAM_ATTR encoder1_ISR() { encoderCount[0] += digitalRead(ENC_B[0]) ? ENCODER_DIRECTION_SIGN[0] : -ENCODER_DIRECTION_SIGN[0]; }
void IRAM_ATTR encoder2_ISR() { encoderCount[1] += digitalRead(ENC_B[1]) ? ENCODER_DIRECTION_SIGN[1] : -ENCODER_DIRECTION_SIGN[1]; }
void IRAM_ATTR encoder3_ISR() { encoderCount[2] += digitalRead(ENC_B[2]) ? ENCODER_DIRECTION_SIGN[2] : -ENCODER_DIRECTION_SIGN[2]; }

void encoder_Init() {
  for (uint8_t i = 0; i < config::wheel_count; ++i) { pinMode(ENC_A[i], INPUT_PULLUP); pinMode(ENC_B[i], INPUT_PULLUP); }
  attachInterrupt(digitalPinToInterrupt(ENC_A[0]), encoder1_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A[1]), encoder2_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A[2]), encoder3_ISR, RISING);
}

bool encoder_Update() {
  static uint32_t previousUs = 0; const uint32_t nowUs = micros();
  if (previousUs != 0 && nowUs - previousUs < ENCODER_UPDATE_INTERVAL_US) return false;
  encoderDtS = previousUs == 0 ? ENCODER_UPDATE_INTERVAL_US * 1.0e-6f : (nowUs - previousUs) * 1.0e-6f;
  previousUs = nowUs;
  int32_t currentCount[config::wheel_count];
  noInterrupts(); for (uint8_t i = 0; i < config::wheel_count; ++i) currentCount[i] = encoderCount[i]; interrupts();
  const float meterPerCount = 2.0f * kPi * config::wheel_radius_m / ENCODER_COUNTS_PER_WHEEL_REV;
  for (uint8_t i = 0; i < config::wheel_count; ++i) {
    const int32_t deltaCount = currentCount[i] - previousEncoderCount[i]; previousEncoderCount[i] = currentCount[i];
    wheelDeltaDistanceM[i] = deltaCount * meterPerCount; wheelSpeedMps[i] = wheelDeltaDistanceM[i] / encoderDtS;
    totalDistanceM[i] += wheelDeltaDistanceM[i];
  }
  return true;
}
