#include "robot_types.h"

// AMT102測定輪: E1=A41/B40、E2=A2/B1。
constexpr int ENC_A[config::tracking_encoder_count] = {41, 2};
constexpr int ENC_B[config::tracking_encoder_count] = {40, 1};

// 暫定符号。
// +1: このデコーダーのカウント増加をrobot_types.hのtracking_direction_rad方向の移動として扱う。
// 診断結果で反対だったエンコーダーだけ-1へ変更する。
constexpr int8_t ENCODER_DIRECTION_SIGN[config::tracking_encoder_count] = {1, 1};

// A/B両相のCHANGEを数える4逓倍後の「測定輪1回転当たり実カウント数」。
// AMT102のDIP設定が未確認のため1440は暫定値。
constexpr float ENCODER_COUNTS_PER_REV = 1440.0f;
constexpr uint32_t ENCODER_UPDATE_INTERVAL_US = 10000;

volatile int32_t encoderCount[config::tracking_encoder_count] = {0, 0};
volatile uint8_t encoderPreviousState[config::tracking_encoder_count] = {0, 0};
int32_t previousEncoderCount[config::tracking_encoder_count] = {0, 0};
float trackingEncoderDeltaDistanceM[config::tracking_encoder_count] = {0.0f, 0.0f};
float trackingEncoderSpeedMps[config::tracking_encoder_count] = {0.0f, 0.0f};
float trackingEncoderTotalDistanceM[config::tracking_encoder_count] = {0.0f, 0.0f};
float encoderDtS = 0.0f;

void IRAM_ATTR encoder_UpdateState(uint8_t index) {
  const uint8_t currentState =
    (static_cast<uint8_t>(digitalRead(ENC_A[index])) << 1) |
    static_cast<uint8_t>(digitalRead(ENC_B[index]));
  const uint8_t transition = (encoderPreviousState[index] << 2) | currentState;
  int8_t delta = 0;

  switch (transition) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      delta = 1;
      break;
    case 0b0010:
    case 0b1011:
    case 0b1101:
    case 0b0100:
      delta = -1;
      break;
  }

  encoderCount[index] += delta * ENCODER_DIRECTION_SIGN[index];
  encoderPreviousState[index] = currentState;
}

void IRAM_ATTR encoder1A_ISR() { encoder_UpdateState(0); }
void IRAM_ATTR encoder1B_ISR() { encoder_UpdateState(0); }
void IRAM_ATTR encoder2A_ISR() { encoder_UpdateState(1); }
void IRAM_ATTR encoder2B_ISR() { encoder_UpdateState(1); }

void encoder_Init() {
  for (uint8_t i = 0; i < config::tracking_encoder_count; ++i) {
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
    encoderPreviousState[i] =
      (static_cast<uint8_t>(digitalRead(ENC_A[i])) << 1) |
      static_cast<uint8_t>(digitalRead(ENC_B[i]));
  }

  attachInterrupt(digitalPinToInterrupt(ENC_A[0]), encoder1A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B[0]), encoder1B_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[1]), encoder2A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B[1]), encoder2B_ISR, CHANGE);
}

bool encoder_Update() {
  static uint32_t previousUs = 0;
  const uint32_t nowUs = micros();
  if (previousUs != 0 && nowUs - previousUs < ENCODER_UPDATE_INTERVAL_US) return false;
  encoderDtS = previousUs == 0 ? ENCODER_UPDATE_INTERVAL_US * 1.0e-6f : (nowUs - previousUs) * 1.0e-6f;
  previousUs = nowUs;

  int32_t currentCount[config::tracking_encoder_count];
  noInterrupts();
  for (uint8_t i = 0; i < config::tracking_encoder_count; ++i) currentCount[i] = encoderCount[i];
  interrupts();

  const float meterPerCount =
    2.0f * kPi * config::tracking_wheel_radius_m / ENCODER_COUNTS_PER_REV;
  for (uint8_t i = 0; i < config::tracking_encoder_count; ++i) {
    const int32_t deltaCount = currentCount[i] - previousEncoderCount[i];
    previousEncoderCount[i] = currentCount[i];
    trackingEncoderDeltaDistanceM[i] = deltaCount * meterPerCount;
    trackingEncoderSpeedMps[i] = trackingEncoderDeltaDistanceM[i] / encoderDtS;
    trackingEncoderTotalDistanceM[i] += trackingEncoderDeltaDistanceM[i];
  }
  return true;
}
