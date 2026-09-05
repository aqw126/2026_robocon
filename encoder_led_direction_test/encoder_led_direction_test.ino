// シリアルモニターで確認する2エンコーダー増減テスト
// ボード: ESP32S3 Dev Module
// シリアルモニター: 115200 baud
//
// 機体の向きを変えずに平行移動させる。
//   エンコーダー1: 300度方向と、その反対の120度方向
//   エンコーダー2: 210度方向と、その反対の 30度方向

#include <Arduino.h>

constexpr int ENC1_A = 41;
constexpr int ENC1_B = 40;
constexpr int ENC2_A = 2;
constexpr int ENC2_B = 1;

constexpr unsigned long SAMPLE_INTERVAL_MS = 100;
constexpr unsigned long STATUS_INTERVAL_MS = 1000;

volatile int32_t encoder1Count = 0;
volatile int32_t encoder2Count = 0;

void IRAM_ATTR encoder1ISR() {
  encoder1Count += digitalRead(ENC1_B) ? 1 : -1;
}

void IRAM_ATTR encoder2ISR() {
  encoder2Count += digitalRead(ENC2_B) ? 1 : -1;
}

const char *countChangeName(int32_t delta) {
  if (delta > 0) return "INCREASE";
  if (delta < 0) return "DECREASE";
  return "STOP";
}

void printResult(int32_t count1, int32_t delta1, int32_t count2, int32_t delta2) {
  Serial.print("ENC1 count=");
  Serial.print(count1);
  Serial.print(" delta=");
  Serial.print(delta1);
  Serial.print(" ");
  Serial.print(countChangeName(delta1));

  Serial.print(" | ENC2 count=");
  Serial.print(count2);
  Serial.print(" delta=");
  Serial.print(delta2);
  Serial.print(" ");
  Serial.println(countChangeName(delta2));
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC1_A), encoder1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), encoder2ISR, RISING);

  Serial.println("=== Encoder count direction test ===");
  Serial.println("Board: ESP32S3 Dev Module / Monitor: 115200 baud");
  Serial.println("ENC1: move robot toward 300 deg, then 120 deg");
  Serial.println("ENC2: move robot toward 210 deg, then 30 deg");
  Serial.println("INCREASE=count up, DECREASE=count down, STOP=no change");
  Serial.println();
}

void loop() {
  static unsigned long previousSampleMs = 0;
  static unsigned long previousStatusMs = 0;
  static int32_t previousCount1 = 0;
  static int32_t previousCount2 = 0;

  const unsigned long nowMs = millis();
  if (nowMs - previousSampleMs < SAMPLE_INTERVAL_MS) return;
  previousSampleMs = nowMs;

  int32_t count1;
  int32_t count2;
  noInterrupts();
  count1 = encoder1Count;
  count2 = encoder2Count;
  interrupts();

  const int32_t delta1 = count1 - previousCount1;
  const int32_t delta2 = count2 - previousCount2;
  previousCount1 = count1;
  previousCount2 = count2;

  const bool countChanged = delta1 != 0 || delta2 != 0;
  const bool statusDue = nowMs - previousStatusMs >= STATUS_INTERVAL_MS;
  if (countChanged || statusDue) {
    printResult(count1, delta1, count2, delta2);
    previousStatusMs = nowMs;
  }
}
