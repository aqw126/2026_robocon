// RGB LEDで確認する2エンコーダー方向テスト
// ボード: ESP32S3 Dev Module
// Ozobot DRVKitのオンボードRGB LEDはGPIO42。
//
// エンコーダーを必ず1個ずつ回す。
//   緑: カウント増加  -> ENCODER_DIRECTION_SIGN = 1
//   赤: カウント減少  -> ENCODER_DIRECTION_SIGN = -1
//   黄: 2個が同時に動いた
//   消灯: 入力なし

#include <Arduino.h>

constexpr int ENC1_A = 41;
constexpr int ENC1_B = 40;
constexpr int ENC2_A = 2;
constexpr int ENC2_B = 1;
constexpr int RGB_LED_PIN = 42;

constexpr unsigned long SAMPLE_INTERVAL_MS = 50;
constexpr unsigned long HOLD_COLOR_MS = 400;
constexpr uint8_t LED_BRIGHTNESS = 30;

volatile int32_t encoder1Count = 0;
volatile int32_t encoder2Count = 0;

void IRAM_ATTR encoder1ISR() {
  encoder1Count += digitalRead(ENC1_B) ? 1 : -1;
}

void IRAM_ATTR encoder2ISR() {
  encoder2Count += digitalRead(ENC2_B) ? 1 : -1;
}

void setLed(uint8_t red, uint8_t green, uint8_t blue) {
  rgbLedWrite(RGB_LED_PIN, red, green, blue);
}

void setup() {
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC1_A), encoder1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), encoder2ISR, RISING);

  // 起動確認として青を1秒点灯する。
  setLed(0, 0, LED_BRIGHTNESS);
  delay(1000);
  setLed(0, 0, 0);
}

void loop() {
  static unsigned long previousSampleMs = 0;
  static unsigned long lastMovementMs = 0;
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

  const bool encoder1Moved = delta1 != 0;
  const bool encoder2Moved = delta2 != 0;

  if (encoder1Moved && encoder2Moved) {
    setLed(LED_BRIGHTNESS, LED_BRIGHTNESS, 0);  // 黄: 同時入力
    lastMovementMs = nowMs;
  } else if (encoder1Moved) {
    if (delta1 > 0) setLed(0, LED_BRIGHTNESS, 0);  // 緑: 増加
    else setLed(LED_BRIGHTNESS, 0, 0);             // 赤: 減少
    lastMovementMs = nowMs;
  } else if (encoder2Moved) {
    if (delta2 > 0) setLed(0, LED_BRIGHTNESS, 0);  // 緑: 増加
    else setLed(LED_BRIGHTNESS, 0, 0);             // 赤: 減少
    lastMovementMs = nowMs;
  } else if (nowMs - lastMovementMs >= HOLD_COLOR_MS) {
    setLed(0, 0, 0);
  }
}
