#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// r2と同じBNO055配線とI2Cアドレス。
constexpr int BNO_SDA_PIN = 21;
constexpr int BNO_SCL_PIN = 22;
constexpr uint8_t BNO_ADDRESS = 0x28;

// Ozobot DRVKitで使用しているオンボードRGB LEDの仮設定。
constexpr int RGB_LED_PIN = 42;
constexpr uint8_t LED_BRIGHTNESS = 30;

// 誤差や静止時の揺れを判定しないよう、15度以上の変化で方向を確定する。
constexpr float DIRECTION_THRESHOLD_DEG = 15.0f;

Adafruit_BNO055 bno(55, BNO_ADDRESS);
bool bnoAvailable = false;
float startHeadingDeg = 0.0f;
int directionResult = 0;  // +1=増加、-1=減少、0=未判定

void setLed(uint8_t red, uint8_t green, uint8_t blue) {
  rgbLedWrite(RGB_LED_PIN, red, green, blue);
}

float normalizeDegrees(float angleDeg) {
  while (angleDeg > 180.0f) angleDeg -= 360.0f;
  while (angleDeg <= -180.0f) angleDeg += 360.0f;
  return angleDeg;
}

void setup() {
  // 青: 起動・初期化中
  setLed(0, 0, LED_BRIGHTNESS);

  Wire.begin(BNO_SDA_PIN, BNO_SCL_PIN);
  Wire.setClock(100000);
  bnoAvailable = bno.begin();
  if (!bnoAvailable) return;

  bno.setExtCrystalUse(true);
  delay(1000);
  startHeadingDeg = bno.getVector(Adafruit_BNO055::VECTOR_EULER).x();
}

void loop() {
  if (!bnoAvailable) {
    // 紫点滅: BNO055を認識できない。
    const bool on = (millis() / 300) % 2 == 0;
    setLed(on ? LED_BRIGHTNESS : 0, 0, on ? LED_BRIGHTNESS : 0);
    return;
  }

  if (directionResult == 0) {
    const float headingDeg = bno.getVector(Adafruit_BNO055::VECTOR_EULER).x();
    const float changeDeg = normalizeDegrees(headingDeg - startHeadingDeg);

    if (changeDeg >= DIRECTION_THRESHOLD_DEG) directionResult = 1;
    else if (changeDeg <= -DIRECTION_THRESHOLD_DEG) directionResult = -1;
  }

  if (directionResult > 0) {
    setLed(0, LED_BRIGHTNESS, 0);  // 緑: 反時計回りで角度が増加
  } else if (directionResult < 0) {
    setLed(LED_BRIGHTNESS, 0, 0);  // 赤: 反時計回りで角度が減少
  } else {
    setLed(0, 0, LED_BRIGHTNESS);  // 青: まだ15度以上回していない
  }

  delay(20);
}
