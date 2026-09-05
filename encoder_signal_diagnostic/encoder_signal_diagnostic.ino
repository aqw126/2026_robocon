// AMT102 2個のA/B相信号を個別に確認する診断コード
// ボード: ESP32S3 Dev Module
// シリアルモニター: 115200 baud

#include <Arduino.h>

constexpr int ENC1_A_PIN = 41;
constexpr int ENC1_B_PIN = 40;
constexpr int ENC2_A_PIN = 2;
constexpr int ENC2_B_PIN = 1;

constexpr unsigned long SAMPLE_INTERVAL_MS = 100;
constexpr unsigned long IDLE_PRINT_INTERVAL_MS = 1000;

volatile int32_t encoder1AEdges = 0;
volatile int32_t encoder1BEdges = 0;
volatile int32_t encoder1QuadratureCount = 0;
volatile int32_t encoder1InvalidTransitions = 0;
volatile uint8_t encoder1PreviousState = 0;

volatile int32_t encoder2AEdges = 0;
volatile int32_t encoder2BEdges = 0;
volatile int32_t encoder2QuadratureCount = 0;
volatile int32_t encoder2InvalidTransitions = 0;
volatile uint8_t encoder2PreviousState = 0;

void IRAM_ATTR updateEncoder1() {
  const uint8_t currentState =
    (static_cast<uint8_t>(digitalRead(ENC1_A_PIN)) << 1) |
    static_cast<uint8_t>(digitalRead(ENC1_B_PIN));
  const uint8_t transition = (encoder1PreviousState << 2) | currentState;

  switch (transition) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      ++encoder1QuadratureCount;
      break;
    case 0b0010:
    case 0b1011:
    case 0b1101:
    case 0b0100:
      --encoder1QuadratureCount;
      break;
    default:
      if (currentState != encoder1PreviousState) ++encoder1InvalidTransitions;
      break;
  }

  encoder1PreviousState = currentState;
}

void IRAM_ATTR updateEncoder2() {
  const uint8_t currentState =
    (static_cast<uint8_t>(digitalRead(ENC2_A_PIN)) << 1) |
    static_cast<uint8_t>(digitalRead(ENC2_B_PIN));
  const uint8_t transition = (encoder2PreviousState << 2) | currentState;

  switch (transition) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      ++encoder2QuadratureCount;
      break;
    case 0b0010:
    case 0b1011:
    case 0b1101:
    case 0b0100:
      --encoder2QuadratureCount;
      break;
    default:
      if (currentState != encoder2PreviousState) ++encoder2InvalidTransitions;
      break;
  }

  encoder2PreviousState = currentState;
}

void IRAM_ATTR encoder1A_ISR() {
  ++encoder1AEdges;
  updateEncoder1();
}

void IRAM_ATTR encoder1B_ISR() {
  ++encoder1BEdges;
  updateEncoder1();
}

void IRAM_ATTR encoder2A_ISR() {
  ++encoder2AEdges;
  updateEncoder2();
}

void IRAM_ATTR encoder2B_ISR() {
  ++encoder2BEdges;
  updateEncoder2();
}

struct EncoderSnapshot {
  int32_t aEdges;
  int32_t bEdges;
  int32_t quadratureCount;
  int32_t invalidTransitions;
};

void printEncoder(
  const char *name,
  int aPin,
  int bPin,
  const EncoderSnapshot &current,
  const EncoderSnapshot &previous
) {
  Serial.print(name);
  Serial.print(" A(GPIO");
  Serial.print(aPin);
  Serial.print(")=");
  Serial.print(digitalRead(aPin));
  Serial.print(" edges=");
  Serial.print(current.aEdges);
  Serial.print(" (+");
  Serial.print(current.aEdges - previous.aEdges);
  Serial.print(")");

  Serial.print("  B(GPIO");
  Serial.print(bPin);
  Serial.print(")=");
  Serial.print(digitalRead(bPin));
  Serial.print(" edges=");
  Serial.print(current.bEdges);
  Serial.print(" (+");
  Serial.print(current.bEdges - previous.bEdges);
  Serial.print(")");

  Serial.print("  Q=");
  Serial.print(current.quadratureCount);
  Serial.print(" dQ=");
  Serial.print(current.quadratureCount - previous.quadratureCount);
  Serial.print("  invalid=");
  Serial.print(current.invalidTransitions);
  Serial.print(" (+");
  Serial.print(current.invalidTransitions - previous.invalidTransitions);
  Serial.println(")");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(ENC1_A_PIN, INPUT_PULLUP);
  pinMode(ENC1_B_PIN, INPUT_PULLUP);
  pinMode(ENC2_A_PIN, INPUT_PULLUP);
  pinMode(ENC2_B_PIN, INPUT_PULLUP);

  encoder1PreviousState =
    (static_cast<uint8_t>(digitalRead(ENC1_A_PIN)) << 1) |
    static_cast<uint8_t>(digitalRead(ENC1_B_PIN));
  encoder2PreviousState =
    (static_cast<uint8_t>(digitalRead(ENC2_A_PIN)) << 1) |
    static_cast<uint8_t>(digitalRead(ENC2_B_PIN));

  attachInterrupt(digitalPinToInterrupt(ENC1_A_PIN), encoder1A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B_PIN), encoder1B_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A_PIN), encoder2A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B_PIN), encoder2B_ISR, CHANGE);

  Serial.println("=== AMT102 A/B signal diagnostic ===");
  Serial.println("Board: ESP32S3 Dev Module / Monitor: 115200 baud");
  Serial.println("Lift the robot and rotate only one measuring wheel at a time.");
  Serial.println("A edges and B edges must both increase; dQ must reverse when rotation reverses.");
  Serial.println();
}

void loop() {
  static unsigned long previousSampleMs = 0;
  static unsigned long previousPrintMs = 0;
  static EncoderSnapshot previous1 = {0, 0, 0, 0};
  static EncoderSnapshot previous2 = {0, 0, 0, 0};

  const unsigned long nowMs = millis();
  if (nowMs - previousSampleMs < SAMPLE_INTERVAL_MS) return;
  previousSampleMs = nowMs;

  EncoderSnapshot current1;
  EncoderSnapshot current2;
  noInterrupts();
  current1 = {
    encoder1AEdges,
    encoder1BEdges,
    encoder1QuadratureCount,
    encoder1InvalidTransitions
  };
  current2 = {
    encoder2AEdges,
    encoder2BEdges,
    encoder2QuadratureCount,
    encoder2InvalidTransitions
  };
  interrupts();

  const bool edgeChanged =
    current1.aEdges != previous1.aEdges ||
    current1.bEdges != previous1.bEdges ||
    current2.aEdges != previous2.aEdges ||
    current2.bEdges != previous2.bEdges;
  const bool idlePrintDue = nowMs - previousPrintMs >= IDLE_PRINT_INTERVAL_MS;

  if (edgeChanged || idlePrintDue) {
    printEncoder("ENC1", ENC1_A_PIN, ENC1_B_PIN, current1, previous1);
    printEncoder("ENC2", ENC2_A_PIN, ENC2_B_PIN, current2, previous2);
    Serial.println();
    previousPrintMs = nowMs;
  }

  previous1 = current1;
  previous2 = current2;
}
