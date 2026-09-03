// 2個のエンコーダー方向確認用
// ボードは「ESP32S3 Dev Module」を選択してください。
// シリアルモニター: 115200 bps

constexpr int ENC1_A = 41;
constexpr int ENC1_B = 40;
constexpr int ENC2_A = 2;
constexpr int ENC2_B = 1;

constexpr unsigned long PRINT_INTERVAL_MS = 200;

volatile int32_t encoder1Count = 0;
volatile int32_t encoder2Count = 0;

// メインコードと同じく、A相の立ち上がり時にB相を読んで方向を判定する。
void IRAM_ATTR encoder1ISR() {
  encoder1Count += digitalRead(ENC1_B) ? 1 : -1;
}

void IRAM_ATTR encoder2ISR() {
  encoder2Count += digitalRead(ENC2_B) ? 1 : -1;
}

void setup() {
  Serial.begin(115200);

  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC1_A), encoder1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), encoder2ISR, RISING);

  Serial.println("2-encoder direction test ready");
  Serial.println("Turn one measuring wheel at a time in its defined positive direction.");
  Serial.println("count increases: sign=1, count decreases: sign=-1");
}

void loop() {
  static unsigned long previousPrintMs = 0;
  static int32_t previousCount1 = 0;
  static int32_t previousCount2 = 0;

  const unsigned long nowMs = millis();
  if (nowMs - previousPrintMs < PRINT_INTERVAL_MS) return;
  previousPrintMs = nowMs;

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

  Serial.print("ENC1 total=");
  Serial.print(count1);
  Serial.print(" delta=");
  Serial.print(delta1);
  Serial.print(" | ENC2 total=");
  Serial.print(count2);
  Serial.print(" delta=");
  Serial.println(delta2);
}
