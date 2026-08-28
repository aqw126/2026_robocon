// ============================================================
// ESP32-S3
// Arduino IDE
//
// 三輪オムニホイール用
// A相・B相エンコーダー × 3
//
// 取得するもの
//   ・累積エンコーダーカウント
//   ・前回からのカウント差 Δcount
//   ・前回からの車輪移動量 Δs [m]
// ============================================================


// ============================================================
// エンコーダーピン
// ※ 仮のピン番号
// ============================================================

#define ENC1_A 1
#define ENC1_B 2

#define ENC2_A 3
#define ENC2_B 4

#define ENC3_A 5
#define ENC3_B 6


// ============================================================
// エンコーダー設定
// ============================================================

// エンコーダー1回転あたりのカウント数
//
// 今回は仮に500カウント/回転とする
//
// 注意:
// 実際のエンコーダーによって値を変更する
//
#define ENCODER_PPR 500.0


// ============================================================
// 車輪設定
// ============================================================

// 車輪半径 [m]
// 0.05m = 5cm
#define WHEEL_RADIUS 0.05


// ============================================================
// 更新周期
// ============================================================

// 10ms = 100Hz
#define UPDATE_INTERVAL_MS 10


// ============================================================
// エンコーダーカウント
//
// 割り込みによって変更されるので volatile
// ============================================================

volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;
volatile long encoderCount3 = 0;


// ============================================================
// 前回のエンコーダーカウント
// ============================================================

long previousCount1 = 0;
long previousCount2 = 0;
long previousCount3 = 0;


// ============================================================
// 累積移動距離
//
// デバッグなどに使える
// ============================================================

float totalDistance1 = 0.0;
float totalDistance2 = 0.0;
float totalDistance3 = 0.0;


// ============================================================
// エンコーダー1 割り込み
// ============================================================

void IRAM_ATTR encoder1_ISR()
{
  if (digitalRead(ENC1_B)) {
    encoderCount1++;
  }
  else {
    encoderCount1--;
  }
}


// ============================================================
// エンコーダー2 割り込み
// ============================================================

void IRAM_ATTR encoder2_ISR()
{
  if (digitalRead(ENC2_B)) {
    encoderCount2++;
  }
  else {
    encoderCount2--;
  }
}


// ============================================================
// エンコーダー3 割り込み
// ============================================================

void IRAM_ATTR encoder3_ISR()
{
  if (digitalRead(ENC3_B)) {
    encoderCount3++;
  }
  else {
    encoderCount3--;
  }
}


// ============================================================
// setup
// ============================================================

void encoder_Init()
{
  // ----------------------------------------------------------
  // エンコーダーピン設定
  // ----------------------------------------------------------

  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);

  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  pinMode(ENC3_A, INPUT_PULLUP);
  pinMode(ENC3_B, INPUT_PULLUP);


  // ----------------------------------------------------------
  // A相の立ち上がりで割り込み
  // ----------------------------------------------------------

  attachInterrupt(
    digitalPinToInterrupt(ENC1_A),
    encoder1_ISR,
    RISING
  );

  attachInterrupt(
    digitalPinToInterrupt(ENC2_A),
    encoder2_ISR,
    RISING
  );

  attachInterrupt(
    digitalPinToInterrupt(ENC3_A),
    encoder3_ISR,
    RISING
  );


  Serial.println("================================");
  Serial.println("Encoder / Odometry test start");
  Serial.println("================================");
}


// ============================================================
// loop
// ============================================================

void encoder()
{
  static unsigned long previousTime = 0;

  unsigned long currentTime = millis();


  // ==========================================================
  // 10msごとに処理
  // ==========================================================

  if (currentTime - previousTime >= UPDATE_INTERVAL_MS)
  {
    previousTime = currentTime;


    // ========================================================
    // エンコーダーカウントを安全にコピー
    // ========================================================

    noInterrupts();

    long currentCount1 = encoderCount1;
    long currentCount2 = encoderCount2;
    long currentCount3 = encoderCount3;

    interrupts();


    // ========================================================
    // 前回からのカウント差 Δcount
    // ========================================================

    long deltaCount1 =
      currentCount1 - previousCount1;

    long deltaCount2 =
      currentCount2 - previousCount2;

    long deltaCount3 =
      currentCount3 - previousCount3;


    // ========================================================
    // 今回のカウントを次回の「前回」にする
    // ========================================================

    previousCount1 = currentCount1;
    previousCount2 = currentCount2;
    previousCount3 = currentCount3;


    // ========================================================
    // Δcount → 車輪移動量 Δs
    //
    // 1回転の距離
    // = 2πr
    //
    // Δs
    // = Δcount / PPR × 2πr
    // ========================================================

    float deltaDistance1 =
      ((float)deltaCount1 / ENCODER_PPR)
      * 2.0
      * PI
      * WHEEL_RADIUS;

    float deltaDistance2 =
      ((float)deltaCount2 / ENCODER_PPR)
      * 2.0
      * PI
      * WHEEL_RADIUS;

    float deltaDistance3 =
      ((float)deltaCount3 / ENCODER_PPR)
      * 2.0
      * PI
      * WHEEL_RADIUS;


    // ========================================================
    // 累積移動距離
    // ========================================================

    totalDistance1 += deltaDistance1;
    totalDistance2 += deltaDistance2;
    totalDistance3 += deltaDistance3;


    // ========================================================
    // 出力
    // ========================================================

    Serial.print("dS1 = ");
    Serial.print(deltaDistance1, 5);

    Serial.print(" m    dS2 = ");
    Serial.print(deltaDistance2, 5);

    Serial.print(" m    dS3 = ");
    Serial.print(deltaDistance3, 5);

    Serial.println(" m");


    // ========================================================
    // デバッグ用：累積値
    // ========================================================

    Serial.print("Total: ");

    Serial.print(totalDistance1, 3);
    Serial.print(" m    ");

    Serial.print(totalDistance2, 3);
    Serial.print(" m    ");

    Serial.print(totalDistance3, 3);
    Serial.println(" m");

    Serial.println();
  }
}