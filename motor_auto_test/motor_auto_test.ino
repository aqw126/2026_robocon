// 三輪オムニホイール モーター自動確認用
// 電源投入後、3秒待ってから各モーターを1台ずつ両方向に動かします。
// シリアルモニターからのコマンド入力は不要です。
// 各車輪について「IN側駆動 -> 停止 -> SD側駆動 -> 停止」を実行します。

// M1: 後ろ（車輪は真横向き）
constexpr int M1_IN = 15;
constexpr int M1_SD = 16;

// M2: 左前
constexpr int M2_IN = 6;
constexpr int M2_SD = 7;

// M3: 右前
constexpr int M3_IN = 18;
constexpr int M3_SD = 17;

constexpr int MOTOR_PWM = 25;        // PWM 50で速すぎたため、半分から再確認する
constexpr unsigned long START_WAIT_MS = 3000;
constexpr unsigned long RUN_MS = 1500;
constexpr unsigned long STOP_MS = 800;

void stopAllMotors() {
  analogWrite(M1_IN, 0);
  analogWrite(M1_SD, 0);
  analogWrite(M2_IN, 0);
  analogWrite(M2_SD, 0);
  analogWrite(M3_IN, 0);
  analogWrite(M3_SD, 0);
}

void testMotor(int inPin, int sdPin, const char* name) {
  Serial.print(name);
  Serial.println(" IN側駆動");

  // IN側だけにPWMを出す。
  analogWrite(sdPin, 0);
  analogWrite(inPin, MOTOR_PWM);
  delay(RUN_MS);

  // 方向を切り替える前に、必ず両入力を0にする。
  analogWrite(inPin, 0);
  analogWrite(sdPin, 0);
  Serial.print(name);
  Serial.println(" 停止");
  delay(STOP_MS);

  Serial.print(name);
  Serial.println(" SD側駆動");

  // SD側だけにPWMを出す。
  analogWrite(inPin, 0);
  analogWrite(sdPin, MOTOR_PWM);
  delay(RUN_MS);

  analogWrite(inPin, 0);
  analogWrite(sdPin, 0);
  Serial.print(name);
  Serial.println(" 停止");
  delay(STOP_MS);
}

void setup() {
  Serial.begin(115200);

  pinMode(M1_IN, OUTPUT);
  pinMode(M1_SD, OUTPUT);
  pinMode(M2_IN, OUTPUT);
  pinMode(M2_SD, OUTPUT);
  pinMode(M3_IN, OUTPUT);
  pinMode(M3_SD, OUTPUT);

  stopAllMotors();

  Serial.println("モーター自動確認を3秒後に開始します");
  delay(START_WAIT_MS);

  testMotor(M1_IN, M1_SD, "M1 後ろ");
  testMotor(M2_IN, M2_SD, "M2 左前");
  testMotor(M3_IN, M3_SD, "M3 右前");

  stopAllMotors();
  Serial.println("確認終了：全モーター停止");
}

void loop() {
  // 安全のため、確認動作は電源投入ごとに1回だけ実行します。
}
