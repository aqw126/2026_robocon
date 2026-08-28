// --- 三輪オムニ モーター制御テスト ---
// IN: 4, 6, 15
// SD(PWM): 5, 7, 16

// モーターのピン設定
const int IN1 = 4;
const int SD1 = 5;

const int IN2 = 6;
const int SD2 = 7;

const int IN3 = 15;
const int SD3 = 16;

void setup() {
  // ピンモード設定
  pinMode(IN1, OUTPUT);
  pinMode(SD1, OUTPUT);

  pinMode(IN2, OUTPUT);
  pinMode(SD2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(SD3, OUTPUT);

  Serial.begin(9600);
  Serial.println("三輪オムニ モーターテスト開始");
}

void loop() {
  testMotor(IN1, SD1, "Motor1");
  testMotor(IN2, SD2, "Motor2");
  testMotor(IN3, SD3, "Motor3");
}

// モーターのテスト関数
void testMotor(int IN, int SD, const char* name) {
  Serial.print(name);
  Serial.println(" 正転");

  digitalWrite(IN, HIGH);
  analogWrite(SD, 200);  // 0〜255（速度）

  delay(1500);

  Serial.print(name);
  Serial.println(" 停止");

  analogWrite(SD, 0);
  delay(800);

  Serial.print(name);
  Serial.println(" 逆転");

  digitalWrite(IN, LOW);
  analogWrite(SD, 200);

  delay(1500);

  Serial.print(name);
  Serial.println(" 停止");

  analogWrite(SD, 0);
  delay(800);
}
