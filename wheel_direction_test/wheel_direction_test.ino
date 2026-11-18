// ============================================================
// 駆動輪 1輪ずつの回転方向テスト（r2_bluetooth と同じピン配線）
// シリアルモニタ(115200, 改行なし/あり どちらでも可)から1文字送ると、その輪だけ短時間回して止まる。
//
//   0 / 1 / 2 : 左前 / 後ろ / 右前 を「正方向」（INピンにPWM）
//   3 / 4 / 5 : 左前 / 後ろ / 右前 を「逆方向」（SDピンにPWM）
//   a         : 3輪を同時に正方向（r2_bluetoothで w>0 = 反時計回りのときと同じ組み合わせ）
//   x / y     : アーム（r2_bluetoothの x=上昇 / y=下降 と同じ出力）
//   s         : 即停止
//
// 期待値（r2/omni.ino の逆運動学より）:
//   「正方向」= その車輪が機体を反時計回り(上から見て左回り)に回す向き。
//   床に置いて1輪だけ回すと、機体はその車輪の位置を反時計回りに押す方向へ動く。
//   a は3輪とも正なので、機体はその場で「反時計回り」に回る。
//   逆に動く輪があれば、その輪のIN/SDの配線かピン番号が逆。
// ============================================================
constexpr int MOTOR_IN[3] = {4, 6, 15};   // 0=左前、1=後ろ、2=右前
constexpr int MOTOR_SD[3] = {5, 7, 16};
constexpr int WHEEL_PWM = 50;             // r2_bluetoothの上限(75)より低め
constexpr uint32_t PULSE_MS = 1000;

constexpr int ARM_IN_PIN = 17;
constexpr int ARM_PWM_PIN = 18;
constexpr uint32_t ARM_PULSE_MS = 500;

const char *const WHEEL_NAME[3] = {"左前", "後ろ", "右前"};
uint32_t stopAtMs = 0;
bool running = false;

void stopAll() {
  for (int i = 0; i < 3; ++i) {
    analogWrite(MOTOR_IN[i], 0);
    analogWrite(MOTOR_SD[i], 0);
  }
  analogWrite(ARM_PWM_PIN, 0);
  digitalWrite(ARM_IN_PIN, LOW);
  running = false;
}

void spinWheel(int wheel, bool positive) {
  stopAll();
  analogWrite(positive ? MOTOR_IN[wheel] : MOTOR_SD[wheel], WHEEL_PWM);
  Serial.print(WHEEL_NAME[wheel]);
  Serial.println(positive ? " 正方向" : " 逆方向");
  stopAtMs = millis() + PULSE_MS;
  running = true;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  for (int i = 0; i < 3; ++i) {
    pinMode(MOTOR_IN[i], OUTPUT);
    pinMode(MOTOR_SD[i], OUTPUT);
  }
  pinMode(ARM_IN_PIN, OUTPUT);
  pinMode(ARM_PWM_PIN, OUTPUT);
  stopAll();
  Serial.println("wheel direction test: 0/1/2=正 3/4/5=逆 a=3輪正 x/y=アーム s=停止");
}

void loop() {
  if (running && static_cast<int32_t>(millis() - stopAtMs) >= 0) {
    stopAll();
    Serial.println("stop");
  }

  while (Serial.available()) {
    const char c = Serial.read();
    if (c >= '0' && c <= '2') spinWheel(c - '0', true);
    else if (c >= '3' && c <= '5') spinWheel(c - '3', false);
    else if (c == 'a') {
      stopAll();
      for (int i = 0; i < 3; ++i) analogWrite(MOTOR_IN[i], WHEEL_PWM);
      Serial.println("3輪 正方向（機体は反時計回りに回るはず）");
      stopAtMs = millis() + PULSE_MS;
      running = true;
    } else if (c == 'x') {  // r2_bluetooth の ArmUp と同じ
      stopAll();
      digitalWrite(ARM_IN_PIN, HIGH);
      analogWrite(ARM_PWM_PIN, 255 - 255);  // r2_bluetooth と同じ反転出力（255-duty、duty=255）
      Serial.println("アーム x（上昇のはず）");
      stopAtMs = millis() + ARM_PULSE_MS;
      running = true;
    } else if (c == 'y') {  // r2_bluetooth の ArmDown と同じ
      stopAll();
      digitalWrite(ARM_IN_PIN, LOW);
      analogWrite(ARM_PWM_PIN, 255);
      Serial.println("アーム y（下降のはず）");
      stopAtMs = millis() + ARM_PULSE_MS;
      running = true;
    } else if (c == 's') {
      stopAll();
      Serial.println("stop");
    }
  }
}
