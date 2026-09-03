#include <Arduino.h>

// ============================================================
// ESP32-S3 三輪オムニ: モーター方向確認専用スケッチ
// シリアルモニター: 115200 baud / 改行コードは任意
//
// motor_auto_test.ino と同じく、停止側のピンをLOWにして
// 反対側のピンへ analogWrite() でPWMを出力します。
// 1回の操作はPHASE_MS後に自動停止します。
// ============================================================

constexpr uint8_t MOTOR_COUNT = 3;

// 車輪順: 0=左前、1=後ろ、2=右前
// 正方向は motor_auto_test.ino で使用している向きです。
constexpr int MOTOR_IN[MOTOR_COUNT] = {6, 15, 18};
constexpr int MOTOR_SD[MOTOR_COUNT] = {7, 16, 17};

constexpr int MOTOR_PWM = 25;
constexpr uint32_t PHASE_MS = 3000;

bool motorIsRunning = false;
uint32_t motorStartedMs = 0;

// signedPwm > 0: SD=LOW、IN=PWM（motor_auto_test.inoと同じ向き）
// signedPwm < 0: IN=LOW、SD=PWM（逆向き）
void writeMotorRaw(uint8_t motor, int signedPwm) {
  if (motor >= MOTOR_COUNT) return;

  signedPwm = constrain(signedPwm, -MOTOR_PWM, MOTOR_PWM);
  const int duty = abs(signedPwm);

  // 方向転換時に両方のピンへPWMが残らないよう、先に停止する。
  analogWrite(MOTOR_IN[motor], 0);
  analogWrite(MOTOR_SD[motor], 0);

  if (signedPwm > 0) {
    analogWrite(MOTOR_IN[motor], duty);
  } else if (signedPwm < 0) {
    analogWrite(MOTOR_SD[motor], duty);
  }
}

void stopAllMotors() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    writeMotorRaw(i, 0);
  }
  motorIsRunning = false;
  Serial.println("STOP");
}

void startSingleMotor(uint8_t motor, int pwm, const char *description) {
  stopAllMotors();
  Serial.print("START: ");
  Serial.println(description);
  writeMotorRaw(motor, pwm);
  motorStartedMs = millis();
  motorIsRunning = true;
}

void startAllPositive() {
  stopAllMotors();
  Serial.println("START: all motors in motor_auto_test direction");
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    writeMotorRaw(i, MOTOR_PWM);
  }
  motorStartedMs = millis();
  motorIsRunning = true;
}

void printMenu() {
  Serial.println();
  Serial.println("===== Motor direction test =====");
  Serial.println("1: left-front +  (GPIO 6 PWM,  7 LOW)");
  Serial.println("2: left-front -  (GPIO 6 LOW,  7 PWM)");
  Serial.println("3: rear +        (GPIO 15 PWM, 16 LOW)");
  Serial.println("4: rear -        (GPIO 15 LOW, 16 PWM)");
  Serial.println("5: right-front + (GPIO 18 PWM, 17 LOW)");
  Serial.println("6: right-front - (GPIO 18 LOW, 17 PWM)");
  Serial.println("r: all three +");
  Serial.println("0: stop immediately");
  Serial.println("m: print this menu");
  Serial.println("Each run stops automatically after 3000 ms.");
}

void setup() {
  Serial.begin(115200);
  delay(300);

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    pinMode(MOTOR_IN[i], OUTPUT);
    pinMode(MOTOR_SD[i], OUTPUT);
    digitalWrite(MOTOR_IN[i], LOW);
    digitalWrite(MOTOR_SD[i], LOW);
  }

  stopAllMotors();
  printMenu();
}

void loop() {
  if (motorIsRunning && millis() - motorStartedMs >= PHASE_MS) {
    stopAllMotors();
  }

  while (Serial.available() > 0) {
    const char command = Serial.read();
    switch (command) {
      case '1': startSingleMotor(0,  MOTOR_PWM, "left-front +"); break;
      case '2': startSingleMotor(0, -MOTOR_PWM, "left-front -"); break;
      case '3': startSingleMotor(1,  MOTOR_PWM, "rear +"); break;
      case '4': startSingleMotor(1, -MOTOR_PWM, "rear -"); break;
      case '5': startSingleMotor(2,  MOTOR_PWM, "right-front +"); break;
      case '6': startSingleMotor(2, -MOTOR_PWM, "right-front -"); break;
      case 'r': case 'R': startAllPositive(); break;
      case '0': stopAllMotors(); break;
      case 'm': case 'M': printMenu(); break;
      case '\r': case '\n': case ' ': break;
      default:
        Serial.print("Unknown command: ");
        Serial.println(command);
        break;
    }
  }
}
