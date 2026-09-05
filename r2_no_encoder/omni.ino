#include "robot_types.h"

// 各モーターをIN/SDの2入力で正逆転させるHブリッジ方式。
// 車輪順: 0=左前、1=後ろ、2=右前
constexpr int MOTOR_IN[config::wheel_count] = {6, 15, 18};
constexpr int MOTOR_SD[config::wheel_count] = {7, 16, 17};
// 実機確認結果: 3輪ともIN側駆動が正方向。
constexpr int MOTOR_DIRECTION_SIGN[config::wheel_count] = {1, 1, 1};
constexpr uint32_t MOTOR_PWM_FREQ_HZ = 20000;
constexpr uint8_t MOTOR_PWM_BITS = 8;
constexpr int MOTOR_PWM_MAX = (1 << MOTOR_PWM_BITS) - 1;
constexpr int MOTOR_PWM = 50;  // 速度引き上げ後の上限。0～255

// Arduino-ESP32 2.xではPWMチャンネル番号が必要。
constexpr uint8_t MOTOR_IN_CHANNEL[config::wheel_count] = {0, 2, 4};
constexpr uint8_t MOTOR_SD_CHANNEL[config::wheel_count] = {1, 3, 5};

// 逆運動学: 機体速度 -> 各車輪の接線速度
WheelSpeeds inverse(const BodyTwist &t) {
  WheelSpeeds ws;
  for (uint8_t i = 0; i < config::wheel_count; ++i) {
    const float alpha = config::mount_offset[i];
    ws.v_wheel[i] = -sinf(alpha) * t.vx + cosf(alpha) * t.vy
                  + config::mount_radius_m * t.w;
  }
  return ws;
}

// 順運動学: 車輪速度 -> 機体速度。速度[m/s]・移動量[m]の双方で利用可。
bool omni_Forward(const WheelSpeeds &wheel, BodyTwist &body) {
  float a[3][4];
  for (uint8_t row = 0; row < 3; ++row) {
    const float alpha = config::mount_offset[row];
    a[row][0] = -sinf(alpha); a[row][1] = cosf(alpha);
    a[row][2] = config::mount_radius_m; a[row][3] = wheel.v_wheel[row];
  }
  for (uint8_t col = 0; col < 3; ++col) {
    uint8_t pivot = col;
    for (uint8_t row = col + 1; row < 3; ++row)
      if (fabsf(a[row][col]) > fabsf(a[pivot][col])) pivot = row;
    if (fabsf(a[pivot][col]) < 1.0e-6f) return false;
    if (pivot != col) for (uint8_t j = col; j < 4; ++j) {
      const float t = a[col][j]; a[col][j] = a[pivot][j]; a[pivot][j] = t;
    }
    const float divisor = a[col][col];
    for (uint8_t j = col; j < 4; ++j) a[col][j] /= divisor;
    for (uint8_t row = 0; row < 3; ++row) {
      if (row == col) continue;
      const float factor = a[row][col];
      for (uint8_t j = col; j < 4; ++j) a[row][j] -= factor * a[col][j];
    }
  }
  body.vx = a[0][3]; body.vy = a[1][3]; body.w = a[2][3];
  return true;
}

void omni_WritePwm(uint8_t wheel, bool inSide, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(inSide ? MOTOR_IN[wheel] : MOTOR_SD[wheel], duty);
#else
  ledcWrite(inSide ? MOTOR_IN_CHANNEL[wheel] : MOTOR_SD_CHANNEL[wheel], duty);
#endif
}

void omni_WriteMotor(uint8_t wheel, float signedPwm) {
  if (wheel >= config::wheel_count) return;
  signedPwm *= MOTOR_DIRECTION_SIGN[wheel];
  signedPwm = constrain(signedPwm, -static_cast<float>(MOTOR_PWM), static_cast<float>(MOTOR_PWM));
  const uint32_t duty = static_cast<uint32_t>(fabsf(signedPwm));

  if (signedPwm > 0.0f) {
    omni_WritePwm(wheel, true, duty);
    omni_WritePwm(wheel, false, 0);
  } else if (signedPwm < 0.0f) {
    omni_WritePwm(wheel, true, 0);
    omni_WritePwm(wheel, false, duty);
  } else {
    omni_WritePwm(wheel, true, 0);
    omni_WritePwm(wheel, false, 0);
  }
}

void omni_Stop() { for (uint8_t i = 0; i < config::wheel_count; ++i) omni_WriteMotor(i, 0.0f); }

void omni_Init() {
  for (uint8_t i = 0; i < config::wheel_count; ++i) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(MOTOR_IN[i], MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
    ledcAttach(MOTOR_SD[i], MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
#else
    ledcSetup(MOTOR_IN_CHANNEL[i], MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
    ledcSetup(MOTOR_SD_CHANNEL[i], MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
    ledcAttachPin(MOTOR_IN[i], MOTOR_IN_CHANNEL[i]);
    ledcAttachPin(MOTOR_SD[i], MOTOR_SD_CHANNEL[i]);
#endif
  }
  omni_Stop();
}

// 2個の測定輪は各駆動輪に対応しないため、駆動輪ごとの速度PIDには使わない。
// 位置の閉ループ制御はmission.inoで行い、ここでは車輪速度指令を暫定PWMへ変換する。
constexpr float DRIVE_PWM_PER_MPS = 200.0f;
constexpr float PID_PWM_SLEW_PER_SECOND = 240.0f;
float previousMotorOutput[config::wheel_count] = {0.0f, 0.0f, 0.0f};
WheelSpeeds targetWheelSpeed;
BodyTwist targetBodyVelocity = {0.0f, 0.0f, 0.0f};

void omni_ResetPid() {
  for (uint8_t i = 0; i < config::wheel_count; ++i) {
    previousMotorOutput[i] = 0.0f;
  }
}

void omni_SetBodyVelocity(float vx_mps, float vy_mps, float w_radps) {
  targetBodyVelocity = {vx_mps, vy_mps, w_radps}; targetWheelSpeed = inverse(targetBodyVelocity);
  float largest = 0.0f;
  for (uint8_t i = 0; i < config::wheel_count; ++i) largest = max(largest, fabsf(targetWheelSpeed.v_wheel[i]));
  if (largest > config::max_wheel_speed_mps) {
    const float scale = config::max_wheel_speed_mps / largest;
    for (uint8_t i = 0; i < config::wheel_count; ++i) targetWheelSpeed.v_wheel[i] *= scale;
  }
}

void omni_SetFieldVelocity(float vx_mps, float vy_mps, float w_radps, float heading_rad) {
  const float c = cosf(heading_rad), s = sinf(heading_rad);
  omni_SetBodyVelocity(c * vx_mps + s * vy_mps, -s * vx_mps + c * vy_mps, w_radps);
}

void omni_UpdateOutput(float dt_s) {
  if (dt_s <= 0.0f) return;
  for (uint8_t i = 0; i < config::wheel_count; ++i) {
    const float target = targetWheelSpeed.v_wheel[i];
    if (fabsf(target) < 0.001f) {
      previousMotorOutput[i] = 0.0f;
      omni_WriteMotor(i, 0.0f);
      continue;
    }

    float output = DRIVE_PWM_PER_MPS * target;
    output = constrain(output, -static_cast<float>(MOTOR_PWM), static_cast<float>(MOTOR_PWM));

    // 指令PWMを徐々に変化させ、急発進と機体の姿勢崩れを抑える。
    const float maxOutputChange = PID_PWM_SLEW_PER_SECOND * dt_s;
    output = constrain(
      output,
      previousMotorOutput[i] - maxOutputChange,
      previousMotorOutput[i] + maxOutputChange
    );

    previousMotorOutput[i] = output;
    omni_WriteMotor(i, output);
  }
}
