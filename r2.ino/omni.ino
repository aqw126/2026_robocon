#include <Arduino.h>
#include <math.h>

// 座標系: x=前方, y=左方, 角速度w=反時計回り。単位: m, s, rad
// 車輪順: 0=右下(π/3), 1=左下(2π/3), 2=正面(π)
constexpr float kPi = 3.14159265358979323846f;
namespace config {
constexpr uint8_t wheel_count = 3;
constexpr float wheel_radius_m = 0.050f;       // 実測値に変更
constexpr float mount_radius_m = 0.150f;       // 機体中心から車輪まで
constexpr float mount_offset[wheel_count] = {kPi / 3.0f, 2.0f * kPi / 3.0f, kPi};
constexpr float max_wheel_speed_mps = 1.0f;
}

// IN: 方向、PWM: 速度入力を想定。実配線に合わせて確認・変更すること。
constexpr int MOTOR_IN[config::wheel_count] = {4, 6, 15};
constexpr int MOTOR_PWM[config::wheel_count] = {5, 7, 16};
constexpr int MOTOR_DIRECTION_SIGN[config::wheel_count] = {1, 1, 1};
constexpr uint32_t MOTOR_PWM_FREQ_HZ = 20000;
constexpr uint8_t MOTOR_PWM_BITS = 8;
constexpr int MOTOR_PWM_MAX = (1 << MOTOR_PWM_BITS) - 1;

struct BodyTwist { float vx, vy, w; };
struct WheelSpeeds { float v_wheel[config::wheel_count] = {}; };
struct Pose2D { float x_m = 0.0f, y_m = 0.0f, heading_rad = 0.0f; };

extern float wheelSpeedMps[config::wheel_count];

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

void omni_WriteMotor(uint8_t wheel, float signedPwm) {
  if (wheel >= config::wheel_count) return;
  signedPwm *= MOTOR_DIRECTION_SIGN[wheel];
  signedPwm = constrain(signedPwm, -static_cast<float>(MOTOR_PWM_MAX), static_cast<float>(MOTOR_PWM_MAX));
  digitalWrite(MOTOR_IN[wheel], signedPwm >= 0.0f ? HIGH : LOW);
  const uint32_t duty = static_cast<uint32_t>(fabsf(signedPwm));
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_PWM[wheel], duty);
#else
  ledcWrite(wheel, duty);
#endif
}

void omni_Stop() { for (uint8_t i = 0; i < config::wheel_count; ++i) omni_WriteMotor(i, 0.0f); }

void omni_Init() {
  for (uint8_t i = 0; i < config::wheel_count; ++i) {
    pinMode(MOTOR_IN[i], OUTPUT);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(MOTOR_PWM[i], MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
#else
    ledcSetup(i, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS); ledcAttachPin(MOTOR_PWM[i], i);
#endif
  }
  omni_Stop();
}

// PID出力はPWM duty[-255,255]。実機で必ず調整すること。
struct WheelPID { float kp = 160.0f, ki = 20.0f, kd = 0.0f, integral = 0.0f, previousError = 0.0f; };
WheelPID wheelPid[config::wheel_count];
WheelSpeeds targetWheelSpeed;
BodyTwist targetBodyVelocity = {0.0f, 0.0f, 0.0f};

void omni_ResetPid() {
  for (uint8_t i = 0; i < config::wheel_count; ++i) wheelPid[i].integral = wheelPid[i].previousError = 0.0f;
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

void omni_UpdatePid(float dt_s) {
  if (dt_s <= 0.0f) return;
  for (uint8_t i = 0; i < config::wheel_count; ++i) {
    const float target = targetWheelSpeed.v_wheel[i];
    if (fabsf(target) < 0.001f) { wheelPid[i].integral = wheelPid[i].previousError = 0.0f; omni_WriteMotor(i, 0.0f); continue; }
    const float error = target - wheelSpeedMps[i];
    wheelPid[i].integral = constrain(wheelPid[i].integral + error * dt_s, -2.0f, 2.0f);
    const float derivative = (error - wheelPid[i].previousError) / dt_s;
    float output = wheelPid[i].kp * error + wheelPid[i].ki * wheelPid[i].integral + wheelPid[i].kd * derivative;
    output = constrain(output, -static_cast<float>(MOTOR_PWM_MAX), static_cast<float>(MOTOR_PWM_MAX));
    wheelPid[i].previousError = error; omni_WriteMotor(i, output);
  }
}
