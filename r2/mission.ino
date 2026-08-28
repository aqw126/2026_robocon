#include "robot_types.h"
#include "mission_support.h"

// ============================================================
// 水やりミッションの状態遷移
// WAIT -> 倉庫A/C -> じょうろ取得 -> 庭 -> 散水 -> 反対倉庫 -> DONE
// フィールド座標・タイミングは仮値。実コートで測定して置き換えること。
// ============================================================
enum MissionState {
  WAIT_START,
  GO_WAREHOUSE_A,
  GO_WAREHOUSE_C,
  GET_CAN,
  GO_GARDEN,
  WATERING,
  GO_OTHER_WAREHOUSE,
  DONE,
  ERROR_STOP,
};

enum FirstWarehouse { WAREHOUSE_A, WAREHOUSE_C };
constexpr FirstWarehouse FIRST_WAREHOUSE = WAREHOUSE_C;

// すべてフィールド座標[m]。起動地点=(0, 0)、前方=+x、左方=+y。
constexpr float WAREHOUSE_A_X_M = 1.20f;
constexpr float WAREHOUSE_A_Y_M = 0.65f;
constexpr float WAREHOUSE_C_X_M = 1.20f;
constexpr float WAREHOUSE_C_Y_M = -0.65f;
constexpr float GARDEN_X_M = 2.20f;
constexpr float GARDEN_Y_M = 0.00f;

constexpr int START_SWITCH_PIN = -1;  // 実ピンを設定。未接続時は -1
constexpr bool START_SWITCH_ACTIVE_LOW = true;
constexpr bool AUTO_START_FOR_TEST = false;  // テスト時だけtrue。本番はfalse。
constexpr uint32_t MOVE_TIMEOUT_MS = 12000;
constexpr float MISSION_POSITION_KP = 0.80f;
constexpr float MISSION_MAX_SPEED_MPS = 0.30f;
constexpr float MISSION_ARRIVAL_RADIUS_M = 0.05f;
constexpr float MISSION_HEADING_RAD = 0.0f;
constexpr float MISSION_HEADING_KP = 1.5f;

// ToFを有効化し、正面の壁のx座標を測定した場合だけ有効にする。
constexpr bool TOF_POSITION_CORRECTION_ENABLED = false;
constexpr float TOF_REFERENCE_WALL_X_M = 2.50f;
constexpr float TOF_SENSOR_FORWARD_OFFSET_M = 0.05f;

MissionState missionState = WAIT_START;
uint32_t missionStateEnteredMs = 0;
FirstWarehouse firstWarehouse = FIRST_WAREHOUSE;

const char *mission_StateName(MissionState state) {
  switch (state) {
    case WAIT_START: return "WAIT_START";
    case GO_WAREHOUSE_A: return "GO_WAREHOUSE_A";
    case GO_WAREHOUSE_C: return "GO_WAREHOUSE_C";
    case GET_CAN: return "GET_CAN";
    case GO_GARDEN: return "GO_GARDEN";
    case WATERING: return "WATERING";
    case GO_OTHER_WAREHOUSE: return "GO_OTHER_WAREHOUSE";
    case DONE: return "DONE";
    case ERROR_STOP: return "ERROR_STOP";
  }
  return "UNKNOWN";
}

void mission_SetState(MissionState nextState) {
  missionState = nextState;
  missionStateEnteredMs = millis();
  Serial.print("Mission state: ");
  Serial.println(mission_StateName(nextState));

  if (nextState == GET_CAN) mechanism_StartGetCan();
  if (nextState == WATERING) mechanism_StartWater();
  if (nextState == DONE || nextState == ERROR_STOP) {
    omni_SetBodyVelocity(0.0f, 0.0f, 0.0f);
    mechanism_Stop();
  }
}

bool mission_StartRequested() {
  if (AUTO_START_FOR_TEST) return true;
  if (START_SWITCH_PIN < 0) return false;
  const bool level = digitalRead(START_SWITCH_PIN) == HIGH;
  return START_SWITCH_ACTIVE_LOW ? !level : level;
}

float mission_NormalizeAngle(float angle) {
  while (angle > kPi) angle -= 2.0f * kPi;
  while (angle <= -kPi) angle += 2.0f * kPi;
  return angle;
}

void mission_ApplyTofCorrection() {
  if (!TOF_POSITION_CORRECTION_ENABLED) return;
  ToFReading reading;
  if (!tof_GetFrontDistance(reading)) return;

  // ToFが機体正面を向き、x=一定の壁を測るという仮定でx座標を補正する。
  const float headingCos = cosf(robotPose.heading_rad);
  if (fabsf(headingCos) < 0.7f) return;
  robotPose.x_m = TOF_REFERENCE_WALL_X_M
                 - (reading.distance_m + TOF_SENSOR_FORWARD_OFFSET_M) * headingCos;
}

bool mission_DriveTo(float targetX_m, float targetY_m) {
  mission_ApplyTofCorrection();
  const float dx = targetX_m - robotPose.x_m;
  const float dy = targetY_m - robotPose.y_m;
  const float distance = sqrtf(dx * dx + dy * dy);
  if (distance <= MISSION_ARRIVAL_RADIUS_M) {
    omni_SetBodyVelocity(0.0f, 0.0f, 0.0f);
    return true;
  }

  float vxField = MISSION_POSITION_KP * dx;
  float vyField = MISSION_POSITION_KP * dy;
  const float speed = sqrtf(vxField * vxField + vyField * vyField);
  if (speed > MISSION_MAX_SPEED_MPS) {
    const float scale = MISSION_MAX_SPEED_MPS / speed;
    vxField *= scale;
    vyField *= scale;
  }
  const float headingError = mission_NormalizeAngle(MISSION_HEADING_RAD - robotPose.heading_rad);
  omni_SetFieldVelocity(vxField, vyField, MISSION_HEADING_KP * headingError, robotPose.heading_rad);
  return false;
}

bool mission_MoveTimedOut() { return millis() - missionStateEnteredMs > MOVE_TIMEOUT_MS; }

void mission_Init() {
  if (START_SWITCH_PIN >= 0) pinMode(START_SWITCH_PIN, INPUT_PULLUP);
  tof_Init();
  mechanism_Init();
  mission_SetState(WAIT_START);
}

void mission_Update() {
  switch (missionState) {
    case WAIT_START:
      omni_SetBodyVelocity(0.0f, 0.0f, 0.0f);
      if (mission_StartRequested()) {
        gyro_ResetHeading();
        odometry_Reset(0.0f, 0.0f);
        mission_SetState(firstWarehouse == WAREHOUSE_A ? GO_WAREHOUSE_A : GO_WAREHOUSE_C);
      }
      break;

    case GO_WAREHOUSE_A:
      if (mission_DriveTo(WAREHOUSE_A_X_M, WAREHOUSE_A_Y_M)) mission_SetState(GET_CAN);
      else if (mission_MoveTimedOut()) mission_SetState(ERROR_STOP);
      break;

    case GO_WAREHOUSE_C:
      if (mission_DriveTo(WAREHOUSE_C_X_M, WAREHOUSE_C_Y_M)) mission_SetState(GET_CAN);
      else if (mission_MoveTimedOut()) mission_SetState(ERROR_STOP);
      break;

    case GET_CAN:
      if (mechanism_IsActionFinished()) mission_SetState(GO_GARDEN);
      break;

    case GO_GARDEN:
      if (mission_DriveTo(GARDEN_X_M, GARDEN_Y_M)) mission_SetState(WATERING);
      else if (mission_MoveTimedOut()) mission_SetState(ERROR_STOP);
      break;

    case WATERING:
      if (mechanism_IsActionFinished()) mission_SetState(GO_OTHER_WAREHOUSE);
      break;

    case GO_OTHER_WAREHOUSE:
      if (firstWarehouse == WAREHOUSE_A) {
        if (mission_DriveTo(WAREHOUSE_C_X_M, WAREHOUSE_C_Y_M)) mission_SetState(DONE);
      } else {
        if (mission_DriveTo(WAREHOUSE_A_X_M, WAREHOUSE_A_Y_M)) mission_SetState(DONE);
      }
      if (missionState == GO_OTHER_WAREHOUSE && mission_MoveTimedOut()) mission_SetState(ERROR_STOP);
      break;

    case DONE:
    case ERROR_STOP:
      omni_SetBodyVelocity(0.0f, 0.0f, 0.0f);
      break;
  }
}
