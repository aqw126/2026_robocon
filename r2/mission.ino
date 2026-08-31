#include "robot_types.h"
#include "mission_support.h"

// ============================================================
// フィールド往復ミッション
// 座標系: フィールド左下を (0,0)[m]、右を+x、上を+y。
// スタート時はロボットの前方を +x に合わせること。
//
// ルート:
// R2スタート -> 庭ゾーン -> 倉庫A -> 庭ゾーン -> 倉庫C
//                  -> 庭ゾーン -> 倉庫A ...
// ============================================================

enum MissionState {
  INITIAL_ACTION,
  GO_GARDEN_TO_A,
  GO_WAREHOUSE_A,
  GO_GARDEN_TO_C,
  GO_WAREHOUSE_C,
  DONE,
  ERROR_STOP,
};

// 添付図の4500mm x 2400mmフィールドから読み取った各ゾーンの中心。
// 左下原点。図面寸法に±5%の公差があるため、実コートで最終確認すること。単位はm。
constexpr float FIELD_WIDTH_M = 4.500f;
constexpr float FIELD_HEIGHT_M = 2.400f;
constexpr float R2_START_X_M = 2.050f;
constexpr float R2_START_Y_M = 0.250f;
constexpr float GARDEN_X_M = 3.900f;
constexpr float GARDEN_Y_M = 1.200f;
constexpr float WAREHOUSE_A_X_M = 0.500f;
constexpr float WAREHOUSE_A_Y_M = 2.025f;
constexpr float WAREHOUSE_C_X_M = 1.400f;
constexpr float WAREHOUSE_C_Y_M = 0.375f;

// 電源投入後に最初に行う仮動作: 機体前方へ0.20m/sで3秒間進む。
constexpr float INITIAL_ACTION_VX_MPS = 0.20f;
constexpr uint32_t PHASE_MS = 3000;

// 往復動作・到着判定
constexpr bool LOOP_A_AND_C = true;
constexpr uint32_t MOVE_TIMEOUT_MS = 20000;
constexpr uint32_t TARGET_DWELL_MS = 300;
constexpr float MISSION_POSITION_KP = 0.80f;
constexpr float MISSION_MAX_SPEED_MPS = 0.30f;
constexpr float MISSION_ARRIVAL_RADIUS_M = 0.10f;
constexpr float MISSION_HEADING_RAD = 0.0f;
constexpr float MISSION_HEADING_KP = 1.5f;

// 正面ToFでx=FIELD_WIDTH_Mの壁を測る場合だけ有効化する。
constexpr bool TOF_POSITION_CORRECTION_ENABLED = false;
constexpr float TOF_REFERENCE_WALL_X_M = FIELD_WIDTH_M;
constexpr float TOF_SENSOR_FORWARD_OFFSET_M = 0.05f;

MissionState missionState = INITIAL_ACTION;
uint32_t missionStateEnteredMs = 0;
uint32_t targetArrivalSinceMs = 0;

const char *mission_StateName(MissionState state) {
  switch (state) {
    case INITIAL_ACTION: return "INITIAL_ACTION";
    case GO_GARDEN_TO_A: return "GO_GARDEN_TO_A";
    case GO_WAREHOUSE_A: return "GO_WAREHOUSE_A";
    case GO_GARDEN_TO_C: return "GO_GARDEN_TO_C";
    case GO_WAREHOUSE_C: return "GO_WAREHOUSE_C";
    case DONE: return "DONE";
    case ERROR_STOP: return "ERROR_STOP";
  }
  return "UNKNOWN";
}

void mission_SetState(MissionState nextState) {
  missionState = nextState;
  missionStateEnteredMs = millis();
  targetArrivalSinceMs = 0;
  Serial.print("Mission state: ");
  Serial.println(mission_StateName(nextState));

  if (nextState == DONE || nextState == ERROR_STOP) {
    omni_SetBodyVelocity(0.0f, 0.0f, 0.0f);
    mechanism_Stop();
  }
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

  // ToFが機体正面を向き、フィールド右端（x=4.5m）の壁を測る場合の補正。
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
    if (targetArrivalSinceMs == 0) targetArrivalSinceMs = millis();
    return millis() - targetArrivalSinceMs >= TARGET_DWELL_MS;
  }

  targetArrivalSinceMs = 0;
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

bool mission_MoveTimedOut() {
  return millis() - missionStateEnteredMs > MOVE_TIMEOUT_MS;
}

void mission_Init() {
  tof_Init();
  mechanism_Init();

  // 現在の向きをフィールド+xとし、R2スタート位置から初期動作を始める。
  gyro_ResetHeading();
  odometry_Reset(R2_START_X_M, R2_START_Y_M);
  mission_SetState(INITIAL_ACTION);
}

void mission_Update() {
  switch (missionState) {
    case INITIAL_ACTION:
      omni_SetBodyVelocity(INITIAL_ACTION_VX_MPS, 0.0f, 0.0f);
      if (millis() - missionStateEnteredMs >= PHASE_MS) {
        omni_SetBodyVelocity(0.0f, 0.0f, 0.0f);
        mission_SetState(GO_GARDEN_TO_A);
      }
      break;

    case GO_GARDEN_TO_A:
      if (mission_DriveTo(GARDEN_X_M, GARDEN_Y_M)) mission_SetState(GO_WAREHOUSE_A);
      else if (mission_MoveTimedOut()) mission_SetState(ERROR_STOP);
      break;

    case GO_WAREHOUSE_A:
      if (mission_DriveTo(WAREHOUSE_A_X_M, WAREHOUSE_A_Y_M)) mission_SetState(GO_GARDEN_TO_C);
      else if (mission_MoveTimedOut()) mission_SetState(ERROR_STOP);
      break;

    case GO_GARDEN_TO_C:
      if (mission_DriveTo(GARDEN_X_M, GARDEN_Y_M)) mission_SetState(GO_WAREHOUSE_C);
      else if (mission_MoveTimedOut()) mission_SetState(ERROR_STOP);
      break;

    case GO_WAREHOUSE_C:
      if (mission_DriveTo(WAREHOUSE_C_X_M, WAREHOUSE_C_Y_M)) {
        mission_SetState(LOOP_A_AND_C ? GO_GARDEN_TO_A : DONE);
      } else if (mission_MoveTimedOut()) {
        mission_SetState(ERROR_STOP);
      }
      break;

    case DONE:
    case ERROR_STOP:
      omni_SetBodyVelocity(0.0f, 0.0f, 0.0f);
      break;
  }
}
