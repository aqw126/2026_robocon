#include "robot_types.h"
#include "mission_support.h"

// ============================================================
// フィールド往復ミッション
// 座標系: 図面左下を(0,0)[m]とし、短辺方向を+x、長辺方向を+yとする。
// スタート時はR2スタートゾーン中心で、ロボット正面を-y（倉庫C側）へ向ける。
//
// ルート:
// 初期動作を含め、倉庫C -> 庭 -> 倉庫A -> 庭 -> 倉庫C ... を繰り返す。
// ============================================================

enum MissionState {
  INITIAL_GO_WAREHOUSE_C,
  INITIAL_ARM_ACTION,
  INITIAL_TURN_FROM_C,
  INITIAL_GO_GARDEN_1,
  INITIAL_BACK_TO_PROMENADE_1,
  INITIAL_SHIFT_TO_A_X,
  INITIAL_TURN_TO_A,
  INITIAL_GO_WAREHOUSE_A,
  INITIAL_TURN_FROM_A,
  INITIAL_GO_GARDEN_2,
  INITIAL_BACK_TO_PROMENADE_2,
  LOOP_SHIFT_TO_C_X,
  LOOP_GO_WAREHOUSE_C,
  LOOP_GO_GARDEN_FROM_C,
  LOOP_BACK_TO_PROMENADE_FOR_A,
  LOOP_SHIFT_TO_A_X,
  LOOP_GO_WAREHOUSE_A,
  LOOP_GO_GARDEN_FROM_A,
  LOOP_BACK_TO_PROMENADE_FOR_C,
  DONE,
  ERROR_STOP,
};

// 添付図の2400mm x 4500mmフィールドから読み取った座標。
// 図面寸法に±5%の公差があるため、実コートで最終確認すること。単位はm。
constexpr float FIELD_X_SIZE_M = 2.400f;
constexpr float FIELD_Y_SIZE_M = 4.500f;
constexpr float R2_START_X_M = 0.250f;
constexpr float R2_START_Y_M = 2.050f;
constexpr float GARDEN_X_M = 1.200f;
constexpr float GARDEN_Y_M = 3.900f;
constexpr float WAREHOUSE_A_X_M = 2.025f;
constexpr float WAREHOUSE_A_Y_M = 0.500f;
constexpr float WAREHOUSE_C_X_M = 0.375f;
constexpr float WAREHOUSE_C_Y_M = 1.400f;

// 遊歩道は長辺方向1.8～3.3mの間として、そのY中央を使用する。
constexpr float PROMENADE_CENTER_Y_M = (1.800f + 3.300f) / 2.0f;

// 「ゾーンの奥」は、外径500mmの機体が奥壁に触れる中心位置とする。
// 倉庫Cの奥壁Y=1.0m、倉庫Aの奥壁Y=0.0m、庭の奥壁Y=4.5m。
constexpr float ROBOT_OUTER_DIAMETER_M = 0.500f;
constexpr float ROBOT_OUTER_RADIUS_M = ROBOT_OUTER_DIAMETER_M / 2.0f;
constexpr float WAREHOUSE_C_BACK_WALL_Y_M = 1.000f;
constexpr float WAREHOUSE_A_BACK_WALL_Y_M = 0.000f;
constexpr float GARDEN_BACK_WALL_Y_M = FIELD_Y_SIZE_M;
constexpr float INITIAL_WAREHOUSE_C_DEEP_Y_M = WAREHOUSE_C_BACK_WALL_Y_M + ROBOT_OUTER_RADIUS_M;
constexpr float INITIAL_WAREHOUSE_A_DEEP_Y_M = WAREHOUSE_A_BACK_WALL_Y_M + ROBOT_OUTER_RADIUS_M;
constexpr float INITIAL_GARDEN_DEEP_Y_M = GARDEN_BACK_WALL_Y_M - ROBOT_OUTER_RADIUS_M;

constexpr float HEADING_POSITIVE_Y_RAD = kPi / 2.0f;
constexpr float HEADING_NEGATIVE_Y_RAD = -kPi / 2.0f;
constexpr float R2_START_HEADING_RAD = HEADING_NEGATIVE_Y_RAD;

// 往復動作・到着判定
constexpr bool LOOP_A_AND_C = true;
constexpr uint32_t MOVE_TIMEOUT_MS = 45000;
constexpr uint32_t TARGET_DWELL_MS = 300;
constexpr float MISSION_POSITION_KP = 0.80f;
constexpr float MISSION_X_MAX_SPEED_MPS = 0.20f;
constexpr float MISSION_ENTRY_MAX_SPEED_MPS = 0.15f;
constexpr float MISSION_X_TOLERANCE_M = 0.05f;
constexpr float MISSION_Y_TOLERANCE_M = 0.08f;
constexpr float WALL_TOUCH_TARGET_TOLERANCE_M = 0.02f;
constexpr float MISSION_HEADING_TOLERANCE_RAD = 5.0f * kPi / 180.0f;
constexpr float MISSION_X_ALIGNMENT_HEADING_RAD = 0.0f;
constexpr float MISSION_HEADING_KP = 1.5f;
constexpr float MISSION_MAX_TURN_RATE_RADPS = 0.80f;

MissionState missionState = INITIAL_GO_WAREHOUSE_C;
uint32_t missionStateEnteredMs = 0;
uint32_t targetArrivalSinceMs = 0;

const char *mission_StateName(uint8_t state) {
  switch (state) {
    case INITIAL_GO_WAREHOUSE_C: return "INITIAL_GO_WAREHOUSE_C";
    case INITIAL_ARM_ACTION: return "INITIAL_ARM_ACTION";
    case INITIAL_TURN_FROM_C: return "INITIAL_TURN_FROM_C";
    case INITIAL_GO_GARDEN_1: return "INITIAL_GO_GARDEN_1";
    case INITIAL_BACK_TO_PROMENADE_1: return "INITIAL_BACK_TO_PROMENADE_1";
    case INITIAL_SHIFT_TO_A_X: return "INITIAL_SHIFT_TO_A_X";
    case INITIAL_TURN_TO_A: return "INITIAL_TURN_TO_A";
    case INITIAL_GO_WAREHOUSE_A: return "INITIAL_GO_WAREHOUSE_A";
    case INITIAL_TURN_FROM_A: return "INITIAL_TURN_FROM_A";
    case INITIAL_GO_GARDEN_2: return "INITIAL_GO_GARDEN_2";
    case INITIAL_BACK_TO_PROMENADE_2: return "INITIAL_BACK_TO_PROMENADE_2";
    case LOOP_SHIFT_TO_C_X: return "LOOP_SHIFT_TO_C_X";
    case LOOP_GO_WAREHOUSE_C: return "LOOP_GO_WAREHOUSE_C";
    case LOOP_GO_GARDEN_FROM_C: return "LOOP_GO_GARDEN_FROM_C";
    case LOOP_BACK_TO_PROMENADE_FOR_A: return "LOOP_BACK_TO_PROMENADE_FOR_A";
    case LOOP_SHIFT_TO_A_X: return "LOOP_SHIFT_TO_A_X";
    case LOOP_GO_WAREHOUSE_A: return "LOOP_GO_WAREHOUSE_A";
    case LOOP_GO_GARDEN_FROM_A: return "LOOP_GO_GARDEN_FROM_A";
    case LOOP_BACK_TO_PROMENADE_FOR_C: return "LOOP_BACK_TO_PROMENADE_FOR_C";
    case DONE: return "DONE";
    case ERROR_STOP: return "ERROR_STOP";
  }
  return "UNKNOWN";
}

void mission_SetState(uint8_t nextState) {
  missionState = static_cast<MissionState>(nextState);
  missionStateEnteredMs = millis();
  targetArrivalSinceMs = 0;
  Serial.print("Mission state: ");
  Serial.println(mission_StateName(missionState));

  if (missionState == INITIAL_ARM_ACTION) {
    // アーム実装後もmission側の状態遷移を変えずに使える受け口。
    mechanism_StartGetCan();
  }

  if (missionState == DONE || missionState == ERROR_STOP) {
    omni_SetBodyVelocity(0.0f, 0.0f, 0.0f);
    mechanism_Stop();
  }
}

float mission_NormalizeAngle(float angle) {
  while (angle > kPi) angle -= 2.0f * kPi;
  while (angle <= -kPi) angle += 2.0f * kPi;
  return angle;
}

float mission_HeadingError(float targetHeadingRad) {
  float error = mission_NormalizeAngle(targetHeadingRad - robotPose.heading_rad);

  // 180度付近はBNO055の微小変動で回転方向が交互に反転しやすい。
  // R2e.inoで確認できた時計回り（負）へ固定して回転を開始する。
  constexpr float AMBIGUOUS_180_RANGE_RAD = 10.0f * kPi / 180.0f;
  if (fabsf(fabsf(error) - kPi) <= AMBIGUOUS_180_RANGE_RAD) {
    error = -fabsf(error);
  }
  return error;
}

bool mission_HoldForDwell() {
  if (targetArrivalSinceMs == 0) targetArrivalSinceMs = millis();
  return millis() - targetArrivalSinceMs >= TARGET_DWELL_MS;
}

void mission_CommandFieldVelocity(float vxField, float vyField, float targetHeadingRad) {
  const float headingError = mission_HeadingError(targetHeadingRad);
  const float turnRate = fabsf(headingError) <= MISSION_HEADING_TOLERANCE_RAD
    ? 0.0f
    : constrain(
        MISSION_HEADING_KP * headingError,
        -MISSION_MAX_TURN_RATE_RADPS,
        MISSION_MAX_TURN_RATE_RADPS
      );
  omni_SetFieldVelocity(vxField, vyField, turnRate, robotPose.heading_rad);
}

bool mission_TurnTo(float targetHeadingRad) {
  const float headingError = mission_HeadingError(targetHeadingRad);
  if (fabsf(headingError) <= MISSION_HEADING_TOLERANCE_RAD) {
    mission_CommandFieldVelocity(0.0f, 0.0f, targetHeadingRad);
    return mission_HoldForDwell();
  }

  targetArrivalSinceMs = 0;
  mission_CommandFieldVelocity(0.0f, 0.0f, targetHeadingRad);
  return false;
}

// 姿勢を保ち、フィールドY方向だけへ直進または後退する。
bool mission_DriveAlongY(float targetY_m, float targetHeadingRad, float positionTolerance_m) {
  const float dy = targetY_m - robotPose.y_m;
  if (fabsf(dy) <= positionTolerance_m) {
    mission_CommandFieldVelocity(0.0f, 0.0f, targetHeadingRad);
    return mission_HoldForDwell();
  }

  targetArrivalSinceMs = 0;
  const float vyField = constrain(
    MISSION_POSITION_KP * dy,
    -MISSION_ENTRY_MAX_SPEED_MPS,
    MISSION_ENTRY_MAX_SPEED_MPS
  );
  mission_CommandFieldVelocity(0.0f, vyField, targetHeadingRad);
  return false;
}

// 姿勢を保ったままフィールドX方向だけへ移動する。
bool mission_ShiftAlongX(float targetX_m, float targetHeadingRad) {
  const float dx = targetX_m - robotPose.x_m;
  if (fabsf(dx) <= MISSION_X_TOLERANCE_M) {
    mission_CommandFieldVelocity(0.0f, 0.0f, targetHeadingRad);
    return mission_HoldForDwell();
  }

  targetArrivalSinceMs = 0;
  const float vxField = constrain(
    MISSION_POSITION_KP * dx,
    -MISSION_X_MAX_SPEED_MPS,
    MISSION_X_MAX_SPEED_MPS
  );
  mission_CommandFieldVelocity(vxField, 0.0f, targetHeadingRad);
  return false;
}

bool mission_MoveTimedOut() {
  return millis() - missionStateEnteredMs > MOVE_TIMEOUT_MS;
}

void mission_Init() {
  mechanism_Init();

  // 現在の実機姿勢を「R2中心から倉庫C中心を向く方位」としてBNO055へ対応付ける。
  gyro_ResetHeading(R2_START_HEADING_RAD);
  odometry_Reset(R2_START_X_M, R2_START_Y_M);
  mission_SetState(INITIAL_GO_WAREHOUSE_C);
}

void mission_Update() {
  switch (missionState) {
    // 1. 倉庫Cの奥まで、正面を向いたまま直進する。
    case INITIAL_GO_WAREHOUSE_C:
      if (mission_DriveAlongY(
            INITIAL_WAREHOUSE_C_DEEP_Y_M,
            HEADING_NEGATIVE_Y_RAD,
            WALL_TOUCH_TARGET_TOLERANCE_M)) {
        mission_SetState(INITIAL_ARM_ACTION);
      } else if (mission_MoveTimedOut()) mission_SetState(ERROR_STOP);
      break;

    // 2. アーム動作。mechanism.inoの実機出力は未設定。
    case INITIAL_ARM_ACTION:
      omni_SetBodyVelocity(0.0f, 0.0f, 0.0f);
      if (mechanism_IsActionFinished()) mission_SetState(INITIAL_TURN_FROM_C);
      break;

    // 3. 倉庫C内で180度回転し、+Y方向を向く。
    case INITIAL_TURN_FROM_C:
      if (mission_TurnTo(HEADING_POSITIVE_Y_RAD)) mission_SetState(INITIAL_GO_GARDEN_1);
      else if (mission_MoveTimedOut()) mission_SetState(ERROR_STOP);
      break;

    // 4. Xを変えず、庭ゾーンの奥まで直進する。
    case INITIAL_GO_GARDEN_1:
      if (mission_DriveAlongY(
            INITIAL_GARDEN_DEEP_Y_M,
            HEADING_POSITIVE_Y_RAD,
            WALL_TOUCH_TARGET_TOLERANCE_M)) {
        mission_SetState(INITIAL_BACK_TO_PROMENADE_1);
      } else if (mission_MoveTimedOut()) mission_SetState(ERROR_STOP);
      break;

    // 5. +Yを向いたまま、遊歩道のY中央まで後退する。
    case INITIAL_BACK_TO_PROMENADE_1:
      if (mission_DriveAlongY(
            PROMENADE_CENTER_Y_M,
            HEADING_POSITIVE_Y_RAD,
            MISSION_Y_TOLERANCE_M)) {
        mission_SetState(INITIAL_SHIFT_TO_A_X);
      } else if (mission_MoveTimedOut()) mission_SetState(ERROR_STOP);
      break;

    // 6. 向きを保ったまま、倉庫AのX中心まで真横に移動する。
    case INITIAL_SHIFT_TO_A_X:
      if (mission_ShiftAlongX(WAREHOUSE_A_X_M, HEADING_POSITIVE_Y_RAD)) {
        mission_SetState(INITIAL_TURN_TO_A);
      } else if (mission_MoveTimedOut()) mission_SetState(ERROR_STOP);
      break;

    // 7. 180度回転して、倉庫A側（-Y）を向く。
    case INITIAL_TURN_TO_A:
      if (mission_TurnTo(HEADING_NEGATIVE_Y_RAD)) mission_SetState(INITIAL_GO_WAREHOUSE_A);
      else if (mission_MoveTimedOut()) mission_SetState(ERROR_STOP);
      break;

    // 8. 倉庫Aの奥まで直進する。
    case INITIAL_GO_WAREHOUSE_A:
      if (mission_DriveAlongY(
            INITIAL_WAREHOUSE_A_DEEP_Y_M,
            HEADING_NEGATIVE_Y_RAD,
            WALL_TOUCH_TARGET_TOLERANCE_M)) {
        mission_SetState(INITIAL_TURN_FROM_A);
      } else if (mission_MoveTimedOut()) mission_SetState(ERROR_STOP);
      break;

    // 9. 180度回転して、庭側（+Y）を向く。
    case INITIAL_TURN_FROM_A:
      if (mission_TurnTo(HEADING_POSITIVE_Y_RAD)) mission_SetState(INITIAL_GO_GARDEN_2);
      else if (mission_MoveTimedOut()) mission_SetState(ERROR_STOP);
      break;

    // 10. Xを変えず、庭ゾーンの奥まで直進する。
    case INITIAL_GO_GARDEN_2:
      if (mission_DriveAlongY(
            INITIAL_GARDEN_DEEP_Y_M,
            HEADING_POSITIVE_Y_RAD,
            WALL_TOUCH_TARGET_TOLERANCE_M)) {
        mission_SetState(INITIAL_BACK_TO_PROMENADE_2);
      } else if (mission_MoveTimedOut()) mission_SetState(ERROR_STOP);
      break;

    // 11. +Yを向いたまま遊歩道中央まで後退し、倉庫Cへ向かう。
    case INITIAL_BACK_TO_PROMENADE_2:
      if (mission_DriveAlongY(
            PROMENADE_CENTER_Y_M,
            HEADING_POSITIVE_Y_RAD,
            MISSION_Y_TOLERANCE_M)) {
        mission_SetState(LOOP_SHIFT_TO_C_X);
      } else if (mission_MoveTimedOut()) mission_SetState(ERROR_STOP);
      break;

    // ここから往復動作。倉庫内では横移動せず、必ず遊歩道中央でXを合わせる。
    case LOOP_SHIFT_TO_C_X:
      if (mission_ShiftAlongX(WAREHOUSE_C_X_M, HEADING_POSITIVE_Y_RAD)) {
        mission_SetState(LOOP_GO_WAREHOUSE_C);
      } else if (mission_MoveTimedOut()) mission_SetState(ERROR_STOP);
      break;

    // 往復中は+Yを向いたまま、倉庫Cへ後退して進入する。
    case LOOP_GO_WAREHOUSE_C:
      if (mission_DriveAlongY(
            INITIAL_WAREHOUSE_C_DEEP_Y_M,
            HEADING_POSITIVE_Y_RAD,
            WALL_TOUCH_TARGET_TOLERANCE_M)) {
        mission_SetState(LOOP_GO_GARDEN_FROM_C);
      } else if (mission_MoveTimedOut()) mission_SetState(ERROR_STOP);
      break;

    case LOOP_GO_GARDEN_FROM_C:
      if (mission_DriveAlongY(
            INITIAL_GARDEN_DEEP_Y_M,
            HEADING_POSITIVE_Y_RAD,
            WALL_TOUCH_TARGET_TOLERANCE_M)) {
        mission_SetState(LOOP_BACK_TO_PROMENADE_FOR_A);
      } else if (mission_MoveTimedOut()) mission_SetState(ERROR_STOP);
      break;

    case LOOP_BACK_TO_PROMENADE_FOR_A:
      if (mission_DriveAlongY(
            PROMENADE_CENTER_Y_M,
            HEADING_POSITIVE_Y_RAD,
            MISSION_Y_TOLERANCE_M)) {
        mission_SetState(LOOP_SHIFT_TO_A_X);
      } else if (mission_MoveTimedOut()) mission_SetState(ERROR_STOP);
      break;

    case LOOP_SHIFT_TO_A_X:
      if (mission_ShiftAlongX(WAREHOUSE_A_X_M, HEADING_POSITIVE_Y_RAD)) {
        mission_SetState(LOOP_GO_WAREHOUSE_A);
      } else if (mission_MoveTimedOut()) mission_SetState(ERROR_STOP);
      break;

    // 往復中は+Yを向いたまま、倉庫Aへ後退して進入する。
    case LOOP_GO_WAREHOUSE_A:
      if (mission_DriveAlongY(
            INITIAL_WAREHOUSE_A_DEEP_Y_M,
            HEADING_POSITIVE_Y_RAD,
            WALL_TOUCH_TARGET_TOLERANCE_M)) {
        mission_SetState(LOOP_GO_GARDEN_FROM_A);
      } else if (mission_MoveTimedOut()) mission_SetState(ERROR_STOP);
      break;

    case LOOP_GO_GARDEN_FROM_A:
      if (mission_DriveAlongY(
            INITIAL_GARDEN_DEEP_Y_M,
            HEADING_POSITIVE_Y_RAD,
            WALL_TOUCH_TARGET_TOLERANCE_M)) {
        mission_SetState(LOOP_BACK_TO_PROMENADE_FOR_C);
      } else if (mission_MoveTimedOut()) mission_SetState(ERROR_STOP);
      break;

    case LOOP_BACK_TO_PROMENADE_FOR_C:
      if (mission_DriveAlongY(
            PROMENADE_CENTER_Y_M,
            HEADING_POSITIVE_Y_RAD,
            MISSION_Y_TOLERANCE_M)) {
        mission_SetState(LOOP_A_AND_C ? LOOP_SHIFT_TO_C_X : DONE);
      } else if (mission_MoveTimedOut()) mission_SetState(ERROR_STOP);
      break;

    case DONE:
    case ERROR_STOP:
      omni_SetBodyVelocity(0.0f, 0.0f, 0.0f);
      break;
  }
}
