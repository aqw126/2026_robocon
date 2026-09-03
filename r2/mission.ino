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
  GO_GARDEN_TO_A,
  GO_WAREHOUSE_A,
  GO_GARDEN_TO_C,
  GO_WAREHOUSE_C,
  DONE,
  ERROR_STOP,
};

// 各ゾーンへは、X中心合わせ -> 進入方向へ旋回 -> Y方向へ直進、の順で入る。
enum ZoneApproachPhase {
  ALIGN_ZONE_X,
  FACE_ZONE_ENTRY,
  ENTER_ZONE_ALONG_Y,
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
ZoneApproachPhase zoneApproachPhase = ALIGN_ZONE_X;
uint32_t missionStateEnteredMs = 0;
uint32_t targetArrivalSinceMs = 0;
float zoneEntryHeadingRad = 0.0f;

const char *mission_StateName(MissionState state) {
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
  zoneApproachPhase = ALIGN_ZONE_X;
  missionStateEnteredMs = millis();
  targetArrivalSinceMs = 0;
  Serial.print("Mission state: ");
  Serial.println(mission_StateName(nextState));

  if (nextState == INITIAL_ARM_ACTION) {
    // アーム実装後もmission側の状態遷移を変えずに使える受け口。
    mechanism_StartGetCan();
  }

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

void mission_SetZoneApproachPhase(ZoneApproachPhase nextPhase) {
  zoneApproachPhase = nextPhase;
  targetArrivalSinceMs = 0;
}

bool mission_HoldForDwell() {
  if (targetArrivalSinceMs == 0) targetArrivalSinceMs = millis();
  return millis() - targetArrivalSinceMs >= TARGET_DWELL_MS;
}

void mission_CommandFieldVelocity(float vxField, float vyField, float targetHeadingRad) {
  const float headingError = mission_NormalizeAngle(targetHeadingRad - robotPose.heading_rad);
  const float turnRate = constrain(
    MISSION_HEADING_KP * headingError,
    -MISSION_MAX_TURN_RATE_RADPS,
    MISSION_MAX_TURN_RATE_RADPS
  );
  omni_SetFieldVelocity(vxField, vyField, turnRate, robotPose.heading_rad);
}

bool mission_TurnTo(float targetHeadingRad) {
  const float headingError = mission_NormalizeAngle(targetHeadingRad - robotPose.heading_rad);
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

bool mission_ApproachZone(float targetX_m, float targetY_m) {
  switch (zoneApproachPhase) {
    case ALIGN_ZONE_X: {
      const float dx = targetX_m - robotPose.x_m;
      if (fabsf(dx) <= MISSION_X_TOLERANCE_M) {
        omni_SetBodyVelocity(0.0f, 0.0f, 0.0f);
        if (mission_HoldForDwell()) {
          // 現在位置から目標Yへ向かう方向を、ゾーン進入時の正面とする。
          zoneEntryHeadingRad = targetY_m >= robotPose.y_m ? kPi / 2.0f : -kPi / 2.0f;
          mission_SetZoneApproachPhase(FACE_ZONE_ENTRY);
        }
        return false;
      }

      targetArrivalSinceMs = 0;
      const float vxField = constrain(
        MISSION_POSITION_KP * dx,
        -MISSION_X_MAX_SPEED_MPS,
        MISSION_X_MAX_SPEED_MPS
      );
      // この段階ではYを変えず、ゾーン中心のX座標だけに合わせる。
      mission_CommandFieldVelocity(vxField, 0.0f, MISSION_X_ALIGNMENT_HEADING_RAD);
      return false;
    }

    case FACE_ZONE_ENTRY: {
      const float headingError = mission_NormalizeAngle(zoneEntryHeadingRad - robotPose.heading_rad);
      if (fabsf(headingError) <= MISSION_HEADING_TOLERANCE_RAD) {
        omni_SetBodyVelocity(0.0f, 0.0f, 0.0f);
        if (mission_HoldForDwell()) mission_SetZoneApproachPhase(ENTER_ZONE_ALONG_Y);
        return false;
      }

      targetArrivalSinceMs = 0;
      mission_CommandFieldVelocity(0.0f, 0.0f, zoneEntryHeadingRad);
      return false;
    }

    case ENTER_ZONE_ALONG_Y: {
      const float dy = targetY_m - robotPose.y_m;
      if (fabsf(dy) <= MISSION_Y_TOLERANCE_M) {
        // X方向には補正移動せず、Y軸と平行な進入を最後まで維持する。
        mission_CommandFieldVelocity(0.0f, 0.0f, zoneEntryHeadingRad);
        return mission_HoldForDwell();
      }

      targetArrivalSinceMs = 0;
      const float vyField = constrain(
        MISSION_POSITION_KP * dy,
        -MISSION_ENTRY_MAX_SPEED_MPS,
        MISSION_ENTRY_MAX_SPEED_MPS
      );
      mission_CommandFieldVelocity(0.0f, vyField, zoneEntryHeadingRad);
      return false;
    }
  }

  // 到達不能な状態値を検出した場合は安全停止する。
  omni_SetBodyVelocity(0.0f, 0.0f, 0.0f);
  return false;
}

bool mission_DriveTo(float targetX_m, float targetY_m) {
  if (mission_ApproachZone(targetX_m, targetY_m)) {
    omni_SetBodyVelocity(0.0f, 0.0f, 0.0f);
    return true;
  }
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
        mission_SetState(GO_WAREHOUSE_C);
      } else if (mission_MoveTimedOut()) mission_SetState(ERROR_STOP);
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
