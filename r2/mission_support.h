#pragma once

#include <Arduino.h>

// ToFから得た正面距離。valid=false の間は位置補正に使わない。
struct ToFReading {
  bool valid = false;
  float distance_m = 0.0f;
};

void tof_Init();
void tof_Update();
bool tof_GetFrontDistance(ToFReading &reading);

void mechanism_Init();
void mechanism_StartGetCan();
void mechanism_StartWater();
bool mechanism_IsActionFinished();
void mechanism_Stop();
