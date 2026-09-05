#pragma once

#include <Arduino.h>

void mechanism_Init();
void mechanism_StartGetCan();
void mechanism_StartWater();
bool mechanism_IsActionFinished();
void mechanism_Stop();
