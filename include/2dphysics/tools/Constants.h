#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "Vector2.h"

// fix physics time step
const int FPS = 60;
// 16 ms
const int MS_PER_FRAME = 1000. / FPS;
// 现实中的 1m 等于多少个 pixels
const int PIXELS_PER_METER = 50;
// 现实中的 1 转动等于多少个 pixel 世界中多少转动
const int TURNS_PER_ANGLE = 10;

// limits
// 最大速度
const float MAX_VELOCITY = 100.f * PIXELS_PER_METER;
// 最大转速
const float MAX_ANGULAR_VELOCITY = 100.f * TURNS_PER_ANGLE;

// gravity
const float GRAVITY = 9.8f;
const Vector2 GRAVITYDIRECTION = Vector2(0, 1);

// 万有引力常数
const int G = 6.67428 * 1e-11;

const float EPSILON = 1e-10;
bool IsFloatEqual(float a, float b);

#endif