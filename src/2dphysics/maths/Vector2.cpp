#include "Vector2.h"
#include <assert.h>
#include <cmath>
#include <algorithm>

Vector2::Vector2() {};
Vector2::Vector2(float x, float y) : x(x), y(y) {};

Vector2 Vector2::ZERO = Vector2();
Vector2 Vector2::ONE = Vector2(1, 1);
Vector2 Vector2::UP = Vector2(0, -1);
Vector2 Vector2::DOWN = Vector2(0, 1);
Vector2 Vector2::LEFT = Vector2(-1, 0);
Vector2 Vector2::RIGHT = Vector2(1, 0);

Vector2 Vector2::rotate(float radian) const
{
  Vector2 result = *this;
  float sinC = std::sin(radian);
  float cosC = std::cos(radian);

  // perform rotation and translate to correct position
  result.x = x * cosC - y * sinC;
  result.y = x * sinC + y * cosC;

  return result;
};

Vector2 Vector2::perpendicular() const
{
  return Vector2(-y, x).normalize();
};

Vector2 Vector2::ceil() const
{
  return Vector2(std::ceil(x), std::ceil(y));
};
Vector2 Vector2::floor() const
{
  return Vector2(std::floor(x), std::floor(y));
};

Vector2 Vector2::clamp(float min, float max) const
{
  return Vector2(std::clamp(x, min, max), std::clamp(y, min, max));
};
Vector2 Vector2::min(float min) const
{
  return Vector2(std::min(x, min), std::min(y, min));
};
Vector2 Vector2::max(float max) const
{
  return Vector2(std::max(x, max), std::max(y, max));
};
Vector2 Vector2::clamp(const Vector2 &min, const Vector2 &max) const
{
  return Vector2(std::clamp(x, min.x, max.x), std::clamp(y, min.y, max.y));
};
Vector2 Vector2::min(const Vector2 &min) const
{
  return Vector2(std::min(x, min.x), std::min(y, min.y));
};
Vector2 Vector2::max(const Vector2 &max) const
{
  return Vector2(std::max(x, max.x), std::max(y, max.y));
};

Vector2 Vector2::addScaled(const Vector2 &v, float scale) const
{
  Vector2 result = *this;
  result.x += v.x * scale;
  result.y += v.y * scale;
  return result;
};
float Vector2::radian() const
{
  return std::atan2(y, x);
};
float Vector2::radian(const Vector2 &v) const
{
  return std::atan2(v.y - y, v.x - x);
};
float Vector2::length() const
{
  return std::sqrt(x * x + y * y);
};
float Vector2::lengthSq() const
{
  return x * x + y * y;
};
float Vector2::dot(const Vector2 &v) const
{
  return x * v.x + y * v.y;
};
float Vector2::cross(const Vector2 &v) const
{
  return x * v.y - y * v.x;
};
Vector2 Vector2::crossZ(float w) const
{
  return Vector2(-w * y, w * x);
};
float Vector2::distance(const Vector2 &v) const
{
  float dx = (x - v.x), dy = y - v.y;
  return std::sqrt(dx * dx + dy * dy);
};
float Vector2::distanceSq(const Vector2 &v) const
{
  float dx = (x - v.x), dy = y - v.y;
  return dx * dx + dy * dy;
};
Vector2 Vector2::normalize() const
{
  float l = std::sqrt(x * x + y * y);
  Vector2 result = *this;
  if (l > 0)
  {
    result.x /= l;
    result.y /= l;
  }
  return result;
};

Vector2 Vector2::random(float scale) const
{
  float radian = std::rand() * M_PI * 2.0;
  return Vector2(std::cos(radian) * scale, std::sin(radian) * scale);
};

float Vector2::operator[](int index) const
{
  assert(index < AXIS_COUNT);
  return coord[index];
};

float &Vector2::operator[](int index)
{
  assert(index < AXIS_COUNT);
  return coord[index];
};

Vector2 Vector2::operator-() const
{
  return Vector2(-x, -y);
};

// 友元定义，不需要类命名空间
std::ostream &operator<<(std::ostream &cout, const Vector2 &v)
{
  return cout << "Vector2(" << v.x << ", " << v.y << ")";
};
Vector2 operator/(float scalar, const Vector2 &v)
{
  return Vector2(scalar / v.x, scalar / v.y);
}
Vector2 operator+(float scalar, const Vector2 &v)
{
  return Vector2(scalar + v.x, scalar + v.y);
}
Vector2 operator-(float scalar, const Vector2 &v)
{
  return Vector2(scalar - v.x, scalar - v.y);
}
Vector2 operator*(float scalar, const Vector2 &v)
{
  return Vector2(scalar * v.x, scalar * v.y);
}

Vector2 Vector2::operator+(const Vector2 &v) const
{
  Vector2 result = *this;
  result.x += v.x;
  result.y += v.y;
  return result;
};
Vector2 Vector2::operator+(float v) const
{
  Vector2 result = *this;
  result.x += v;
  result.y += v;
  return result;
};
Vector2 Vector2::operator-(const Vector2 &v) const
{
  Vector2 result = *this;
  result.x -= v.x;
  result.y -= v.y;
  return result;
};
Vector2 Vector2::operator-(float v) const
{
  Vector2 result = *this;
  result.x -= v;
  result.y -= v;
  return result;
};
Vector2 Vector2::operator*(const Vector2 &v) const
{
  Vector2 result = *this;
  result.x *= v.x;
  result.y *= v.y;
  return result;
};
Vector2 Vector2::operator*(float v) const
{
  Vector2 result = *this;
  result.x *= v;
  result.y *= v;
  return result;
};
Vector2 Vector2::operator/(const Vector2 &v) const
{
  Vector2 result = *this;
  result.x /= v.x;
  result.y /= v.y;
  return result;
};
Vector2 Vector2::operator/(float v) const
{
  Vector2 result = *this;
  result.x /= v;
  result.y /= v;
  return result;
};

void Vector2::operator+=(const Vector2 &v)
{
  x += v.x;
  y += v.y;
};
void Vector2::operator+=(float v)
{
  x += v;
  y += v;
};
void Vector2::operator-=(const Vector2 &v)
{
  x -= v.x;
  y -= v.y;
};
void Vector2::operator-=(float v)
{
  x -= v;
  y -= v;
};
void Vector2::operator*=(const Vector2 &v)
{
  x *= v.x;
  y *= v.y;
};
void Vector2::operator*=(float v)
{
  x *= v;
  y *= v;
};
void Vector2::operator/=(const Vector2 &v)
{
  x /= v.x;
  y /= v.y;
};
void Vector2::operator/=(float v)
{
  x /= v;
  y /= v;
};

bool Vector2::operator==(const Vector2 &v) const
{
  return x == v.x && y == v.y;
};
bool Vector2::operator==(float v) const
{
  return x == v && y == v;
};
bool Vector2::operator!=(const Vector2 &v) const
{
  return x != v.x || y != v.y;
};
bool Vector2::operator!=(float v) const
{
  return x != v || y != v;
};
bool Vector2::operator>=(const Vector2 &v) const
{
  return x >= v.x && y >= v.y;
};
bool Vector2::operator>=(float v) const
{
  return x >= v && y >= v;
};
bool Vector2::operator<=(const Vector2 &v) const
{
  return x <= v.x && y <= v.y;
};
bool Vector2::operator<=(float v) const
{
  return x <= v && y <= v;
};
bool Vector2::operator>(const Vector2 &v) const
{
  return x > v.x && y > v.y;
};
bool Vector2::operator>(float v) const
{
  return x > v && y > v;
};
bool Vector2::operator<(const Vector2 &v) const
{
  return x < v.x && y < v.y;
};
bool Vector2::operator<(float v) const
{
  return x < v && y < v;
};