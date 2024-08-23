#ifndef VECTOR2_H
#define VECTOR2_H

#include <ostream>

struct Vector2
{
  const static int AXIS_COUNT = 2;
  union // 声明一份共享内存地址，方便不同的变量进行访问
  {
    struct
    {
      union
      {
        float x;
        float width;
      };
      union
      {
        float y;
        float height;
      };
    };

    float coord[2] = {0.0};
  };

  Vector2();
  Vector2(float x, float y);

  static Vector2 ZERO;
  static Vector2 ONE;
  static Vector2 LEFT;
  static Vector2 RIGHT;
  static Vector2 UP;
  static Vector2 DOWN;

  Vector2 ceil() const;
  Vector2 floor() const;
  Vector2 clamp(float min, float max) const;
  Vector2 min(float min) const;
  Vector2 max(float max) const;
  Vector2 clamp(const Vector2 &min, const Vector2 &max) const;
  Vector2 min(const Vector2 &min) const;
  Vector2 max(const Vector2 &max) const;

  /**
   * Adds two vectors, scaling the 2nd; assumes a and b have the same dimension.
   * @param v - Added vector
   * @param scale - Amount to scale b
   * @returns A vector that is the sum of this + v * scale.
   */
  Vector2 addScaled(const Vector2 &v, float scale) const;
  float radian() const;
  float radian(const Vector2 &v) const;
  float length() const;
  float lengthSq() const;
  float dot(const Vector2 &v) const;
  float cross(const Vector2 &v) const;
  /**
   * conduct (0,0,w) cross (x,y,0)
   * @return {Vector2}
   */
  Vector2 crossZ(float w) const;
  float distance(const Vector2 &v) const;
  float distanceSq(const Vector2 &v) const;
  Vector2 normalize() const;
  /**
   * Create a random unit vector * scale
   * @param scale - length of Vector2
   * @returns The random vector.
   */
  Vector2 random(float scale) const;
  Vector2 rotate(float radian) const;
  /**
   * Create an unit perpendicular vector by clockwise 90 degrees
   */
  Vector2 perpendicular() const;

  float &operator[](int index);
  float operator[](int index) const;
  // 友元并不是类成员
  friend Vector2 operator/(float scalar, const Vector2 &v);
  friend Vector2 operator*(float scalar, const Vector2 &v);
  friend Vector2 operator+(float scalar, const Vector2 &v);
  friend Vector2 operator-(float scalar, const Vector2 &v);
  friend std::ostream &operator<<(std::ostream &cout, const Vector2 &v);

  Vector2 operator+(const Vector2 &v) const;
  Vector2 operator+(float v) const;
  Vector2 operator-() const;
  Vector2 operator-(const Vector2 &v) const;
  Vector2 operator-(float v) const;
  Vector2 operator*(const Vector2 &v) const;
  Vector2 operator*(float v) const;
  Vector2 operator/(const Vector2 &v) const;
  Vector2 operator/(float v) const;

  void operator+=(const Vector2 &v);
  void operator+=(float v);
  void operator-=(const Vector2 &v);
  void operator-=(float v);
  void operator*=(const Vector2 &v);
  void operator*=(float v);
  void operator/=(const Vector2 &v);
  void operator/=(float v);

  bool operator==(const Vector2 &v) const;
  bool operator==(float v) const;
  bool operator!=(const Vector2 &v) const;
  bool operator!=(float v) const;
  bool operator>=(const Vector2 &v) const;
  bool operator>=(float v) const;
  bool operator<=(const Vector2 &v) const;
  bool operator<=(float v) const;
  bool operator>(const Vector2 &v) const;
  bool operator>(float v) const;
  bool operator<(const Vector2 &v) const;
  bool operator<(float v) const;
};

#endif