#ifndef SHAPE_H
#define SHAPE_H

#include "Vector2.h"

enum ShapeType
{
  CIRCLE,
  POLYGON,
  BOX
};

struct Shape
{
  // pure virtual function, 注意只有指针形式才能利用虚函数，实现多态
  virtual ShapeType GetType() const = 0;
  // return shape related angular mass (moment of inertia)
  virtual float GetMomentOfInertia() const = 0;
  virtual Shape *New() const = 0;
  virtual void UpdateVertices(float angle, const Vector2 &position) = 0;
  virtual ~Shape() {};
};

#endif