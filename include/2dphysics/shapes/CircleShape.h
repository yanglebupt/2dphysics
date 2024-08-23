#ifndef CIRCLESHAPE_H
#define CIRCLESHAPE_H

#include "Shape.h"

struct CircleShape : Shape
{
  float radius;

  CircleShape(float radius);
  ~CircleShape();
  float GetMomentOfInertia() const override;
  Shape *New() const override;
  ShapeType GetType() const override;
  void UpdateVertices(float angle, const Vector2 &position) override;
};

#endif