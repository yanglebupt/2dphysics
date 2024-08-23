#ifndef BOXSHAPE_H
#define BOXSHAPE_H

#include "PolygonShape.h"

struct BoxShape : PolygonShape
{
  float width;
  float height;

  BoxShape(float width, float height);
  ~BoxShape();
  float GetMomentOfInertia() const override;
  Shape *New() const override;
  ShapeType GetType() const override;
};

#endif