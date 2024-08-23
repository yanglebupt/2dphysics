#include "BoxShape.h"
#include "Constants.h"

BoxShape::BoxShape(float width, float height) : PolygonShape(), width(width), height(height)
{
  float hw = 0.5 * width, hh = 0.5 * height;
  localVertices.push_back(Vector2(-hw, -hh));
  localVertices.push_back(Vector2(-hw, hh));
  localVertices.push_back(Vector2(hw, hh));
  localVertices.push_back(Vector2(hw, -hh));

  worldVertices = localVertices; // init world vertices, Otherwise, the size is empty
};

BoxShape::~BoxShape() {};

float BoxShape::GetMomentOfInertia() const
{
  return 0.0834 * (width * width + height * height); // pixel 实在是太大了，和实际不符
};

ShapeType BoxShape::GetType() const
{
  return ShapeType::BOX;
};

Shape *BoxShape::New() const
{
  return new BoxShape(width, height);
}