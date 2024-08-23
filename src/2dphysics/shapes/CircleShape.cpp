#include "CircleShape.h"
#include "Constants.h"

CircleShape::CircleShape(float radius) : radius(radius) {};

float CircleShape::GetMomentOfInertia() const
{
  return 0.5 * radius * radius;
};

CircleShape::~CircleShape() {}

ShapeType CircleShape::GetType() const
{
  return ShapeType::CIRCLE;
};

Shape *CircleShape::New() const
{
  return new CircleShape(radius);
}

void CircleShape::UpdateVertices(float angle, const Vector2 &position)
{
  return; // Nothing to do here
}