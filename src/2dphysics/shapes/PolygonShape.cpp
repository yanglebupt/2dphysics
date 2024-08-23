#include "PolygonShape.h"
#include "limits"
#include "stddef.h"
#include "Constants.h"

PolygonShape::PolygonShape() {};

PolygonShape::PolygonShape(const std::vector<Vector2> vertices)
{
  worldVertices = localVertices = vertices;
};

PolygonShape::~PolygonShape()
{
  localVertices.clear();
  worldVertices.clear();
  // 局部变量销毁，触发析构
  std::vector<Vector2>().swap(localVertices);
  std::vector<Vector2>().swap(worldVertices);
};

float PolygonShape::GetMomentOfInertia() const
{
  return localVertices.size() * TURNS_PER_ANGLE * TURNS_PER_ANGLE;
};

ShapeType PolygonShape::GetType() const
{
  return ShapeType::POLYGON;
};

Shape *PolygonShape::New() const
{
  return new PolygonShape(worldVertices);
};

// update the world position after physics simulation
void PolygonShape::UpdateVertices(float angle, const Vector2 &position)
{
  for (size_t i = 0, n = localVertices.size(); i < n; i++)
  {
    // rotation and then translation
    worldVertices[i] = localVertices[i].rotate(angle) + position;
  }
};

float PolygonShape::FindClosestPenetration(const PolygonShape &other, Vector2 &normal, Vector2 &point) const
{
  float closestDepth = std::numeric_limits<float>::lowest(); // lowest() return -infinity min return nearly zero
  for (size_t i = 0, n = worldVertices.size(); i < n; i++)
  {
    Vector2 edgeNormal = GetEdgeNormal(i);
    Vector2 va = worldVertices[i];
    Vector2 closestPoint;
    float minProjection = std::numeric_limits<float>::max();
    for (auto vb : other.worldVertices)
    {
      float p = (vb - va).dot(edgeNormal);
      if (p < minProjection)
      {
        minProjection = p;
        closestPoint = vb;
      }
    }
    if (minProjection > closestDepth)
    {
      closestDepth = minProjection;
      normal = edgeNormal;
      point = closestPoint;
    }
  }

  return closestDepth;
};

Vector2 PolygonShape::GetEdge(int index) const
{
  return worldVertices[(index + 1) % worldVertices.size()] - worldVertices[index];
};

Vector2 PolygonShape::GetEdgeNormal(int index) const
{
  return GetEdge(index).perpendicular();
};