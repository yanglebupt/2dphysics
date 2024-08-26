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

float PolygonShape::FindClosestPenetration(const PolygonShape &other, int &referenceEdgeIndex, int &incidentEdgeIndex, Vector2 &supportPoint) const
{
  float closestDepth = std::numeric_limits<float>::lowest(); // lowest() return -infinity min return nearly zero
  int possibleIncidentEdgeIndex;
  int n = worldVertices.size(), m = other.worldVertices.size();
  for (size_t i = 0; i < n; i++)
  {
    Vector2 edgeNormal = GetEdgeNormal(i);
    Vector2 va = worldVertices[i];
    Vector2 closestPoint;
    int p_incidentEdgeIndex;
    float minProjection = std::numeric_limits<float>::max();
    for (size_t j = 0; j < m; j++)
    {
      Vector2 vb = other.worldVertices[j];
      float p = (vb - va).dot(edgeNormal);
      if (p < minProjection)
      {
        minProjection = p;
        closestPoint = vb;
        p_incidentEdgeIndex = j;
      }
    }
    if (minProjection > closestDepth)
    {
      closestDepth = minProjection;
      referenceEdgeIndex = i;
      supportPoint = closestPoint;
      possibleIncidentEdgeIndex = p_incidentEdgeIndex;
    }
  }

  /**
   * 决定 incidentEdgeIndex
   * - possibleIncidentEdgeIndex 为 incidentEdge 的起点，则 incidentEdgeIndex = possibleIncidentEdgeIndex
   * - possibleIncidentEdgeIndex 为 incidentEdge 的终点，则 incidentEdgeIndex = possibleIncidentEdgeIndex - 1
   **/

  Vector2 referenceEdgeNormal = GetEdgeNormal(referenceEdgeIndex);
  float minD = std::numeric_limits<float>::max();
  for (size_t i = 0; i < 2; i++)
  {
    int p_incidentEdge = (m + possibleIncidentEdgeIndex - i) % m;
    Vector2 incidentEdgeNormal = other.GetEdgeNormal(p_incidentEdge);
    float d = incidentEdgeNormal.dot(referenceEdgeNormal);
    if (d < minD)
    {
      minD = d;
      incidentEdgeIndex = p_incidentEdge;
    }
  }

  return closestDepth;
};

Vector2 PolygonShape::GetEdge(int index) const
{
  return worldVertices[(index + 1) % worldVertices.size()] - worldVertices[index % worldVertices.size()];
};

Vector2 PolygonShape::GetEdgeNormal(int index) const
{
  return GetEdge(index).perpendicular();
};