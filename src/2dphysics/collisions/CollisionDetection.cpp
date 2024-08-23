#include "CollisionDetection.h"
#include "Shape.h"
#include "CircleShape.h"
#include "PolygonShape.h"
#include <stddef.h>
#include <limits>

bool CollisionDetection::IsColliding(RigidBody *a, RigidBody *b, Contact &contact)
{
  bool aIsCircle = a->shape->GetType() == ShapeType::CIRCLE;
  bool bIsCircle = b->shape->GetType() == ShapeType::CIRCLE;

  PolygonShape *aPolygonShape = dynamic_cast<PolygonShape *>(a->shape);
  PolygonShape *bPolygonShape = dynamic_cast<PolygonShape *>(b->shape);
  bool aIsPolygon = aPolygonShape != nullptr;
  bool bIsPolygon = bPolygonShape != nullptr;

  if (aIsCircle && bIsCircle)
    return IsCollidingCircleCircle(a, b, contact);
  else if (aIsPolygon && bIsPolygon)
    return IsCollidingPolygonPolygon(a, b, contact);
  else if (aIsPolygon && bIsCircle)
    return IsCollidingPolygonCircle(a, b, contact);
  else if (bIsPolygon && aIsCircle)
    return IsCollidingPolygonCircle(b, a, contact);

  return false;
};

bool CollisionDetection::IsCollidingCircleCircle(RigidBody *a, RigidBody *b, Contact &contact)
{
  CircleShape *aShape = dynamic_cast<CircleShape *>(a->shape);
  CircleShape *bShape = dynamic_cast<CircleShape *>(b->shape);

  Vector2 ab = b->position - a->position;
  float sumRadius = aShape->radius + bShape->radius;

  if (ab.lengthSq() > sumRadius * sumRadius) // #CHECK: = 边缘也认为是相交
    return false;

  // Compute the collision contact information for collision resolve!
  contact.a = a;
  contact.b = b;
  contact.normal = ab.normalize(); // a to b

  contact.start = b->position - contact.normal * bShape->radius; // start near normal start
  contact.end = a->position + contact.normal * aShape->radius;   // end near normal end

  contact.depth = contact.start.distance(contact.end);

  return true;
};

bool CollisionDetection::IsCollidingPolygonPolygon(RigidBody *a, RigidBody *b, Contact &contact)
{

  PolygonShape *aPolygonShape = dynamic_cast<PolygonShape *>(a->shape);
  PolygonShape *bPolygonShape = dynamic_cast<PolygonShape *>(b->shape);
  Vector2 aNormal, bNormal;
  Vector2 aPoint, bPoint;
  // b 侵入 a
  float bDepth = aPolygonShape->FindClosestPenetration(*bPolygonShape, aNormal, bPoint);
  if (bDepth > 0) // #CHECK: = 边缘也认为是相交
    return false;
  // a 侵入 b
  float aDepth = bPolygonShape->FindClosestPenetration(*aPolygonShape, bNormal, aPoint);
  if (aDepth > 0) // #CHECK: = 边缘也认为是相交
    return false;

  contact.a = a;
  contact.b = b;

  if (aDepth > bDepth) // a 侵入 b 的绝对距离更小，说明靠这边侵入
  {
    contact.normal = -bNormal;
    contact.depth = -aDepth;
    contact.end = aPoint;
    contact.start = aPoint - bNormal * aDepth;
  }
  else
  {
    contact.normal = aNormal;
    contact.depth = -bDepth;
    contact.start = bPoint;
    contact.end = bPoint - bDepth * aNormal;
  }

  return true;
};

bool CollisionDetection::IsCollidingPolygonCircle(RigidBody *a, RigidBody *b, Contact &contact)
{
  PolygonShape *aPolygonShape = dynamic_cast<PolygonShape *>(a->shape);
  CircleShape *bCircleShape = dynamic_cast<CircleShape *>(b->shape);

  // 1. loop all edges and find closest edge to cirlce
  float closedDepth = std::numeric_limits<float>::lowest();
  Vector2 start, end, edgeNormal;
  auto vertices = aPolygonShape->worldVertices;
  for (size_t i = 0, n = vertices.size(); i < n; i++)
  {
    Vector2 _start = vertices[i];
    Vector2 _end = vertices[(i + 1) % n];
    Vector2 _edgeNormal = (_end - _start).perpendicular();
    // distance from cirlce center to edge
    float distanceToEdge = (b->position - _start).dot(_edgeNormal);
    // use maximum distance
    if (distanceToEdge > closedDepth)
    {
      closedDepth = distanceToEdge;
      start = _start;
      end = _end;
      edgeNormal = _edgeNormal;
    }
  }

  float r = bCircleShape->radius;

  // 2. circle center is outside of polygon
  if (closedDepth > 0) // #CHECK: = 边缘也认为是相交
  {
    // outside
    Vector2 startVCircle = b->position - start;
    float outSideStart = startVCircle.dot(end - start);
    if (outSideStart < 0)
    {
      float depth = startVCircle.length();
      if (depth > r) // #CHECK: = 边缘也认为是相交
        return false;
      contact.a = a;
      contact.b = b;
      contact.normal = startVCircle.normalize();
      contact.depth = r - depth;
      contact.end = start;
      contact.start = contact.end - contact.depth * contact.normal;
      return true;
    }
    Vector2 endVCircle = b->position - end;
    float outSideEnd = endVCircle.dot(start - end);
    if (outSideEnd < 0)
    {
      float depth = endVCircle.length();
      if (depth > r) // #CHECK: = 边缘也认为是相交
        return false;
      contact.a = a;
      contact.b = b;
      contact.normal = endVCircle.normalize();
      contact.depth = r - depth;
      contact.end = end;
      contact.start = contact.end - contact.depth * contact.normal;
      return true;
    }
    else
    {
      if (closedDepth > r) // #CHECK: = 边缘也认为是相交
        return false;
      contact.a = a;
      contact.b = b;
      contact.depth = r - closedDepth;
      contact.normal = edgeNormal;
      contact.start = b->position - edgeNormal * r;
      contact.end = contact.start + contact.depth * contact.normal;
      return true;
    }
  }
  else
  {
    // inside
    contact.a = a;
    contact.b = b;
    contact.depth = r - closedDepth;
    contact.normal = edgeNormal;
    contact.start = b->position - r * contact.normal;
    contact.end = b->position - closedDepth * contact.normal;
    return true;
  }
};