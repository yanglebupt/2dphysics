#include "CollisionDetection.h"
#include "Shape.h"
#include "CircleShape.h"
#include "PolygonShape.h"
#include <stddef.h>
#include <limits>

bool CollisionDetection::IsColliding(RigidBody *a, RigidBody *b, std::vector<Contact> &contacts)
{
  bool aIsCircle = a->shape->GetType() == ShapeType::CIRCLE;
  bool bIsCircle = b->shape->GetType() == ShapeType::CIRCLE;

  PolygonShape *aPolygonShape = dynamic_cast<PolygonShape *>(a->shape);
  PolygonShape *bPolygonShape = dynamic_cast<PolygonShape *>(b->shape);
  bool aIsPolygon = aPolygonShape != nullptr;
  bool bIsPolygon = bPolygonShape != nullptr;

  if (aIsCircle && bIsCircle)
    return IsCollidingCircleCircle(a, b, contacts);
  else if (aIsPolygon && bIsPolygon)
    return IsCollidingPolygonPolygon(a, b, contacts);
  else if (aIsPolygon && bIsCircle)
    return IsCollidingPolygonCircle(a, b, contacts);
  else if (bIsPolygon && aIsCircle)
    return IsCollidingPolygonCircle(b, a, contacts);

  return false;
};

bool CollisionDetection::IsCollidingCircleCircle(RigidBody *a, RigidBody *b, std::vector<Contact> &contacts)
{
  CircleShape *aShape = dynamic_cast<CircleShape *>(a->shape);
  CircleShape *bShape = dynamic_cast<CircleShape *>(b->shape);

  Vector2 ab = b->position - a->position;
  float sumRadius = aShape->radius + bShape->radius;

  if (ab.lengthSq() > sumRadius * sumRadius) // #CHECK: = 边缘也认为是相交
    return false;

  // Compute the collision contact information for collision resolve!
  Contact contact;
  contact.a = a;
  contact.b = b;
  contact.normal = ab.normalize(); // a to b

  contact.start = b->position - contact.normal * bShape->radius; // start near normal start
  contact.end = a->position + contact.normal * aShape->radius;   // end near normal end
  contact.depth = contact.start.distance(contact.end);

  contacts.push_back(contact);
  return true;
};

int ClipSegmentToLine(std::vector<Vector2> &inPoints, std::vector<Vector2> &outPoints, Vector2 &c0, Vector2 &c1)
{
  int numOut = 0;
  Vector2 dir = (c1 - c0).normalize();
  float dist0 = (inPoints[0] - c0).cross(dir);
  float dist1 = (inPoints[1] - c0).cross(dir);

  // inPoint 在同侧，直接返回，不需要裁剪
  if (dist0 >= 0)
    outPoints[numOut++] = inPoints[0];
  if (dist1 >= 0)
    outPoints[numOut++] = inPoints[1];

  // inPoint 在异侧，开始裁剪
  if (dist0 * dist1 < 0)
  {
    float totalDist = dist0 - dist1;
    float t = dist0 / totalDist;
    outPoints[numOut++] = inPoints[0] + t * (inPoints[1] - inPoints[0]);
  }

  return numOut;
}

bool CollisionDetection::IsCollidingPolygonPolygon(RigidBody *a, RigidBody *b, std::vector<Contact> &contacts)
{
  PolygonShape *aPolygonShape = dynamic_cast<PolygonShape *>(a->shape);
  PolygonShape *bPolygonShape = dynamic_cast<PolygonShape *>(b->shape);

  int aReferenceEdgeIndex, bReferenceEdgeIndex;
  int aIncidentEdgeIndex, bIncidentEdgeIndex;
  Vector2 aSupportPoint, bSupportPoint;
  // b 侵入 a
  float bDepth = aPolygonShape->FindClosestPenetration(*bPolygonShape, aReferenceEdgeIndex, bIncidentEdgeIndex, bSupportPoint);
  if (bDepth > 0) // #CHECK: = 边缘也认为是相交
    return false;
  // a 侵入 b
  float aDepth = bPolygonShape->FindClosestPenetration(*aPolygonShape, bReferenceEdgeIndex, aIncidentEdgeIndex, aSupportPoint);
  if (aDepth > 0) // #CHECK: = 边缘也认为是相交
    return false;

  // 决定 ReferenceShape 和 IncidentShape
  PolygonShape *referenceShape;
  PolygonShape *incidentShape;
  int referenceEdgeIndex;
  int incidentEdgeIndex;
  if (aDepth > bDepth) // a 侵入 b 的绝对距离更小，说明靠这边侵入
  {
    // a is IncidentShape, b is ReferenceShape
    referenceShape = bPolygonShape;
    incidentShape = aPolygonShape;
    referenceEdgeIndex = bReferenceEdgeIndex;
    incidentEdgeIndex = aIncidentEdgeIndex;
  }
  else
  {
    // a is ReferenceShape, b is IncidentShape
    referenceShape = aPolygonShape;
    incidentShape = bPolygonShape;
    referenceEdgeIndex = aReferenceEdgeIndex;
    incidentEdgeIndex = bIncidentEdgeIndex;
  }

  // Clip incidentEdge by **SideEdges** of referenceEdge in referenceShape to find multiple support points for contact

  // start point and end point of incidentEdge
  int n = referenceShape->worldVertices.size(), m = incidentShape->worldVertices.size();
  Vector2 v0 = incidentShape->worldVertices[incidentEdgeIndex];
  Vector2 v1 = incidentShape->worldVertices[(incidentEdgeIndex + 1) % m];

  // clip 开始的点
  std::vector<Vector2> contactPoints = {v0, v1};
  // clip 结束后的点
  std::vector<Vector2> clippedPoints = contactPoints;
  /**
   * reference side edges only two
   * - Edge(referenceEdgeIndex-1, referenceEdgeIndex)
   * - Edge(referenceEdgeIndex+1, referenceEdgeIndex+2)
   **/
  int sideEdges[2] = {(n + referenceEdgeIndex - 1) % n, (n + referenceEdgeIndex + 1) % n};

  for (int i = 0; i < 2; i++)
  {
    int sideEdgeIndex = sideEdges[i];
    Vector2 c0 = referenceShape->worldVertices[sideEdgeIndex];
    Vector2 c1 = referenceShape->worldVertices[(sideEdgeIndex + 1) % n];
    // 使用 c0, c1 这条边去 clip contactPoints，然后写入 clippedPoints
    int numClipped = ClipSegmentToLine(contactPoints, clippedPoints, c0, c1);
    if (numClipped > 2)
      break;
    // 将这一次的 clippedPoints 作为下一次 clip 开始的点
    contactPoints = clippedPoints;
  }

  // Loop all clipped points, but only keep penetration depth is negative
  // 只保留侵入的点
  Vector2 vref = referenceShape->worldVertices[referenceEdgeIndex];
  Vector2 referenceEdgeNormal = referenceShape->GetEdgeNormal(referenceEdgeIndex);
  for (auto &vclip : clippedPoints)
  {
    float depth = (vclip - vref).dot(referenceEdgeNormal);
    if (depth < 0)
    {
      Contact contact;
      contact.a = a;
      contact.b = b;
      // 当 a 是 referenceShape
      contact.normal = referenceEdgeNormal;
      contact.depth = -depth;
      contact.start = vclip;
      contact.end = contact.start + contact.depth * contact.normal;
      if (aDepth > bDepth)
      {
        std::swap(contact.start, contact.end);
        contact.normal *= -1;
      }
      contacts.push_back(contact);
    }
  }

  return true;
};

bool CollisionDetection::IsCollidingPolygonCircle(RigidBody *a, RigidBody *b, std::vector<Contact> &contacts)
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
      Contact contact;
      contact.a = a;
      contact.b = b;
      contact.normal = startVCircle.normalize();
      contact.depth = r - depth;
      contact.end = start;
      contact.start = contact.end - contact.depth * contact.normal;
      contacts.push_back(contact);
      return true;
    }
    Vector2 endVCircle = b->position - end;
    float outSideEnd = endVCircle.dot(start - end);
    if (outSideEnd < 0)
    {
      float depth = endVCircle.length();
      if (depth > r) // #CHECK: = 边缘也认为是相交
        return false;
      Contact contact;
      contact.a = a;
      contact.b = b;
      contact.normal = endVCircle.normalize();
      contact.depth = r - depth;
      contact.end = end;
      contact.start = contact.end - contact.depth * contact.normal;
      contacts.push_back(contact);
      return true;
    }
    else
    {
      if (closedDepth > r) // #CHECK: = 边缘也认为是相交
        return false;
      Contact contact;
      contact.a = a;
      contact.b = b;
      contact.depth = r - closedDepth;
      contact.normal = edgeNormal;
      contact.start = b->position - edgeNormal * r;
      contact.end = contact.start + contact.depth * contact.normal;
      contacts.push_back(contact);
      return true;
    }
  }
  else
  {
    // inside
    Contact contact;
    contact.a = a;
    contact.b = b;
    contact.depth = r - closedDepth;
    contact.normal = edgeNormal;
    contact.start = b->position - r * contact.normal;
    contact.end = b->position - closedDepth * contact.normal;
    contacts.push_back(contact);
    return true;
  }
};