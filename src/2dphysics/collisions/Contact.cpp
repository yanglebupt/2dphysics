#include "Contact.h"
#include "Constants.h"
#include <cmath>

bool Contact::ResolvePenetration() const
{
  if (a->IsStatic() && b->IsStatic())
    return false;

  float t_inv_mass = a->invMass + b->invMass;
  float da = (depth / t_inv_mass) * a->invMass;
  float db = (depth / t_inv_mass) * b->invMass;

  a->position -= normal * da;
  b->position += normal * db;

  // immediately update world position
  a->shape->UpdateVertices(a->rotation, a->position);
  b->shape->UpdateVertices(b->rotation, b->position);

  return true;
};

bool Contact::ResolveLinearCollision() const
{
  if (!ResolvePenetration())
    return false;

  float e = std::max(a->restitution, b->restitution);
  Vector2 v_rel = a->velocity - b->velocity;

  float impulseMag = (1 + e) * v_rel.dot(normal) / (a->invMass + b->invMass);
  Vector2 jn = impulseMag * normal;

  a->ApplyImpulse(-jn);
  b->ApplyImpulse(jn);

  return true;
};

Vector2 Contact::GetCollisionImpulse(const Vector2 &ra, const Vector2 &rb, const Vector2 &v_rel, const Vector2 &n, float e) const
{
  float RaCroN = ra.cross(n);
  float RbCroN = rb.cross(n);
  float impulseMag = (1 + e) * v_rel.dot(n) / (a->invMass + b->invMass + RaCroN * RaCroN * a->invI + RbCroN * RbCroN * b->invI);
  return impulseMag * n;
}
bool Contact::ResolveCollision() const
{
  if (!ResolvePenetration())
    return false;

  float e = std::max(a->restitution, b->restitution);
  float f = std::max(a->friction, b->friction);

  Vector2 ra = end - a->position, rb = start - b->position;
  Vector2 v_rel = a->GetResultantVelocityFromV(ra) - b->GetResultantVelocityFromV(rb);

  Vector2 jn = GetCollisionImpulse(ra, rb, v_rel, normal, e);
  Vector2 jt = GetCollisionImpulse(ra, rb, v_rel, normal.perpendicular(), e);

  Vector2 j = jn + f * jt;

  a->ApplyImpulseV(-j, ra);
  b->ApplyImpulseV(j, rb);

  return true;
};