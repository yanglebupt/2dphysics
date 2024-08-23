#include "Force.h"
#include "Constants.h"

Vector2 Force::GenerateGravityForce(const Particle &particle)
{
  return 1 / particle.invMass * GRAVITY * GRAVITYDIRECTION;
}

Vector2 Force::GenerateGravityForce(const Particle &particle, const Vector2 &gravityDirection, float gravity)
{
  return 1 / particle.invMass * gravity * gravityDirection;
}

Vector2 Force::GenerateDragForce(const Particle &particle, float k)
{
  Vector2 back = -particle.velocity;
  return k * back.lengthSq() * back.normalize();
}

Vector2 Force::GenerateFrictionForce(const Particle &particle, float k)
{
  Vector2 back = -particle.velocity;
  return k * back.normalize();
};

Vector2 Force::GenerateGravitationForce(const Particle &a, const Particle &b)
{
  Vector2 distance = b.position - a.position;

  return G / (a.invMass * b.invMass * distance.lengthSq()) * distance.normalize();
}

Vector2 Force::GenerateGravitationForce(const Particle &a, const Particle &b, float G)
{
  Vector2 distance = b.position - a.position;

  return G / (a.invMass * b.invMass * distance.lengthSq()) * distance.normalize();
}

Vector2 Force::GenerateSpringForce(const Particle &A, const Vector2 &anchor, float retLen, float k)
{
  Vector2 distance = A.position - anchor;
  return k * (retLen - distance.lengthSq()) * distance.normalize();
}

Vector2 Force::GenerateSpringForce(const Particle &A, const Particle &B, float retLen, float k)
{
  Vector2 distance = A.position - B.position;
  return k * (retLen - distance.lengthSq()) * distance.normalize();
}