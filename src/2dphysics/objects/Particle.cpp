#include "Particle.h"
#include "Constants.h"
#include <cmath>

Particle::Particle(float x, float y, float mass, float linearMultiplier) : position(x, y), linearMultiplier(linearMultiplier)
{
  SetInvMass(mass);
};

Particle::Particle(Vector2 position, float mass, float linearMultiplier) : position(position), linearMultiplier(linearMultiplier)
{
  SetInvMass(mass);
};

bool Particle::IsStatic()
{
  return IsFloatEqual(invMass, 0);
};

void Particle::SetInvMass(float mass)
{
  invMass = mass != 0.f ? linearMultiplier / mass : 0.f; // != 0.0 only happens for exact zero;
};

void Particle::IntegrateLinear(float dt)
{
  if (IsStatic())
    return;

  acceleration = sumForce * invMass * PIXELS_PER_METER;
  velocity += dt * acceleration;
  velocity = velocity.min(MAX_VELOCITY);
  position += dt * velocity;

  ClearForce();
}

void Particle::ApplyImpulse(const Vector2 &J)
{
  if (IsStatic())
    return;
  velocity += J * invMass;
};

void Particle::AddForce(const Vector2 &force)
{
  if (IsStatic())
    return;
  sumForce += force;
};

void Particle::ClearForce()
{
  sumForce = Vector2::ZERO;
}