#ifndef PARTICLE_H
#define PARTICLE_H

#include "Vector2.h"

struct Particle
{
  // inverse mass can simulate infinite mass
  float invMass;

  Vector2 position;
  Vector2 velocity;
  Vector2 acceleration;
  Vector2 sumForce;

  /**
   * each update velocity will multiply this multiplier
   * for example position += dt * velocity * PIXELS_PER_METER * linearMultiplier;
   **/
  float linearMultiplier;

  /**
   * mass > 1/EPSILON  or mass == 0 indicates infinite mass, static body
   * invMass = 1/mass < EPSILON --> 0 or invMass = 0
   */
  Particle(float x, float y, float mass, float linearMultiplier = 1.f);
  Particle(Vector2 position, float mass, float linearMultiplier = 1.f);

  bool IsStatic();

  void SetInvMass(float mass);
  /**
   * update position, velocity and acceleration using particle's velocity, acceleration and sumForce
   */
  void IntegrateLinear(float dt);

  /**
   * add force to particle sumForce
   */
  void AddForce(const Vector2 &force);

  /**
   * add impulse to particle velocity
   */
  void ApplyImpulse(const Vector2 &J);

  /**
   * set particle sumForce to zeor
   */
  void ClearForce();
};

#endif