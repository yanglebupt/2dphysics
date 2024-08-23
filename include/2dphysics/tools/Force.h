#ifndef FORCE_H
#define FORCE_H
#include "Vector2.h"
#include "Particle.h"

struct Force
{
  /**
   * Generate gravity force
   * you can change the global gravity in "Constants.h"
   * @param particle target particle which will applied the generated force.
   * Note in this function we would not apply force, you must manually apply the generated force using "AddForce"
   */
  static Vector2 GenerateGravityForce(const Particle &particle);
  static Vector2 GenerateGravityForce(const Particle &particle, const Vector2 &gravityDirection, float gravity);

  /**
   * Generate resistance drag force
   * @param particle target particle which will applied the generated force.
   * Note in this function we would not apply force, you must manually apply the generated force using "AddForce"
   * @param k is the resistance drag constant, which equal 0.5*ρ*K_d*S
   * ρ is density, K_d is contact resistance coefficient, S is contact area
   */
  static Vector2 GenerateDragForce(const Particle &particle, float k);

  /**
   * Generate static friction and dynamic friction, which relate to surface normal force
   * @param particle target particle which will applied the generated force.
   * Note in this function we would not apply force, you must manually apply the generated force using "AddForce"
   * @param k is the dynamic friction constant
   */
  static Vector2 GenerateFrictionForce(const Particle &particle, float k);

  /**
   * Generate universal gravitation between two particles.
   * The generated force should be applied to particle A, negative should be applied to particle B.
   * @param A particle A
   * @param B particle B
   * @param G universal gravitation cofficient
   */
  static Vector2 GenerateGravitationForce(const Particle &A, const Particle &B);
  static Vector2 GenerateGravitationForce(const Particle &A, const Particle &B, float G);

  /**
   * Generate spring force with a fixed anchor point
   */
  static Vector2 GenerateSpringForce(const Particle &A, const Vector2 &anchor, float retLen, float k);
  /**
   * Generate spring force between two particles.
   * The generated force should be applied to particle A, negative should be applied to particle B.
   */
  static Vector2 GenerateSpringForce(const Particle &A, const Particle &B, float retLen, float k);
};

#endif