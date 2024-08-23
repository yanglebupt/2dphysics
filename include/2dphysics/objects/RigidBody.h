#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "Particle.h"
#include "Shape.h"

struct Multiplier
{
  float angular = 1.f;
  float linear = 1.f;
};

/**
 * rigid body is a particle(pivot/centroid) with specific shape, so it also has angle velocity
 * it is non deformable in an ideal state
 */
struct RigidBody : Particle
{
  bool isColliding;
  // shape 是 virtual，必须是指针
  Shape *shape;

  // Linear motion inherited from particle

  // Angular motion
  float rotation;
  float angularVelocity;
  float angularAcceleration;
  /**
   * inverse angular mass same as inverse mass, which is the inverse torque moment inertia
   * which relates to different shape and different rotate axis, and it's mass
   **/
  float invI;
  // 总扭矩  和 总力类似
  float sumTorque;
  // 碰撞恢复系数，越大弹性越大，越小，损失的越多
  float restitution;
  // 摩擦系数
  float friction;
  /**
   * each update angular velocity will multiply this multiplier
   * for example rotation += dt * angularVelocity * TURNS_PER_ANGLE * angularMultiplier;
   **/
  float angularMultiplier;

  void SetInvI(float I);
  // update all angular state
  void IntegrateAngular(float dt);

  // only update velocity and angular velocity by force and torque
  void IntegrateForce(float dt);
  // only update position and angular by velocity and angular velocity
  void IntegrateVelocity(float dt);

  // shape 不要传栈上的内存
  RigidBody(const Shape &shape, float x, float y, float mass, Multiplier multiplier = {});
  RigidBody(const Shape &shape, Vector2 position, float mass, Multiplier multiplier = {});
  /**
   * customize you shape Moment Of Inertia, otherwise we will use the Moment Of Inertia of specific shape
   * Moment Of Inertia same as mass, more bigger, more hard to rotate
   */
  RigidBody(const Shape &shape, float x, float y, float mass, float momentOfInertia, Multiplier multiplier = {});
  RigidBody(const Shape &shape, Vector2 position, float mass, float momentOfInertia, Multiplier multiplier = {});

  void SetDefault();

  void AddTorque(float torque);
  void ClearTorque();

  // update linear, angular and the update vertices
  void Update(float dt);

  /**
   * get resultant velocity for a vector
   * including angular velocity and linear velocity
   * @param r start from rigidbody position end with target point in rigidbody
   **/
  Vector2 GetResultantVelocityFromV(const Vector2 &r);

  /**
   * get resultant velocity for a point in rigidbody
   * including angular velocity and linear velocity
   * @param point a point in rigidbody
   **/
  Vector2 GetResultantVelocityFromP(const Vector2 &point);

  /**
   * add impulse to particle velocity **Note** No angular velocity
   */
  void ApplyImpulse(const Vector2 &J);

  /**
   * add impulse to rigidbody angular velocity
   */
  void ApplyImpulse(float impulse);

  /**
   * add impulse to rigibody's point velocity and angular velocity
   * @param r start from rigidbody position end with target point in rigidbody
   */
  void ApplyImpulseV(const Vector2 &J, const Vector2 &r);

  /**
   * add impulse to rigibody's point velocity and angular velocity
   * @param point a point in rigidbody
   */
  void ApplyImpulseP(const Vector2 &J, const Vector2 &point);

  Vector2 WorldPointToLocalPoint(const Vector2 &point) const;
  Vector2 LocalPointToWorldPoint(const Vector2 &point) const;

  ~RigidBody();
};

#endif