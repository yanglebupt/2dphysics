#ifndef CONTACT_H
#define CONTACT_H

#include "RigidBody.h"
#include "Vector2.h"

struct Contact
{
  // 发生碰撞的两个刚体
  RigidBody *a;
  RigidBody *b;

  // 碰撞的起点和终点
  Vector2 start; // collision point in rigid body B
  Vector2 end;   // collision point in rigid body A

  /**
   * 碰撞的法线
   * patential impulse should be applied，which direction is from A to B
   **/
  Vector2 normal;

  /**
   * 碰撞陷入的深度
   * will determine how big the impulse should be used
   **/
  float depth;

  /**
   * projection method 通过更改位置，来避免陷入（重叠）
   * d_a = depth * m_b / (m_a + m_b)
   * d_b = depth * m_a / (m_a + m_b)
   * Note we should use inverse mass to calculate
   * @return bool is resolved, if all static, will return false
   */
  bool ResolvePenetration();

  /**
   * impulse change the velocity directly I=FΔt=mΔv
   * momentum conservation and energy conservation
   * impulse direction same as normal
   * v_rel = v_a - v_b  project to normal direction should decay with -e
   *
   * **Note still we would not resolve angular velocity, we just resolve linear velocity**
   */
  bool ResolveLinearCollision();

  /**
   * impulse change the velocity and anguler velocity directly
   * we will also add both normal and tangent impulse!
   * ** Note impulse will applied to velocity and angular velocity
   */
  bool ResolveCollision();

  /**
   * 计算沿着某个碰撞方向的冲量
   */
  Vector2 GetCollisionImpulse(const Vector2 &ra, const Vector2 &rb, const Vector2 &v_rel, const Vector2 &n, float e);
};

#endif