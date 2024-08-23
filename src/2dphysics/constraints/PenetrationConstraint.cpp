#include "PenetrationConstraint.h"
#include <algorithm>

PenetrationConstraint::PenetrationConstraint(Contact contact, float biasBeta) : Constraint(contact.a, contact.b, biasBeta)
{
  startInBLocal = b->WorldPointToLocalPoint(contact.start);
  endInALocal = a->WorldPointToLocalPoint(contact.end);
  normalInALocal = a->WorldPointToLocalPoint(contact.normal);
  lambda = VectorN(2);
};

MatrixMN PenetrationConstraint::GetJ(float dt)
{
  Vector2 pa = a->LocalPointToWorldPoint(endInALocal);
  Vector2 pb = b->LocalPointToWorldPoint(startInBLocal);
  Vector2 n = a->LocalPointToWorldPoint(normalInALocal);

  Vector2 ra = pa - a->position;
  Vector2 rb = pb - b->position;

  // when constraints satisfied (pb - pa).dot(n) >= 0
  float C = (pb - pa).dot(n);
  // C = std::min(0.f, C + 0.01f); // when C >= -0.01, 马上就要离开，return C=0

  MatrixMN J(2, 6);

  J[0][0] = -n.x;
  J[0][1] = -n.y;
  J[0][2] = n.cross(ra);

  J[0][3] = n.x;
  J[0][4] = n.y;
  J[0][5] = rb.cross(n);

  friction = std::max(a->friction, b->friction);
  if (friction > 0.f)
  {
    Vector2 t = n.perpendicular();

    J[1][0] = -t.x;
    J[1][1] = -t.y;
    J[1][2] = t.cross(ra);

    J[1][3] = t.x;
    J[1][4] = t.y;
    J[1][5] = rb.cross(t);
  }

  Vector2 v_rel = a->GetResultantVelocityFromV(ra) - b->GetResultantVelocityFromV(rb);

  float restitution = std::max(a->restitution, b->restitution);
  bias = (biasBeta / dt) * C + restitution * v_rel.dot(n);

  return J;
};

void PenetrationConstraint::Solve(float dt)
{
  VectorN rhs = J * GetVelocityVector() * -1.f;
  rhs[0] -= bias;

  // 2. 计算 λ 冲量振幅
  VectorN λ = MatrixMN::SolveGaussSeidel(lhs, rhs);

  // 缓存 λ
  VectorN oldLambda = lambda;
  lambda += λ;
  lambda[0] = lambda[0] < 0.f ? 0.f : lambda[0];

  // clamp tangent friction between [-uλ_n, uλ_n]
  if (friction > 0.f)
  {
    float maxFriction = lambda[0] * friction;
    lambda[1] = std::clamp(lambda[1], -maxFriction, maxFriction);
  }

  λ = lambda - oldLambda;

  // 3. 施加冲量
  ApplyImpulse(J.Transpose() * λ);
};