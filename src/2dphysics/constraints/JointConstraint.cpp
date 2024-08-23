#include "JointConstraint.h"

JointConstraint::JointConstraint(RigidBody *a, RigidBody *b, Vector2 anchor, float biasBeta) : Constraint(a, b, biasBeta)
{
  anchorInALocal = a->WorldPointToLocalPoint(anchor);
  anchorInBLocal = b->WorldPointToLocalPoint(anchor);
  lambda = VectorN(1);
}

MatrixMN JointConstraint::GetJ(float dt)
{
  MatrixMN J(1, 6);

  Vector2 pa = a->LocalPointToWorldPoint(anchorInALocal);
  Vector2 pb = b->LocalPointToWorldPoint(anchorInBLocal);

  Vector2 ra = pa - a->position;
  Vector2 rb = pb - b->position;

  Vector2 pa_b = 2.f * (pa - pb);

  // when constraints satisfied pa = pb, s.t || pa-pb || = 0
  float C = 0.25f * pa_b.lengthSq();

  J[0][0] = pa_b[0];
  J[0][1] = pa_b[1];

  J[0][2] = ra.cross(pa_b);

  J[0][3] = pa_b[0] * -1.f;
  J[0][4] = pa_b[1] * -1.f;

  J[0][5] = pa_b.cross(rb);

  // compute bias term (baumgarte stabilization)
  bias = (biasBeta / dt) * C;

  return J;
};