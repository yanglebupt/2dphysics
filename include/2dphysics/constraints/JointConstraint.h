#ifndef JOINTCONSTRAINT_H
#define JOINTCONSTRAINT_H

#include "Constraint.h"

/**
 * JointConstraint with a fixed joint point
 */
struct JointConstraint : Constraint
{
private:
  Vector2 anchorInALocal;
  Vector2 anchorInBLocal;

public:
  JointConstraint(RigidBody *a, RigidBody *b, Vector2 anchor, float biasBeta = 0.3f);

  /**
   * J = [2(pa - pb), 2(ra X (pa - pb)), 2(pb - pa), 2(rb X (pb - pa))]
   */
  MatrixMN GetJ(float dt) override;
};

#endif