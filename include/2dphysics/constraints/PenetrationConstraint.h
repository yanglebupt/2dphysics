#ifndef PENETRATIONCONSTRAINT_H
#define PENETRATIONCONSTRAINT_H

#include "Constraint.h"
#include "Contact.h"

struct PenetrationConstraint : Constraint
{
private:
  Vector2 startInBLocal;
  Vector2 normalInALocal;
  Vector2 endInALocal;
  float friction;

public:
  PenetrationConstraint(Contact contact, float biasBeta = 0.3f);

  /**
   * J = [-n, -ra X n, n, rb X n]
   *     [-t, -ra X t, t, rb X t]
   */
  MatrixMN GetJ(float dt) override;

  void Solve(float dt) override;
};

#endif