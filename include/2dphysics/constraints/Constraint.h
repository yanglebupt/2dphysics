#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "RigidBody.h"
#include "MatrixMN.h"
#include <tuple>

/**
 * constraint between two rigid bodies "a" and "b"
 * with a local solve
 **/
struct Constraint
{
public:
  RigidBody *a;
  RigidBody *b;
  float biasBeta;
  MatrixMN InvM;
  // pre computed
  float bias;
  // pre computed
  MatrixMN J;
  // pre computed
  MatrixMN lhs;
  // cached
  VectorN lambda;

  Constraint(RigidBody *a, RigidBody *b, float biasBeta = 0.3f);

  /**
   * reciprocal mass diagonal matrix of rigid body "a" and "b"
   * [1/ma  0      0      0      0          0]
   * [0      1/ma  0      0      0          0]
   * [0      0      1/Ia  0      0          0]
   * [0      0      0      1/mb  0          0]
   * [0      0      0      0      1/mb      0]
   * [0      0      0      0      0      1/Ib]
   */
  MatrixMN GetInvMMatrix();

  /**
   * velocity of rigid body "a" and "b"
   * [va_x, va_y, wa, vb_x, vb_y, wb].T
   */
  VectorN GetVelocityVector();

  void ApplyImpulse(VectorN impulse);

  /**
   * 计算梯度 J （冲量方向），J=∂C/∂P and return (M, N) matrix
   **/
  virtual MatrixMN GetJ(float dt) = 0;

  virtual void PreSolve(float dt);
  // calculate solved impulse and apply
  virtual void Solve(float dt);
  virtual void PostSolve(float dt);
};

#endif