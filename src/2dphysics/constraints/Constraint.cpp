#include "Constraint.h"

Constraint::Constraint(RigidBody *a, RigidBody *b, float biasBeta) : a(a), b(b), biasBeta(biasBeta)
{
  InvM = GetInvMMatrix();
};

MatrixMN Constraint::GetInvMMatrix()
{
  MatrixMN invM(6, 6);
  invM[0][0] = invM[1][1] = a->invMass;
  invM[2][2] = a->invI;

  invM[3][3] = invM[4][4] = b->invMass;
  invM[5][5] = b->invI;

  return invM;
}

VectorN Constraint::GetVelocityVector()
{
  VectorN vec(6);

  vec[0] = a->velocity.x;
  vec[1] = a->velocity.y;
  vec[2] = a->angularVelocity;

  vec[3] = b->velocity.x;
  vec[4] = b->velocity.y;
  vec[5] = b->angularVelocity;

  return vec;
}

void Constraint::ApplyImpulse(VectorN impulse)
{
  a->ApplyImpulse(Vector2(impulse[0], impulse[1]));
  a->ApplyImpulse(impulse[2]);

  b->ApplyImpulse(Vector2(impulse[3], impulse[4]));
  b->ApplyImpulse(impulse[5]);
};

void Constraint::PreSolve(float dt)
{
  // 1. 计算梯度 J （冲量方向），J=∂C/∂P，由不同的子类实现
  // 我们只需要在 PreSolve 中计算一次，后面 Solve 多次迭代使用即可
  J = GetJ(dt);
  MatrixMN JT = J.Transpose();
  lhs = J * InvM * JT;

  // 2. apply cached lambda from previous solve
  ApplyImpulse(JT * lambda);
};

void Constraint::Solve(float dt)
{
  VectorN rhs = (J * GetVelocityVector() + bias) * -1.f; // MxN Nx1 = Mx1

  // 2. 计算 λ 冲量振幅
  VectorN λ = MatrixMN::SolveGaussSeidel(lhs, rhs); // MxM Mx1 = Mx1

  // 缓存 λ
  lambda += λ;

  // 3. 施加冲量
  ApplyImpulse(J.Transpose() * λ); // NxM Mx1 = Nx1
};

void Constraint::PostSolve(float dt) {};
