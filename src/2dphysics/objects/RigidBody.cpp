#include "RigidBody.h"
#include "PolygonShape.h"
#include "Constants.h"

RigidBody::RigidBody(const Shape &shape, float x, float y, float mass, Multiplier multiplier) : Particle(x, y, mass, multiplier.linear),
                                                                                                shape(shape.New()), angularMultiplier(multiplier.angular)
{
  SetInvI(mass * shape.GetMomentOfInertia());
  SetDefault();
}

RigidBody::RigidBody(const Shape &shape, Vector2 position, float mass, Multiplier multiplier) : Particle(position, mass, multiplier.linear),
                                                                                                shape(shape.New()), angularMultiplier(multiplier.angular)
{
  SetInvI(mass * shape.GetMomentOfInertia());
  SetDefault();
}

RigidBody::RigidBody(const Shape &shape, float x, float y, float mass, float momentOfInertia, Multiplier multiplier) : Particle(x, y, mass, multiplier.linear),
                                                                                                                       shape(shape.New()), angularMultiplier(multiplier.angular)
{
  SetInvI(mass * momentOfInertia);
  SetDefault();
}

RigidBody::RigidBody(const Shape &shape, Vector2 position, float mass, float momentOfInertia, Multiplier multiplier) : Particle(position, mass, multiplier.linear),
                                                                                                                       shape(shape.New()), angularMultiplier(multiplier.angular)
{
  SetInvI(mass * momentOfInertia);
  SetDefault();
}

void RigidBody::SetInvI(float I)
{
  invI = I != 0.f ? angularMultiplier / I : 0.f;
};

void RigidBody::SetDefault()
{
  restitution = .5f;
  friction = 0.1f;
  rotation = 0.f;
  angularVelocity = 0.f;
  angularAcceleration = 0.f;
  sumTorque = 0.f;
  angularMultiplier = 1.f;
  velocity = Vector2::ZERO;
  acceleration = Vector2::ZERO;
  sumForce = Vector2::ZERO;

  shape->UpdateVertices(rotation, position);
};

void RigidBody::AddTorque(float torque)
{
  if (IsStatic())
    return;
  sumTorque += torque;
};

Vector2 RigidBody::GetResultantVelocityFromP(const Vector2 &point)
{
  return GetResultantVelocityFromV(point - position);
};

Vector2 RigidBody::GetResultantVelocityFromV(const Vector2 &r)
{
  return velocity + r.crossZ(angularVelocity);
};

void RigidBody::ApplyImpulse(const Vector2 &J)
{
  Particle::ApplyImpulse(J);
};

void RigidBody::ApplyImpulse(float J)
{
  if (IsStatic())
    return;
  angularVelocity += J * invI;
};

void RigidBody::ApplyImpulseV(const Vector2 &J, const Vector2 &r)
{
  if (IsStatic())
    return;
  velocity += J * invMass;
  angularVelocity += r.cross(J) * invI; // 注意 cross 不满足交换律
};

void RigidBody::ApplyImpulseP(const Vector2 &J, const Vector2 &point)
{
  ApplyImpulseV(J, point - position);
};

void RigidBody::ClearTorque()
{
  sumTorque = 0;
};

void RigidBody::IntegrateAngular(float dt)
{
  if (IsStatic())
    return;

  angularAcceleration = sumTorque * invI * TURNS_PER_ANGLE;
  angularVelocity += dt * angularAcceleration;
  angularVelocity = std::min(angularVelocity, MAX_ANGULAR_VELOCITY);
  rotation += dt * angularVelocity;

  ClearTorque();
}

void RigidBody::IntegrateForce(float dt)
{
  if (IsStatic())
    return;

  acceleration = sumForce * invMass * PIXELS_PER_METER;
  velocity += dt * acceleration;
  velocity = velocity.min(MAX_VELOCITY);

  angularAcceleration = sumTorque * invI * TURNS_PER_ANGLE;
  angularVelocity += dt * angularAcceleration;
  angularVelocity = std::min(angularVelocity, MAX_ANGULAR_VELOCITY);

  ClearForce();
  ClearTorque();
};

void RigidBody::IntegrateVelocity(float dt)
{
  if (!IsStatic())
  {
    position += dt * velocity;
    rotation += dt * angularVelocity;
  }
  shape->UpdateVertices(rotation, position);
};

void RigidBody::Update(float dt)
{
  IntegrateLinear(dt);
  IntegrateAngular(dt);
  shape->UpdateVertices(rotation, position);
};

Vector2 RigidBody::WorldPointToLocalPoint(const Vector2 &point) const
{
  return (point - position).rotate(-rotation);
};

Vector2 RigidBody::LocalPointToWorldPoint(const Vector2 &point) const
{
  return point.rotate(rotation) + position;
};

RigidBody::~RigidBody()
{
  delete shape;
};