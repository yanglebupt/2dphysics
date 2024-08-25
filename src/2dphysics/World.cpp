#include "World.h"
#include "Constants.h"
#include "Force.h"
#include "Contact.h"
#include "CollisionDetection.h"
#include "PenetrationConstraint.h"

World::World(float gravity, Vector2 gravityDirection) : gravity(gravity), gravityDirection(gravityDirection) {};
World::World(float gravity) : gravity(gravity), gravityDirection(GRAVITYDIRECTION) {};
World::World() : gravity(GRAVITY), gravityDirection(GRAVITYDIRECTION) {};
World::~World()
{
  for (auto body : bodies)
  {
    delete body;
  }
  for (auto constraint : constraints)
  {
    delete constraint;
  }

  bodies.clear();
  constraints.clear();
  forces.clear();
  torques.clear();
  std::vector<RigidBody *>().swap(bodies);
  std::vector<Constraint *>().swap(constraints);
  std::vector<Vector2>().swap(forces);
  std::vector<float>().swap(torques);
};

std::vector<RigidBody *> World::GetBodies() const
{
  return bodies;
};

void World::AddBody(RigidBody *body)
{
  bodies.push_back(body);
};

void World::AddForce(const Vector2 &force)
{
  forces.push_back(force);
};

void World::AddTorque(float torque)
{
  torques.push_back(torque);
};

void World::AddConstraint(Constraint *constraint)
{
  constraints.push_back(constraint);
};

void World::Update(float dt, std::function<void(const Contact &contact)> callback)
{
  // 添加力和扭矩
  for (auto &body : bodies)
  {
    Vector2 wei = Force::GenerateGravityForce(*body);
    body->AddForce(wei);

    for (auto force : forces)
      body->AddForce(force);
    for (auto torque : torques)
      body->AddTorque(torque);

    body->isColliding = false;
  }

  // 先只更新速度
  for (auto &body : bodies)
  {
    body->IntegrateForce(dt);
  }

  std::vector<PenetrationConstraint> penetrations; // 局部变量，自动释放
  if (enablePenetrationConstraint)
  {
    CheckCollision([callback, &penetrations, this](const Contact &contact)
                   { 
      callback(contact);
      // 添加侵入约束，局部变量，自动释放，不要 new
      penetrations.push_back(PenetrationConstraint(contact, penetrationConstraintBias)); });
  }

  // 求解约束，施加冲量，再次更新速度
  for (auto &constraint : constraints)
  {
    constraint->PreSolve(dt);
  }
  for (auto &constraint : penetrations)
  {
    constraint.PreSolve(dt);
  }

  for (size_t i = 0; i < constraintSolveIterations; i++)
  {
    for (auto &constraint : constraints)
    {
      constraint->Solve(dt);
    }
    for (auto &constraint : penetrations)
    {
      constraint.Solve(dt);
    }
  }

  for (auto &constraint : constraints)
  {
    constraint->PostSolve(dt);
  }
  for (auto &constraint : penetrations)
  {
    constraint.PostSolve(dt);
  }

  // 最后更新位置
  for (auto &body : bodies)
  {
    body->IntegrateVelocity(dt);
  }

  if (!enablePenetrationConstraint)
    CheckCollision();
};

void World::CheckCollision(std::function<void(const Contact &contact)> callback)
{
  for (size_t i = 0; i < bodies.size(); i++)
  {
    for (size_t j = i + 1; j < bodies.size(); j++)
    {
      RigidBody *a = bodies[i];
      RigidBody *b = bodies[j];
      Contact contact;
      if (CollisionDetection::IsColliding(a, b, contact))
      {
        contact.ResolveCollision();
        a->isColliding = b->isColliding = true;
        callback(contact);
      }
    }
  }
};