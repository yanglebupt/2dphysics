#ifndef WORLD_H
#define WORLD_H

#include <functional>
#include <vector>
#include "RigidBody.h"
#include "Contact.h"
#include "Constraint.h"

struct World
{
private:
  float gravity;
  Vector2 gravityDirection;
  std::vector<RigidBody *> bodies;
  // forces applied to every rigidbody
  std::vector<Vector2> forces;
  // torques applied to every rigidbody
  std::vector<float> torques;
  // constraints needed to solve
  std::vector<Constraint *> constraints;
  int constraintSolveIterations = 5;
  float penetrationConstraintBias = 0.2f;

public:
  World(float gravity, Vector2 gravityDirection);
  World(float gravity);
  World();
  ~World();

  std::vector<RigidBody *> GetBodies() const;

  void AddBody(RigidBody *body);
  // add force to every rigidbody
  void AddForce(const Vector2 &force);
  // add torque to every rigidbody
  void AddTorque(float torque);
  void AddConstraint(Constraint *constraint);

  void Update(float dt, std::function<void(const Contact &contact)> callback = [](const Contact &_contact) {});

  // check all rigidbodies with other for collision
  void CheckCollision(std::function<void(const Contact &contact)> callback = [](const Contact &_contact) {});
};

#endif