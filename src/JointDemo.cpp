#ifndef JOINTDEMO_H
#define JOINTDEMO_H

#include "ApplicationBase.cpp"
#include "JointConstraint.h"

/**
 * 展示 JointConstraint + collision 的效果，注意使用的是 World.cpp 中使用的是 contact.ResolveCollision();
 **/
struct JointDemo : ApplicationBase
{
  const static int NUM_BODIES = 8;
  void Setup(const char *title, float x, float y, float width, float height) override
  {
    debug = false;
    running = Graphics::OpenWindow(title, x, y, width, height);
    world = new World();
    world->enablePenetrationConstraint = false;

    // add bodies and joint constraints
    RigidBody *parentBody;
    for (size_t i = 0; i < NUM_BODIES; i++)
    {
      float mass = i == 0 ? 0.f : 1.f;
      RigidBody *body = new RigidBody(BoxShape(30, 30), Graphics::width * 0.5 - (i * 40), 100, mass);
      if (i > 0)
      {
        JointConstraint *joint = new JointConstraint(parentBody, body, parentBody->position, 0.2f);
        world->AddConstraint(joint);
      }
      world->AddBody(body);
      parentBody = body;
    }

    float wallWidth = 20, wallHeight = 20;

    RigidBody *floor = new RigidBody(BoxShape(Graphics::width - wallWidth * 2.2, wallHeight), Graphics::center.x, Graphics::height - wallHeight * 0.6, 0);
    floor->restitution = 0.2;
    world->AddBody(floor);

    RigidBody *leftFloor = new RigidBody(BoxShape(wallWidth, Graphics::height), wallWidth * 0.6, Graphics::center.y, 0);
    leftFloor->restitution = 0.2;
    world->AddBody(leftFloor);

    RigidBody *rightFloor = new RigidBody(BoxShape(wallWidth, Graphics::height), Graphics::width - wallWidth * 0.6, Graphics::center.y, 0);
    rightFloor->restitution = 0.2;
    world->AddBody(rightFloor);
  };

  void Render() override
  {
    ApplicationBase::Render();
    auto bodies = world->GetBodies();
    for (size_t i = 1; i < NUM_BODIES; i++)
    {
      auto preBody = bodies[i - 1];
      auto body = bodies[i];
      Graphics::DrawLine(preBody->position.x, preBody->position.y, body->position.x, body->position.y, 0xFF0000FF);
    }
  };
};

#endif