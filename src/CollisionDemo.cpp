#ifndef COLLISIONDEMO_H
#define COLLISIONDEMO_H

#include "ApplicationBase.cpp"

/**
 * 只展示基本的碰撞检测，注意使用的是 World.cpp 中使用的是 contact.ResolveCollision();
 **/
struct CollisionDemo : ApplicationBase
{
  void Setup(const char *title, float x, float y, float width, float height) override
  {
    debug = false;
    running = Graphics::OpenWindow(title, x, y, width, height);
    world = new World();
    world->enablePenetrationConstraint = false;

    RigidBody *bigBall = new RigidBody(BoxShape(150, 150), Graphics::center, 0);
    bigBall->rotation = -M_PI / 8;
    bigBall->restitution = 0.2;
    world->AddBody(bigBall);

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
};

#endif