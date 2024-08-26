#ifndef PENETRATIONCONSTRAINTDEMO_H
#define PENETRATIONCONSTRAINTDEMO_H

#include "ApplicationBase.cpp"

/**
 * 只展示基本的碰撞检测，注意使用的是 World.cpp 中使用的是 PenetrationConstraint;
 **/
struct PenetrationConstraintDemo : ApplicationBase
{
  void Setup(const char *title, float x, float y, float width, float height) override
  {
    debug = true;
    running = Graphics::OpenWindow(title, x, y, width, height);
    world = new World();
    world->enablePenetrationConstraint = true;

    RigidBody *bigBall = new RigidBody(BoxShape(150, 150), Graphics::center, 0);
    bigBall->rotation = -M_PI / 8;
    bigBall->restitution = 0.;
    bigBall->friction = 0.;
    world->AddBody(bigBall);

    float wallWidth = 20, wallHeight = 20;

    RigidBody *floor = new RigidBody(BoxShape(Graphics::width - wallWidth * 2.2, wallHeight), Graphics::center.x, Graphics::height - wallHeight * 0.6, 0);
    floor->restitution = 0.;
    floor->friction = 0.;
    world->AddBody(floor);

    RigidBody *leftFloor = new RigidBody(BoxShape(wallWidth, Graphics::height), wallWidth * 0.6, Graphics::center.y, 0);
    leftFloor->restitution = 0.;
    leftFloor->friction = 0.;
    world->AddBody(leftFloor);

    RigidBody *rightFloor = new RigidBody(BoxShape(wallWidth, Graphics::height), Graphics::width - wallWidth * 0.6, Graphics::center.y, 0);
    rightFloor->restitution = 0.;
    rightFloor->friction = 0.;
    world->AddBody(rightFloor);
  };

  void Input() override
  {
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
      switch (event.type)
      {
      case SDL_QUIT:
      {
        running = false;
        break;
      }
      case SDL_KEYDOWN:
      {
        if (event.key.keysym.sym == SDLK_ESCAPE)
          running = false;
        if (event.key.keysym.sym == SDLK_d)
          debug = !debug;
        break;
      }
      case SDL_MOUSEBUTTONDOWN:
      {
        int x, y;
        SDL_GetMouseState(&x, &y);
        float p = 1. * rand() / RAND_MAX;
        RigidBody *body;
        // if (p <= 0.3)
        // {
        //   body = new RigidBody(CircleShape(20), x, y, 1);
        //   body->restitution = 0.7;
        //   body->friction = 0.4;
        // }
        // else if (p <= 0.7)
        {
          body = new RigidBody(BoxShape(40, 40), x, y, 1);
          body->restitution = 0.0;
          body->friction = 0.1;
        }
        // else
        // {
        //   std::vector<Vector2> vertices = {
        //       {20, 60},
        //       {40, 20},
        //       {20, -60},
        //       {-20, -60},
        //       {-40, 20},
        //   };
        //   body = new RigidBody(PolygonShape(vertices), x, y, 1);
        //   body->restitution = 0.1;
        // }
        world->AddBody(body);
        break;
      }
      }
    }
  };
};

#endif