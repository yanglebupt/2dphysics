#ifndef APPLICATIONBASE_H
#define APPLICATIONBASE_H

#include "Scene.h"
#include "CircleShape.h"
#include "BoxShape.h"
#include "PolygonShape.h"
#include "ATimer.h"
#include "Graphics.h"

struct ApplicationBase : Scene
{
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
        if (p <= 0.3)
        {
          body = new RigidBody(CircleShape(20), x, y, 1);
          body->restitution = 0.7;
          body->friction = 0.4;
        }
        else if (p <= 0.7)
        {
          body = new RigidBody(BoxShape(40, 40), x, y, 1);
          body->restitution = 0.3;
        }
        else
        {
          std::vector<Vector2> vertices = {
              {20, 60},
              {40, 20},
              {20, -60},
              {-20, -60},
              {-40, 20},
          };
          body = new RigidBody(PolygonShape(vertices), x, y, 1);
          body->restitution = 0.2;
        }
        world->AddBody(body);
        break;
      }
      }
    }
  };

  void Update() override
  {
    float dt = ATimer::GetFixedDeltaTime();
    world->Update(dt, [this](const Contact &contact)
                  {
                  if (debug)
                  {
                    Graphics::DrawFillCircle(contact.start.x, contact.start.y, 3, 0xFFFF00FF);
                    Graphics::DrawFillCircle(contact.end.x, contact.end.y, 3, 0xFFFF00FF);
                    Graphics::DrawLine(contact.start.x, contact.start.y,
                                        contact.start.x + contact.normal.x * 15, contact.start.y + contact.normal.y * 15,
                                        0xFFFF00FF);
                  } });
  };

  void Render() override
  {
    for (auto body : world->GetBodies())
    {
      Uint32 color = debug ? (body->isColliding ? 0xFF0000FF : 0xFFFFFFFF) : 0xFFFFFFFF;
      switch (body->shape->GetType())
      {
      case ShapeType::CIRCLE:
      {
        CircleShape *shape = dynamic_cast<CircleShape *>(body->shape);
        Graphics::DrawCircle(body->position.x, body->position.y, shape->radius, body->rotation, color);
        break;
      }
      case ShapeType::BOX:
      case ShapeType::POLYGON:
      {
        PolygonShape *shape = dynamic_cast<PolygonShape *>(body->shape);
        Graphics::DrawPolygon(body->position.x, body->position.y, shape->worldVertices, color);
      }
      }
    }
  };
};

#endif