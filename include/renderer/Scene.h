#ifndef SCENE_H
#define SCENE_H

#include "World.h"
#include <vector>

struct Scene
{

protected:
  bool running;
  World *world;
  bool debug;

public:
  ~Scene();
  // game is loop
  bool IsRunning();
  // open a window
  void Setup();
  virtual void Setup(const char *title, float x, float y, float width, float height);
  // listen for input events
  virtual void Input();
  // update the physics world
  virtual void Update();
  // render the scene
  virtual void Render();
  // destroy all
  void Destroy();
  // frame
  void Frame();
  // loop
  int Loop();
};

#endif