#include "SDL2/SDL.h"
#include <iostream>
#include "CollisionDemo.cpp"
#include "JointDemo.cpp"

int main(int argc, char **argv)
{
  Scene *demo = new JointDemo();
  demo->Setup();
  demo->Loop();
  delete demo;
  return 0;
}
