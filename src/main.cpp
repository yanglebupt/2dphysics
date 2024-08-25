#include "SDL2/SDL.h"
#include <iostream>
#include "CollisionDemo.cpp"
#include "JointDemo.cpp"
#include "PenetrationConstraintDemo.cpp"

int main(int argc, char **argv)
{
  Scene *demo = new PenetrationConstraintDemo();
  demo->Setup();
  demo->Loop();
  delete demo;
  return 0;
}
