#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include "RigidBody.h"
#include "Contact.h"
#include <vector>

struct CollisionDetection
{
  static bool IsColliding(RigidBody *a, RigidBody *b, std::vector<Contact> &contacts);
  static bool IsCollidingCircleCircle(RigidBody *a, RigidBody *b, std::vector<Contact> &contacts);
  /**
   * @param a is polygon
   * @param b is circle
   */
  static bool IsCollidingPolygonCircle(RigidBody *a, RigidBody *b, std::vector<Contact> &contacts);
  static bool IsCollidingPolygonPolygon(RigidBody *a, RigidBody *b, std::vector<Contact> &contacts);
};

#endif