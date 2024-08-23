#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include "RigidBody.h"
#include "Contact.h"

struct CollisionDetection
{
  static bool IsColliding(RigidBody *a, RigidBody *b, Contact &contact);
  static bool IsCollidingCircleCircle(RigidBody *a, RigidBody *b, Contact &contact);
  /**
   * @param a is polygon
   * @param b is circle
   */
  static bool IsCollidingPolygonCircle(RigidBody *a, RigidBody *b, Contact &contact);
  static bool IsCollidingPolygonPolygon(RigidBody *a, RigidBody *b, Contact &contact);
};

#endif