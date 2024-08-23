#include <cmath>
#include "Constants.h"

bool IsFloatEqual(float a, float b)
{
  return std::abs(a - b) < EPSILON;
}
