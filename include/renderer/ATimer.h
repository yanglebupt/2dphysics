#ifndef ATIMER_H
#define ATIMER_H

struct ATimer
{
private:
  // return ms
  static int timePreviousFrame;
  static float timeScale;

public:
  // return s
  static float GetFixedDeltaTime();
  static float GetFixedElapsedDeltaTime();
  static float GetDeltaTime();
  static float GetElapsedTime();
};

#endif