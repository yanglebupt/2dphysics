#include "ATimer.h"
#include "SDL2/SDL.h"
#include "Constants.h"

int ATimer::timePreviousFrame = 0;
float ATimer::timeScale = 1.f;

float ATimer::GetFixedDeltaTime()
{
  // each frame has fixed time，看看 update 的间隔时间是否小于物理上一帧的时间
  int timeToWait = MS_PER_FRAME - (SDL_GetTicks() - timePreviousFrame);
  // wait for fixed physics update time
  if (timeToWait > 0)
    SDL_Delay(timeToWait);

  int nowTime = SDL_GetTicks();
  float deltaTime = (nowTime - timePreviousFrame) / 1000.f; // ms --> s
  /**
   * 有些操作会导致画面不更新，但是 SDL_GetTicks 得到的仍是最新的时间
   * 例如拖拽窗口，此时画面静止，但是 SDL_GetTicks 仍然再增长
   */
  if (deltaTime > MS_PER_FRAME * 1e-3)
    deltaTime = MS_PER_FRAME * 1e-3;
  timePreviousFrame = nowTime;

  return deltaTime;
}
