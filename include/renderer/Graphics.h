#ifndef GRAPHICS_H
#define GRAPHICS_H
#include "SDL2/SDL.h"
#include <vector>
#include "Vector2.h"

/**
 * static method for draw shape
 */
struct Graphics
{
private:
  static void SetDrawColor(Uint32 color);

public:
  static float width;
  static float height;
  static Vector2 center;
  static SDL_Window *window;
  static SDL_Renderer *renderer;
  // open a window return true else return false
  static bool OpenWindow(const char *title, float x, float y, float width, float height);
  static void CloseWindow();
  static void ClearScreen(Uint32 color);
  static void RenderFrame();

  // draw shapes
  static void DrawLine(float x0, float y0, float x1, float y1, Uint32 color);
  static void DrawCircle(float x, float y, float radius, Uint32 color);
  static void DrawFillCircle(float x, float y, float radius, Uint32 color);
  static void DrawCircle(float x, float y, float radius, float angle, Uint32 color);
  static void DrawFillCircle(float x, float y, float radius, float angle, Uint32 color);
  static void DrawRect(float x, float y, float width, float height, Uint32 color);
  static void DrawFillRect(float x, float y, float width, float height, Uint32 color);
  static void DrawPolygon(float x, float y, const std::vector<Vector2> &vertices, Uint32 color);
  static void DrawFillPolygon(float x, float y, const std::vector<Vector2> &vertices, Uint32 color);
};

#endif