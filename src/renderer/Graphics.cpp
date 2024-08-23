#include "Graphics.h"
#include <cmath>

float SDL_RenderDrawCircle(SDL_Renderer *renderer, float x, float y, float radius)
{
  float offsetx, offsety, d;
  float status;

  offsetx = 0;
  offsety = radius;
  d = radius - 1;
  status = 0;

  while (offsety >= offsetx)
  {
    status += SDL_RenderDrawPoint(renderer, x + offsetx, y + offsety);
    status += SDL_RenderDrawPoint(renderer, x + offsety, y + offsetx);
    status += SDL_RenderDrawPoint(renderer, x - offsetx, y + offsety);
    status += SDL_RenderDrawPoint(renderer, x - offsety, y + offsetx);
    status += SDL_RenderDrawPoint(renderer, x + offsetx, y - offsety);
    status += SDL_RenderDrawPoint(renderer, x + offsety, y - offsetx);
    status += SDL_RenderDrawPoint(renderer, x - offsetx, y - offsety);
    status += SDL_RenderDrawPoint(renderer, x - offsety, y - offsetx);

    if (status < 0)
    {
      status = -1;
      break;
    }

    if (d >= 2 * offsetx)
    {
      d -= 2 * offsetx + 1;
      offsetx += 1;
    }
    else if (d < 2 * (radius - offsety))
    {
      d += 2 * offsety - 1;
      offsety -= 1;
    }
    else
    {
      d += 2 * (offsety - offsetx - 1);
      offsety -= 1;
      offsetx += 1;
    }
  }

  return status;
}

float SDL_RenderFillCircle(SDL_Renderer *renderer, float x, float y, float radius)
{
  float offsetx, offsety, d;
  float status;

  offsetx = 0;
  offsety = radius;
  d = radius - 1;
  status = 0;

  while (offsety >= offsetx)
  {

    status += SDL_RenderDrawLine(renderer, x - offsety, y + offsetx,
                                 x + offsety, y + offsetx);
    status += SDL_RenderDrawLine(renderer, x - offsetx, y + offsety,
                                 x + offsetx, y + offsety);
    status += SDL_RenderDrawLine(renderer, x - offsetx, y - offsety,
                                 x + offsetx, y - offsety);
    status += SDL_RenderDrawLine(renderer, x - offsety, y - offsetx,
                                 x + offsety, y - offsetx);

    if (status < 0)
    {
      status = -1;
      break;
    }

    if (d >= 2 * offsetx)
    {
      d -= 2 * offsetx + 1;
      offsetx += 1;
    }
    else if (d < 2 * (radius - offsety))
    {
      d += 2 * offsety - 1;
      offsety -= 1;
    }
    else
    {
      d += 2 * (offsety - offsetx - 1);
      offsety -= 1;
      offsetx += 1;
    }
  }

  return status;
}

void Uint32ToUint8(Uint32 value, Uint8 *bytes)
{
  for (uint8_t i = 0; i < 4; i++)
  {
    // 先取低位 -> a -> b -> g -> r
    bytes[4 - i - 1] = (value >> (i * 8)) & 0xFF;
  }
}

void Graphics::SetDrawColor(Uint32 color)
{
  Uint8 rgba[4];
  Uint32ToUint8(color, rgba);
  SDL_SetRenderDrawColor(renderer, rgba[0], rgba[1], rgba[2], rgba[3]);
};

float Graphics::width = -1;
float Graphics::height = -1;
Vector2 Graphics::center = Vector2::ZERO;
SDL_Window *Graphics::window = nullptr;
SDL_Renderer *Graphics::renderer = nullptr;

bool Graphics::OpenWindow(const char *title, float x, float y, float w, float h)
{
  width = w;
  height = h;
  center = Vector2(0.5 * w, 0.5 * h);
  window = SDL_CreateWindow(title, x, y, w, h, SDL_WINDOW_SHOWN);
  if (window == nullptr)
    return false;
  renderer = SDL_CreateRenderer(window, -1, 0);
  if (renderer == nullptr)
    return false;

  return true;
}

void Graphics::ClearScreen(Uint32 color)
{
  SetDrawColor(color);
  SDL_RenderClear(renderer);
};

void Graphics::RenderFrame()
{
  SDL_RenderPresent(renderer);
};

void Graphics::CloseWindow()
{
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}

// draw shapes

void Graphics::DrawLine(float x0, float y0, float x1, float y1, Uint32 color)
{
  SetDrawColor(color);
  SDL_RenderDrawLine(renderer, x0, y0, x1, y1);
};

void Graphics::DrawCircle(float x, float y, float radius, Uint32 color)
{
  SetDrawColor(color);
  SDL_RenderDrawCircle(renderer, x, y, radius);
};

void Graphics::DrawFillCircle(float x, float y, float radius, Uint32 color)
{
  SetDrawColor(color);
  SDL_RenderFillCircle(renderer, x, y, radius);
};

void Graphics::DrawCircle(float x, float y, float radius, float angle, Uint32 color)
{
  SetDrawColor(color);
  SDL_RenderDrawCircle(renderer, x, y, radius);
  SDL_RenderDrawLine(renderer, x, y, x + radius * std::cos(angle), y - radius * std::sin(angle));
};

void Graphics::DrawFillCircle(float x, float y, float radius, float angle, Uint32 color) {

};

void Graphics::DrawRect(float x, float y, float width, float height, Uint32 color) {

};

void Graphics::DrawFillRect(float x, float y, float width, float height, Uint32 color) {

};

void Graphics::DrawPolygon(float x, float y, const std::vector<Vector2> &vertices, Uint32 color)
{
  SetDrawColor(color);
  size_t size = vertices.size();
  SDL_FPoint points[size + 1];

  for (size_t i = 0; i < size; i++)
  {
    points[i] = {vertices[i].x, vertices[i].y};
  }
  points[size] = {vertices[0].x, vertices[0].y};

  SDL_RenderDrawLinesF(renderer, points, size + 1);
  SDL_RenderDrawPoint(renderer, x, y);
};

void Graphics::DrawFillPolygon(float x, float y, const std::vector<Vector2> &vertices, Uint32 color) {

};