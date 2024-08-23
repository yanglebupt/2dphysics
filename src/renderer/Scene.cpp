#include "Scene.h"
#include "Graphics.h"
#include <iostream>

bool Scene::IsRunning()
{
  return running;
};

void Scene::Destroy()
{
  Graphics::CloseWindow();
};

Scene::~Scene()
{
  std::cout << "Scene destructor" << std::endl;
  delete world;
  Destroy();
};

void Scene::Setup()
{
  Setup("Demo", 100, 100, 800, 600);
};

void Scene::Setup(const char *title, float x, float y, float width, float height) {};
void Scene::Input() {};
void Scene::Update() {};
void Scene::Render() {};

void Scene::Frame()
{
  Input();
  Graphics::ClearScreen(0x242424FF);
  Update();
  Render();
  Graphics::RenderFrame();
};

int Scene::Loop()
{
  while (IsRunning())
  {
    Frame();
  }
  Destroy();
  return 0;
};
