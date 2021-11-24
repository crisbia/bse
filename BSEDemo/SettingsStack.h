#ifndef _SCENE_SETTINGS_STACK_INCLUDED
#define _SCENE_SETTINGS_STACK_INCLUDED

#include "bseMath.h"
#include <stack>

namespace bse
{
namespace phx
{
class Scene;
}
}

class SceneSettingsEntry
{
public:
  void save(bse::phx::Scene* scene);
  void restore(bse::phx::Scene* scene);
public:
  bse::Vec2 gravity;
  float   friction;
  float   restitution;
};

class SceneSettingsStack
{
public:
  SceneSettingsStack(bse::phx::Scene* scene=0);
  void push();
  void pop();

  void setScene(bse::phx::Scene* scene) { m_scene = scene; }

protected:
  std::stack<SceneSettingsEntry> m_stack;
  bse::phx::Scene* m_scene;
};

#endif // _SCENE_SETTINGS_STACK_INCLUDED