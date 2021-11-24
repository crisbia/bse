#include "SettingsStack.h"
#include "bseScene.h"

void SceneSettingsEntry::save(bse::phx::Scene* scene)
{
  gravity = scene->getGravity();
  friction = scene->getDefaultMaterial()->getFriction();
  restitution = scene->getDefaultMaterial()->getRestitution();
}

void SceneSettingsEntry::restore(bse::phx::Scene* scene)
{
  scene->setGravity(gravity);
  scene->getDefaultMaterial()->setFriction(friction);
  scene->getDefaultMaterial()->setRestitution(restitution);
}


SceneSettingsStack::SceneSettingsStack(bse::phx::Scene* scene) :
  m_scene(scene)
{

}

void SceneSettingsStack::push()
{
  SceneSettingsEntry entry;
  entry.save(m_scene);
  m_stack.push(entry);
}

void SceneSettingsStack::pop()
{
  if (m_stack.empty())
    return;

  SceneSettingsEntry& entry = m_stack.top();
  entry.restore(m_scene);

  // Get rid of the top stack value.
  m_stack.pop();
}
