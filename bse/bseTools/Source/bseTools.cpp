#include "bseTools.h"

#include <list>

namespace bse
{

class ToolsHandlerImpl : public ToolsHandler
{
public:
  ToolsHandlerImpl(ToolsDesc* desc);

public:
  // Handler interface implementation.
  virtual phx::Scene* createPhysicsScene(phx::SceneDesc* desc);
  virtual void      destroyPhysicsScene(phx::Scene* scene);

  // Tools objects management.
public:
  void cleanUpTools();
protected:
  typedef std::list<phx::Scene*> PhysicsScenesList;
  PhysicsScenesList m_physicsScenes;
};


ToolsHandlerImpl::ToolsHandlerImpl(ToolsDesc* desc)
{

}

phx::Scene* ToolsHandlerImpl::createPhysicsScene(phx::SceneDesc* desc)
{
  phx::Scene* scene = phx::Scene::create(desc);
  m_physicsScenes.push_back(scene);
  return scene;
}

void ToolsHandlerImpl::destroyPhysicsScene(phx::Scene* scene)
{
  m_physicsScenes.remove(scene);
  phx::Scene::destroy(scene);
}

void ToolsHandlerImpl::cleanUpTools()
{
  for (PhysicsScenesList::iterator iter = m_physicsScenes.begin(); iter != m_physicsScenes.end(); ++iter)
  {
    phx::Scene::destroy(*iter);
  }  
}

static ToolsHandlerImpl* gToolsHandler = 0;

ToolsHandler* toolsInit(ToolsDesc* desc)
{
  if (gToolsHandler==0)
    gToolsHandler = new ToolsHandlerImpl(desc);

  return gToolsHandler;
}

void toolsShutDown()
{
  if (gToolsHandler)
    gToolsHandler->cleanUpTools();

  delete gToolsHandler;
  gToolsHandler = 0;
}

}