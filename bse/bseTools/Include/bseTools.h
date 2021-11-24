#ifndef _BSE_TOOLS_H_INCLUDED
#define _BSE_TOOLS_H_INCLUDED

#include "bsePhysics.h"

namespace bse
{

class ToolsDesc
{
public:
  ToolsDesc()
  {
    randSeed = 0;
  }

  // TODO here, all the custom things, like logger, debug draw, memory manager...
  unsigned int randSeed;
};

// Abstract class that handles all the public interactions with the library.
class ToolsHandler
{
public:
  virtual phx::Scene* createPhysicsScene(phx::SceneDesc* desc) = 0;
  virtual void      destroyPhysicsScene(phx::Scene* scene) = 0;
};

extern ToolsHandler* toolsInit(ToolsDesc* desc);
extern void toolsShutDown();

}

#endif // _BSE_TOOLS_H_INCLUDED