#ifndef _BSE_AISCENE_H_INCLUDED
#define _BSE_AISCENE_H_INCLUDED

#include <vector>
#include "bseGraph.h"

namespace bse
{
namespace ai
{

class AIContextManager;

class AISceneDesc
{
public:
  AISceneDesc() :
      numContexts(1),
      maxNumGraphNodes(0),
      maxNumGraphConnections(0)
  {
  }
  int numContexts;
  int maxNumGraphNodes;
  int maxNumGraphConnections;
};

class AIScene
{
public:
  static AIScene* create(const AISceneDesc* desc);
  static void destroy(AIScene* scene);
private:
  AIScene(const AISceneDesc* desc);
  virtual ~AIScene();

public:
  Graph* createPathFinderGraph(const GraphHeuristic* heuristic);
  void destroyPathFinderGraph(Graph* graph);

  AIContextManager* getContextManager() const { return m_aiContextManager; }

  void reset(bse::UInt numContexts);

private:
  typedef std::vector<Graph*> PathFinderGraphsList;
  
  PathFinderGraphsList m_pathFinderGraphs;
  AIContextManager*    m_aiContextManager;
  AISceneDesc          m_desc;
};

}
}

#endif // _BSE_AISCENE_H_INCLUDED
