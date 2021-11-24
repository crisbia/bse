#include "bseAIScene.h"
#include "bseGraph.h"
#include "bseAIContext.h"
#include "bseCommons.h"

namespace bse
{
namespace ai
{

//---------------------------------------------------------------------------------------------------------------------
AIScene* AIScene::create(const AISceneDesc* desc)
{
  return new AIScene(desc);
}

//---------------------------------------------------------------------------------------------------------------------
void AIScene::destroy(AIScene* scene)
{
  delete scene;
}

//---------------------------------------------------------------------------------------------------------------------
AIScene::AIScene(const AISceneDesc* desc) :
                                          m_desc(*desc)
{
  m_aiContextManager = new AIContextManager(m_desc.numContexts, m_desc.maxNumGraphNodes);
}

//---------------------------------------------------------------------------------------------------------------------
AIScene::~AIScene()
{
  for (PathFinderGraphsList::iterator iter = m_pathFinderGraphs.begin(); iter != m_pathFinderGraphs.end(); ++iter)
  {
    delete (*iter);
  }

  delete m_aiContextManager;
}

//---------------------------------------------------------------------------------------------------------------------
Graph* AIScene::createPathFinderGraph(const GraphHeuristic* heuristic)
{
  Graph* graph = new Graph(this, heuristic);
  m_pathFinderGraphs.push_back(graph);
  return graph;
}

//---------------------------------------------------------------------------------------------------------------------
void AIScene::reset(bse::UInt numContexts)
{
  // Reset is mostly for performance profiling reasons, so reset the contexts, so that
  // later timings are consistent.
  m_aiContextManager->reset(numContexts);
}

//---------------------------------------------------------------------------------------------------------------------
void AIScene::destroyPathFinderGraph(Graph* graph)
{
  if (graph)
  {
    size_t iGraph = m_pathFinderGraphs.size();
    for (size_t i=0; i<m_pathFinderGraphs.size(); ++i)
      if (graph == m_pathFinderGraphs[i])
      {
        iGraph = i;
        break;
      }

    if (iGraph < m_pathFinderGraphs.size())
    {
      if (iGraph < m_pathFinderGraphs.size()-1)
      {
        m_pathFinderGraphs[iGraph] = m_pathFinderGraphs.back();
      }

      m_pathFinderGraphs.pop_back();
      delete graph;
    }
  }
}

}
}