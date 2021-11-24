#include "bseAIContext.h"
#include "bseBitField.h"
#include "bseCommons.h"

namespace bse
{
namespace ai
{
/// \brief In a multithreaded environment an instance of the context must
/// be associated with a thread, so there's no sharing of critical data, like pools
//---------------------------------------------------------------------------------------------------------------------
PathFinderContext::PathFinderContext(AIContext* aiContext) :
  m_aiContext(aiContext)
{
  // TODO: provide descriptor to pre-decide the size of the scene
  openBucket.initBuckets(0, aiContext->maxNumGraphNodes, 10);
  //openList.reserve(aiContext->maxNumGraphNodes / 10); // 10% of the graph size

  closedBits = new bse::BitField(aiContext->maxNumGraphNodes);

#ifdef BSE_ENABLE_PROFILER
  m_profiler = new bse::Profiler();
#endif
}

//---------------------------------------------------------------------------------------------------------------------
PathFinderContext::~PathFinderContext()
{
  delete closedBits;
#ifdef BSE_ENABLE_PROFILER
  delete m_profiler;
#endif
}

//---------------------------------------------------------------------------------------------------------------------
void PathFinderContext::initialize(bse::UInt nodesPoolSize)
{
  (void)nodesPoolSize;
}

//---------------------------------------------------------------------------------------------------------------------
void PathFinderContext::clear()
{
  graphNodesPool.clear();
  openList.clear();
  openBucket.clear();
  closedBits->clear();

  // Let's not clear the profiler here, but do it manually
  // where the context is used.
/*
#ifdef BSE_ENABLE_PROFILER
  m_profiler->clear();
#endif
*/
}

//---------------------------------------------------------------------------------------------------------------------
AIContext::AIContext(bse::UInt contextId, bse::UInt maxNumGraphNodes) :
  m_contextId(contextId)
{
  this->maxNumGraphNodes = maxNumGraphNodes;
  pathFinderContext = new PathFinderContext(this);
}

//---------------------------------------------------------------------------------------------------------------------
AIContext::~AIContext()
{
  delete pathFinderContext;
}

//---------------------------------------------------------------------------------------------------------------------
AIContextManager::AIContextManager(bse::UInt numberOfContexts, bse::UInt maxNumGraphNodes) :
  m_numberOfContexts(numberOfContexts),
  m_maxNumGraphNodes(maxNumGraphNodes)
{
  std::lock_guard<std::mutex> lg(m_contextMut);

  for (bse::UInt i=0; i<numberOfContexts; ++i)
  {
    m_contexts.push_back(new AIContext(i, maxNumGraphNodes));
  }

  m_contextReady.set();
}

//---------------------------------------------------------------------------------------------------------------------
AIContextManager::~AIContextManager()
{
  std::lock_guard<std::mutex> lg(m_contextMut);

  while (!m_contexts.empty())
  {
    AIContext* context = m_contexts.back();
    m_contexts.pop_back();
    delete context;
  }
}

//---------------------------------------------------------------------------------------------------------------------
AIContext* AIContextManager::acquireContext()
{
  m_contextReady.wait();

  std::lock_guard<std::mutex> lg(m_contextMut);

  AIContext* context = m_contexts.back();
  m_contexts.pop_back();

  return context;
}

//---------------------------------------------------------------------------------------------------------------------
void AIContextManager::releaseContext(AIContext* context)
{
  if (context)
  {
    std::lock_guard<std::mutex> lg(m_contextMut);

    // restore context's initial conditions.
    context->pathFinderContext->clear();
    m_contexts.push_back(context);

    m_contextReady.set();
  }
}

//---------------------------------------------------------------------------------------------------------------------
void AIContextManager::reset(bse::UInt numContexts)
{
  while (!m_contexts.empty())
  {
    AIContext* context = m_contexts.back();
    m_contexts.pop_back();
    delete context;
  }

  for (bse::UInt i=0; i<numContexts; ++i)
  {
    m_contexts.push_back(new AIContext(i, m_maxNumGraphNodes));
  }
}

#ifdef BSE_ENABLE_PROFILER
//---------------------------------------------------------------------------------------------------------------------
void AIContextManager::clearProfilerData()
{
  for (ContextsList::iterator iter = m_contexts.begin(); iter != m_contexts.end(); ++iter)
  {
    (*iter)->pathFinderContext->getProfiler()->clear();
  }
}

//---------------------------------------------------------------------------------------------------------------------
bse::ProfilerTask* AIContextManager::getProfilerData() const
{
  // Return the profiling data of the first context.
  if (m_contexts.size() > 0)
  {
    return m_contexts[0]->pathFinderContext->getProfiler()->getRoot();
  }
  return 0;
}
#endif // BSE_ENABLE_PROFILER

}
}
