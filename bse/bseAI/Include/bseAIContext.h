#ifndef _BSE_AICONTEXT_H_INCLUDED
#define _BSE_AICONTEXT_H_INCLUDED


#include "bseTypes.h"
#include "bsePools.h"
#include "bseGraph.h"
#include "bseProfilingTools.h"
#include "bseThreadingTools.h"

#include <deque>
#include <mutex>

namespace bse
{
  class BitField;

namespace ai
{

class AIContext;
class PathFinderContext;

typedef FrameObjectsPool<GraphNodeGhost, 1024> GraphNodesPool;


/**
 *  \brief In a multithreaded environment an instance of the context must
 *  be associated with a thread, so there's no sharing of critical data, like pools
 *
 */
class PathFinderContext
{
public:
  PathFinderContext(AIContext* aiContext);
  virtual ~PathFinderContext();

  void clear();

public:
  /**
   *  \brief Prepare structure with a reasonable size for the scenario.
   */
  void initialize(bse::UInt nodesPoolSize);

  GraphNodesPool   graphNodesPool;
  OpenListHeap     openList;
  OpenListBucket   openBucket;
  bse::BitField*   closedBits;

#ifdef BSE_ENABLE_PROFILER
  // Having the profiler as part of the context allows to
  // profile when working with threads, without having to
  // synchronize the profiler itself.
  bse::Profiler* getProfiler() const { return m_profiler; }
#endif

private:
  AIContext* m_aiContext;

#ifdef BSE_ENABLE_PROFILER
  bse::Profiler* m_profiler;
#endif
};


/**
 * \brief Context for the AI world.
 */
class AIContext
{
public:
  AIContext(bse::UInt contextId, bse::UInt maxNumGraphNodes);
  virtual ~AIContext();

  PathFinderContext* pathFinderContext;
  bse::UInt maxNumGraphNodes;

  bse::UInt m_contextId;
};


/**
 *  \brief Provides tasks with the right context to execute.
 *  A context manager instance should be part of the virtual world
 *  and it should be sized with at least a context per each thread
 *  that can use the world in parallel.
 *  Contexts can waste a lot of memory, especially if they are
 *  pre-sized to avoid memory allocations.
 */
class AIContextManager
{
public:
  AIContextManager(bse::UInt numberOfContexts, bse::UInt maxNumGraphNodes);
  virtual ~AIContextManager();

  /// \brief Blocks the caller until a context is available, then returns a context
  AIContext* acquireContext();

  /// \brief Returns a context to the manager, so it can be used by other threads.
  void releaseContext(AIContext* context);

  void reset(bse::UInt numContexts);

#ifdef BSE_ENABLE_PROFILER
  void clearProfilerData();
  bse::ProfilerTask* getProfilerData() const;
#endif // BSE_ENABLE_PROFILER

protected:
  typedef std::vector<AIContext*> ContextsList;
  ContextsList  m_contexts;
  bse::UInt       m_numberOfContexts;
  bse::UInt       m_maxNumGraphNodes;

private:
  std::mutex m_contextMut;

  bse::Event m_contextReady;
};

}
}

#endif // _BSE_AICONTEXT_H_INCLUDED
