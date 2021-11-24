#ifndef _BSE_THREADEDCONTEXT_H_INCLUDED
#define _BSE_THREADEDCONTEXT_H_INCLUDED


#include "bseTypes.h"
#include "bseThreadingTools.h"
#include <vector>
#include <mutex>

namespace bse
{

/**
 *  \brief Provides tasks with the right context to execute.
 *  A context manager instance should be part of the virtual world
 *  and it should be sized with at least a context per each thread
 *  that can use the world in parallel.
 *  Contexts can waste a lot of memory, especially if they are
 *  pre-sized to avoid memory allocations.
 */

template<typename ContextType>
class ContextManager
{
public:
  ContextManager(bse::UInt numberOfContexts) :
      m_numberOfContexts(numberOfContexts)
  {
    std::lock_guard<std::mutex> lg(m_contextMut);

    for (bse::UInt i=0; i<numberOfContexts; ++i)
      m_contexts.push_back(new ContextType());

    m_contextReady.set();
  }

  ~ContextManager()
  {
    std::lock_guard<std::mutex> lg(m_contextMut);

    while (!m_contexts.empty())
    {
      ContextType* context = m_contexts.back();
      m_contexts.pop_back();
      delete context;
    }
  }

  /**
   *  \brief Blocks the caller until a context is available, then returns a context
   *  to work on.
   */
  ContextType* getContext()
  {
    m_contextReady.wait();

    std::lock_guard<std::mutex> lg(m_contextMut);

    ContextType* context = m_contexts.back();
    m_contexts.pop_back();

    return context;
  }

  /**
   *  \brief Returns a context to the manager, so it can be used by other threads.
   */
  void releaseContext(ContextType* context)
  {
    if (context)
    {
      std::lock_guard<std::mutex> lg(m_contextMut);
  
      // restore context's initial conditions.
      context->clear();
      m_contexts.push_back(context);
    }
  }

protected:
  typedef std::vector<ContextType*> ContextsList;
  ContextsList  m_contexts;
  bse::UInt       m_numberOfContexts;

private:
  std::mutex m_contextMut;

  bse::Event m_contextReady;
};

}

#endif // _BSE_THREADEDCONTEXT_H_INCLUDED





