#include "bseTask.h"
#include "bseProfilingTools.h"
#include "bseThreadingTools.h"

#include <thread>
#include <mutex>

namespace bse
{


//---------------------------------------------------------------------------------------------------------------------
// simple task scheduler implementation
// this allows to specify the number of threads and an affinity mask
class SimpleTaskScheduler : public TaskScheduler
{
public:
  SimpleTaskScheduler(bse::UInt numWorkerThreads, bse::UInt puMask);
  virtual ~SimpleTaskScheduler();

  virtual void scheduleTask(Task* task);
  virtual void waitForAllTasks();
  virtual bse::UInt getNumWorkerThreads() const { return m_numWorkerThreads; }

private:
  // waits for work to do from the scheduler and the execute it
  static void *threadFunc( void *ptr );

  void *workerThreadFunc();

  void shutDownThreads();

  bse::UInt m_numWorkerThreads;
  bse::UInt m_puMask; // affinity mask.

  std::vector<std::thread> m_workerThreads;

  bse::Event m_jobReadyEvent;
  bse::Event m_shutDownEvent;
  std::mutex m_tasksQueueMut;

  TasksList m_tasksQueue;
  std::atomic_int m_numActiveWorkers;
};

//---------------------------------------------------------------------------------------------------------------------
TaskScheduler *initDefaultScheduler(bse::UInt numWorkerThreads, bse::UInt puMask)
{
  return new SimpleTaskScheduler(numWorkerThreads, puMask);
}

//---------------------------------------------------------------------------------------------------------------------
void destroyDefaultScheduler(TaskScheduler *scheduler)
{
  delete ((SimpleTaskScheduler*)scheduler);
}

//---------------------------------------------------------------------------------------------------------------------
SimpleTaskScheduler::SimpleTaskScheduler(bse::UInt numWorkerThreads, bse::UInt puMask) :
    m_numWorkerThreads(numWorkerThreads),
    m_puMask(puMask),
    m_workerThreads(0),
    m_numActiveWorkers(0)
{
  // This allows to specify 0 thread. It means that all the work is done in the user thread
  if (m_numWorkerThreads>0)
  {
    // create worker threads
    for (bse::UInt iThread=0; iThread<m_numWorkerThreads; ++iThread)
    {
      std::thread t(SimpleTaskScheduler::threadFunc, (void*)this);
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------
SimpleTaskScheduler::~SimpleTaskScheduler()
{
  shutDownThreads();
}

//---------------------------------------------------------------------------------------------------------------------
void SimpleTaskScheduler::shutDownThreads()
{
  // TODO careful, this event might need to be auto reset, otherwise it wouldn't shutdown all the threads
  for (bse::UInt iThread=0; iThread<m_numWorkerThreads; ++iThread)
  {
    m_shutDownEvent.set();
    m_workerThreads[iThread].join();
  }

  m_numWorkerThreads = 0;
  m_workerThreads.clear();
}

//---------------------------------------------------------------------------------------------------------------------
// thread function used by all the threads
void* SimpleTaskScheduler::threadFunc(void* userData)
{
  SimpleTaskScheduler* scheduler = (SimpleTaskScheduler*)userData;
  return scheduler->workerThreadFunc();
}

//---------------------------------------------------------------------------------------------------------------------
void *SimpleTaskScheduler::workerThreadFunc()
{
  bool goOn = true;
  while (goOn)
  {
    if (m_shutDownEvent.tryWait(1))
    {
      break;
    }

    if (!m_jobReadyEvent.tryWait(1))
    {
 //     continue;
    }

    Task* jobToExecute = 0;

    {
      std::lock_guard<std::mutex> lg(m_tasksQueueMut);

      if (m_tasksQueue.empty())
      {
  //      m_jobReadyEvent.reset();
      }
      else
      {
        //interlocked_increment(m_numActiveWorkers, 1);
        jobToExecute = m_tasksQueue.front();
        // BSE_ASSERT(jobToExecute);
        m_tasksQueue.pop_front();
      }
    }

    if (jobToExecute)
    {
      std::atomic_fetch_add(&m_numActiveWorkers, 1);
      jobToExecute->compute();
      std::atomic_fetch_sub(&m_numActiveWorkers, 1);
    }
  }

  return 0;
}

//---------------------------------------------------------------------------------------------------------------------
void SimpleTaskScheduler::scheduleTask(Task* task)
{
  std::lock_guard<std::mutex> lg(m_tasksQueueMut);
  m_tasksQueue.push_back(task);
}

//---------------------------------------------------------------------------------------------------------------------
void SimpleTaskScheduler::waitForAllTasks()
{
  // use the main thread as a worker while waiting
  bool goOn = true;
  while (goOn)
  {
    Task* jobToExecute = 0;

    {
      std::lock_guard<std::mutex> lg(m_tasksQueueMut);

      if (m_tasksQueue.empty())
      {
        //m_jobReadyEvent.reset();
      }
      else
      {
        jobToExecute = m_tasksQueue.front();
        m_tasksQueue.pop_front();
      }
    }

    if (jobToExecute)
    {
      std::atomic_fetch_add(&m_numActiveWorkers, 1);
      jobToExecute->compute();
      std::atomic_fetch_sub(&m_numActiveWorkers, 1);
    }
    else
    {
      goOn = false;
      continue;
    }
  }

  // wait for all thread to finish their jobs

  int v = 0;
	while (!std::atomic_compare_exchange_weak(&m_numActiveWorkers,&v,0));
}

}

