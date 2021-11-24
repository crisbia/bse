#include "bseProfilingTools.h"
#include "bseCommons.h"
#include <string.h>

#include <sys/time.h>

namespace bse
{

//---------------------------------------------------------------------------------------------------------------------
GenericTimer::GenericTimer()
{
    m_frequencyMultiplier = 0;
#if defined(WIN32)
  QueryPerformanceFrequency(&m_frequency);
  if (m_frequency.QuadPart == 0)
    m_frequencyMultiplier = 0.0f;
  else
    m_frequencyMultiplier = float(1000.0f / m_frequency.QuadPart);
#endif
}

//---------------------------------------------------------------------------------------------------------------------
void GenericTimer::startTimer()
{
#if defined(WIN32)
  QueryPerformanceCounter(&m_startTime);
#else
  timeval tv;
  gettimeofday(&tv, 0 );
  m_startTime = tv.tv_sec * 1000000 + tv.tv_usec;
#endif

  m_elapsedTime = 0.0f;
  m_isRunning = true;
}

//---------------------------------------------------------------------------------------------------------------------
const float GenericTimer::stopTimer()
{
#if defined(WIN32)
  LARGE_INTEGER endTime;
  QueryPerformanceCounter(&endTime);
  m_elapsedTime = float( (endTime.QuadPart - m_startTime.QuadPart) * m_frequencyMultiplier );
#else
  timeval tv;
  gettimeofday(&tv, 0 );
  uint64_t endTime = tv.tv_sec * 1000000 + tv.tv_usec;
  m_elapsedTime = float( (endTime - m_startTime) * 0.001f );
#endif
  m_isRunning = false;
  return m_elapsedTime;
}

//---------------------------------------------------------------------------------------------------------------------
const float GenericTimer::getElapsedTime() const
{
  // when the timer is not running return the last elapsed
  float elapsedTime = m_elapsedTime;
  if (m_isRunning)
  {
#if 0
    // when the timer is running, get the timing and return it
    timeval tv;
    gettimeofday(&tv, 0 );
    uint64_t endTime = tv.tv_sec * 1000000 + tv.tv_usec;
    elapsedTime = float( (endTime - m_startTime) * 0.001f );
#endif
  }
  return elapsedTime;
}

//---------------------------------------------------------------------------------------------------------------------
ProfilerTask::ProfilerTask(ProfilerTask* parent, const char* name) :
  m_parent(parent),
  m_name(name),
  m_numberOfCalls(0),
  m_totalTime(0.0f)
{
  if (m_parent)
  {
    m_parent->addSubTask(this);
  }
}

//---------------------------------------------------------------------------------------------------------------------
ProfilerTask::~ProfilerTask()
{
  for (ProfilerTasksMap::iterator iter = m_children.begin(); iter != m_children.end(); ++iter)
  {
    delete (iter->second);
  }
}

//---------------------------------------------------------------------------------------------------------------------
void ProfilerTask::clear()
{
  for (ProfilerTasksMap::iterator iter = m_children.begin(); iter != m_children.end(); ++iter)
  {
    (iter->second)->clear();
  }

  m_numberOfCalls = 0;
  m_totalTime = 0;
}

//---------------------------------------------------------------------------------------------------------------------
void ProfilerTask::start()
{
  ++m_numberOfCalls;
  m_timer.startTimer();
}

//---------------------------------------------------------------------------------------------------------------------
void ProfilerTask::stop()
{
  m_totalTime += m_timer.stopTimer();
}

//---------------------------------------------------------------------------------------------------------------------
void ProfilerTask::addSubTask(ProfilerTask* subTask)
{
  BSE_ASSERT(subTask);
  BSE_ASSERT(m_children.find(subTask->getName()) == m_children.end()); // Don't add twice.
  m_children.insert(std::make_pair(subTask->getName(), subTask));
}

//---------------------------------------------------------------------------------------------------------------------
void ProfilerTask::removeSubTask(ProfilerTask* subTask)
{
  BSE_ASSERT(subTask);
  m_children.erase(subTask->getName());
}

//---------------------------------------------------------------------------------------------------------------------
void ProfilerTask::getSubTasks(std::vector<ProfilerTask*>& subTasks) const
{
  for (ProfilerTasksMap::const_iterator iter = m_children.begin(); iter != m_children.end(); ++iter)
  {
    subTasks.push_back(iter->second);
  }
}

//---------------------------------------------------------------------------------------------------------------------
ProfilerTask* ProfilerTask::getSubTask(const char* name) const
{
  for (ProfilerTasksMap::const_iterator iter = m_children.begin(); iter != m_children.end(); ++iter)
  {
    if (strcmp(iter->first, name)==0)
    {
      return iter->second;
    }
  }
  return 0;
}

//---------------------------------------------------------------------------------------------------------------------
Profiler::Profiler() :
  m_currentTask(0)
{
  m_root = new ProfilerTask(0, "Profiler");
  m_currentRoot = m_root;
}

//---------------------------------------------------------------------------------------------------------------------
Profiler::~Profiler()
{
  delete m_root;
  m_currentTask = 0;
  m_currentRoot = 0;
}

//---------------------------------------------------------------------------------------------------------------------
void Profiler::clear()
{
  m_root->clear();
  m_currentRoot = m_root;
  m_currentTask = 0;
}

//---------------------------------------------------------------------------------------------------------------------
ProfilerTask* Profiler::start(const char* name)
{
  BSE_ASSERT(m_currentRoot);
  ProfilerTask* task = m_currentRoot->getSubTask(name);
  if (!task)
  {
    task = new ProfilerTask(m_currentRoot, name);
  }

  task->start();
  m_currentRoot = task;
  return task;
}

//---------------------------------------------------------------------------------------------------------------------
void Profiler::stop(ProfilerTask* task)
{
  BSE_ASSERT(task);
  task->stop();
  m_currentRoot = task->getParent();
}

}
