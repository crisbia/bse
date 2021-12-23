#ifndef _BSE_PROFILINGTOOLS_INCLUDED
#define _BSE_PROFILINGTOOLS_INCLUDED


#define BSE_ENABLE_PROFILERx // comment before delivery.

#ifdef BSE_ENABLE_PROFILER
# define BSE_ENABLE_PROFILER_DYNAMICS
# ifdef BSE_ENABLE_PROFILER_DYNAMICS
#   define BSE_ENABLE_PROFILER_VELOCITIESSOLVER
#   define BSE_ENABLE_PROFILER_POSITIONSSOLVER
#   define BSE_ENABLE_PROFILER_VELOCITIESINTEGRATION
#   define BSE_ENABLE_PROFILER_POSITIONSINTEGRATION
# endif
# define BSE_ENABLE_PROFILER_COLLISION
# ifdef BSE_ENABLE_PROFILER_COLLISION
#   define BSE_ENABLE_PROFILER_BROADPHASE
#   define BSE_ENABLE_PROFILER_NARROWPHASE
# endif
#endif

#include "bseTypes.h"
#include <map>
#include <vector>

#include <stdint.h>

namespace bse
{

//---------------------------------------------------------------------------------------------------------------------
class GenericTimer
{
public:
  GenericTimer();
  ~GenericTimer() { m_isRunning = false; }
  void startTimer();
  const float stopTimer(); // stop and return the elapsed time
  const float getElapsedTime() const;

protected:
#if defined(WIN32)
	LARGE_INTEGER m_startTime;
	LARGE_INTEGER m_frequency;
#else
	uint64_t m_startTime;
#endif

  float m_elapsedTime;
  float m_frequencyMultiplier;
  bool m_isRunning;
};

//---------------------------------------------------------------------------------------------------------------------
class ProfilerTask
{
public:
  ProfilerTask(ProfilerTask* parent, const char* name);
  ~ProfilerTask();
  const char* getName() const { return m_name; }
  void clear();
  void start();
  void stop();
  void getSubTasks(std::vector<ProfilerTask*>& subTasks) const;
  ProfilerTask* getSubTask(const char* name) const;
  ProfilerTask* getParent() const { return m_parent; }

  typedef std::map<const char*, ProfilerTask*> ProfilerTasksMap;
  ProfilerTasksMap m_children;

private:
  void addSubTask(ProfilerTask* subTask);
  void removeSubTask(ProfilerTask* subTask);

  ProfilerTask* m_parent;
  const char*   m_name;

public:
  GenericTimer m_timer;
  bse::UInt m_numberOfCalls;
  float m_totalTime;
};

//---------------------------------------------------------------------------------------------------------------------
class Profiler
{
public:
  Profiler();
  ~Profiler();
  void clear();
  ProfilerTask* start(const char* name);
  void stop(ProfilerTask* task);
  ProfilerTask* getRoot() const { return m_root; }

  ProfilerTask* m_root;
  ProfilerTask* m_currentTask;
  ProfilerTask* m_currentRoot;
};

//---------------------------------------------------------------------------------------------------------------------
class ProfileTaskWrapper
{
public:
  ProfileTaskWrapper(Profiler* profiler, const char* taskName) :
    m_profiler(profiler)
  {
    m_task = m_profiler->start(taskName);
  }

  ~ProfileTaskWrapper()
  {
    m_profiler->stop(m_task);
  }

private:
  Profiler* m_profiler;
  ProfilerTask* m_task;
};

}

#endif // _BSE_PROFILINGTOOLS_INCLUDED
