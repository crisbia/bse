#ifndef _BSE_TASK_H_INCLUDED
#define _BSE_TASK_H_INCLUDED

#include <deque>

#include "bseTypes.h"

namespace bse
{

class Task;
class TaskScheduler;
class SimpleTaskScheduler;

//---------------------------------------------------------------------------------------------------------------------
class Task
{
public:
  virtual void compute() = 0;
};


typedef std::deque<Task*> TasksList;

//---------------------------------------------------------------------------------------------------------------------
// generic task scheduler
class TaskScheduler
{
public:
  virtual ~TaskScheduler() {}
  
  // setup a task to be scheduled
  virtual void scheduleTask(Task* task) = 0;
  
  // wait for all the tasks to be completed. blocking.
  virtual void waitForAllTasks() = 0;

  // return the number of worker threads
  virtual bse::UInt getNumWorkerThreads() const = 0;
};

//---------------------------------------------------------------------------------------------------------------------
TaskScheduler *initDefaultScheduler(bse::UInt numWorkerThreads, bse::UInt puMask);

//---------------------------------------------------------------------------------------------------------------------
void destroyDefaultScheduler(TaskScheduler *scheduler);

}

#endif // _BSE_TASK_H_INCLUDED
