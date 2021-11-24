#ifndef _BSE_THREADING_TOOLS_H_INCLUDED
#define _BSE_THREADING_TOOLS_H_INCLUDED

#include <mutex>
#include <condition_variable>

namespace bse
{

/**
 * \brief Monitor
 */

class Monitor
{
public:
  friend class MonitorLocker;
  Monitor();
  virtual ~Monitor();
protected:
  void lock();
  void release();

private:
  std::mutex m_mutex;
};

/**
 * \brief helper class to help locking/releasing a monitor using the RAII approach
 */
class MonitorLocker
{
public:
  MonitorLocker(Monitor* monitor);
  ~MonitorLocker();

private:
  Monitor* m_monitor;
};


class Event
{
public:
  Event(bool signalled = false, bool ar = false) :
    m_auto(ar),
    m_signalled(signalled)
  {
  }

  ~Event()
  {
  }

  void set()
  {
    std::lock_guard<std::mutex> l(m_mutex);

    // only set and signal if we are unset
    if (m_signalled == false)
    {
      m_signalled = true;

      m_cond.notify_all();
    }
  }

  void wait()
  {
    std::lock_guard<std::mutex> l(m_mutex);

    // while (m_signalled == false)
    // {
    //   m_cond.wait(l);
    // }

    // if we're an autoreset event, auto reset
    if (m_auto)
    {
      m_signalled = false;
    }
  }

  bool tryWait(int timeOutMsec)
  {
    std::lock_guard<std::mutex> l(m_mutex);

    // if (!m_cond.wait(l, timeOutMsec))
    // {
    //   return false;
    // }

//    while (m_signalled == false)
//    {
//      m_cond.wait(l);
//    }

    // if we're an autoreset event, auto reset
    if (m_auto)
    {
      m_signalled = false;
    }

    m_mutex.unlock();

    return true;
  }

  void reset()
  {
    std::lock_guard<std::mutex> l(m_mutex);

    m_signalled = false;
  }

private:
  std::mutex m_mutex;
  std::condition_variable m_cond;
  bool m_auto;
  bool m_signalled;
};

}

#endif // _BSE_THREADING_TOOLS_H_INCLUDED
