#include "bseThreadingTools.h"

namespace bse
{

Monitor::Monitor()
{

}

Monitor::~Monitor()
{

}

void Monitor::lock()
{

}

void Monitor::release()
{

}


MonitorLocker::MonitorLocker(Monitor* monitor) : m_monitor(monitor)
{
  m_monitor->lock();
}

MonitorLocker::~MonitorLocker()
{
  m_monitor->release();
}

}