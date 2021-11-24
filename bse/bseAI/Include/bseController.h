#ifndef _BSE_CONTROLLER_H_INCLUDED
#define _BSE_CONTROLLER_H_INCLUDED

class bseAgent;

/// Base class for implementation of AI controllers.
// A controller needs to be attached to an agent. An agent doesn't need to be tied to a specific controller
// statically. The controller can be modified on-the-fly to implement different behaviours.
// The agent updates its own controller, so there's no need to keep track of the controllers. The logic
// of the agent itself takes care of deciding which controller is active.

class bseController
{
public:
  bseController
  bseAgent *getAgent() const { return m_agent; }

  virtual void update(bseReal dt) = 0;
protected:
  // Agent which this controller drives.
  bseAgent *m_agent;
};

#endif // _BSE_CONTROLLER_H_INCLUDED
