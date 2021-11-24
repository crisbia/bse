#ifndef _BSE_KINEMATICAGENT_H_INCLUDED
#define _BSE_KINEMATICAGENT_H_INCLUDED

#include "bseAgent.h"
#include "bseMath.h"

namespace bse
{
namespace ai
{

class KinematicAgent : public Agent
{
public:
  KinematicAgent()
  {
    unlockX();
    unlockY();
  }

  const bse::Vec2 getPosition() const { return m_position; }
  const bse::Vec2 getVelocity() const { return m_velocity; }
  void updatePosition(const bse::Vec2& pos) { m_position = pos; }
  void updateVelocity(const bse::Vec2& vel) { m_velocity = vel; }

  // note: this stuff could be probably abstracted for agents... but, dynamic agents allow filtering???
  void lockX() { m_xLocked = true; }
  void lockY() { m_yLocked = true; }
  void unlockX() { m_xLocked = false; }
  void unlockY() { m_yLocked = false; }
  const bool isXLocked() const { return m_xLocked; }
  const bool isYLocked() const { return m_yLocked; }

  void pursue(KinematicAgent* targetAgent, bse::Real deltaTime, bse::Real anticipation)
  {
    bse::Vec2 tPos = targetAgent->getPosition();
    bse::Vec2 tVel = targetAgent->getVelocity();
    bse::Vec2 normal = tVel;
    normal.normalize();

    // try to anticipate where the target is going to be at the next frame
    tVel *= deltaTime;
    normal *= anticipation;
    bse::Vec2 newPos = tPos + tVel;// + normal; 

    if (!isXLocked())
      m_position.x = newPos.x;
    if (!isYLocked())
      m_position.y = newPos.y;
  }
protected:
  bool m_xLocked;
  bool m_yLocked;

  bse::Vec2 m_position;
  bse::Vec2 m_velocity;
};

}
}

#endif // _BSE_KINEMATICAGENT_H_INCLUDED
