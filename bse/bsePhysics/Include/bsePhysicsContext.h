#ifndef _BSE_PHYSICSCONTEXT_H_INCLUDED
#define _BSE_PHYSICSCONTEXT_H_INCLUDED

#include "bseTypes.h"
#include "bseThreadedContexts.h"

namespace bse
{
namespace phx
{

//---------------------------------------------------------------------------------------------------------------------
/**
 *  \brief Context for physics scenes
 */
class PhysicsContext
{
public:
  PhysicsContext()
  {

  }
};

//---------------------------------------------------------------------------------------------------------------------
class PhysicsContextManager : public ContextManager<PhysicsContext>
{
public:
  PhysicsContextManager(bse::UInt numberOfContexts) : ContextManager<PhysicsContext>(numberOfContexts) 
  {

  }

  ~PhysicsContextManager()
  {

  }
};

}
}

#endif // _BSE_PHYSICSCONTEXT_H_INCLUDED
