#include "bseMaterial.h"
#include <cmath>

namespace bse
{
namespace phx
{

bse::Real Material::combineFriction(const Material* material1, const Material* material2)
{
  bse::Real frictionValue = 0;
  
  bse::Real friction1 = material1->getFriction();
  bse::Real friction2 = material2->getFriction();
  FrictionMode frictionMode1 = material1->getFrictionMode();
  FrictionMode frictionMode2 = material2->getFrictionMode();

  // handle priorities
  FrictionMode frictionMode = frictionMode1;
  if (frictionMode2 < frictionMode1)
    frictionMode = frictionMode2;

  switch (frictionMode)
  {
  case BSE_FRICTIONMODE_ADD:
    frictionValue = friction1 + friction2;
    break;
  case BSE_FRICTIONMODE_MULT:
    frictionValue = friction1 * friction2;
    break;
  case BSE_FRICTIONMODE_MULTSQRT:
    frictionValue = std::sqrt(friction1 + friction2);
    break;
  default:
    break;
  }

  return frictionValue;
}

bse::Real Material::combineRestitution(const Material* material1, const Material* material2)
{
  bse::Real restitutionValue = 0;

  bse::Real restitution1 = material1->getRestitution();
  bse::Real restitution2 = material2->getRestitution();
  RestitutionMode restitutionMode1 = material1->getRestitutionMode();
  RestitutionMode restitutionMode2 = material2->getRestitutionMode();

  RestitutionMode restitutionMode = restitutionMode1;
  if (restitutionMode2 < restitutionMode1)
    restitutionMode = restitutionMode2;

  switch (restitutionMode)
  {
  case BSE_RESTITUTIONMODE_ADD:
    restitutionValue = restitution1 + restitution2;
    break;
  case BSE_RESTITUTIONMODE_MULT:
    restitutionValue = restitution1 * restitution2;
    break;
  case BSE_RESTITUTIONMODE_MULTSQRT:
    restitutionValue = sqrt(restitution1 * restitution2);
    break;
  default:
    break;
  }

  return restitutionValue;
}

}
}